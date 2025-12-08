#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"

#include "control/Controller.h"
#include "visualization/PolyscopeRenderer.h"
#include <cstdlib>
#include <glm/glm.hpp>
#include <glm/gtx/compatibility.hpp>
#include <numeric>


PolyscopeRenderer::PolyscopeRenderer(Simulation* sim)
    : sim(sim) {
}

void PolyscopeRenderer::initialize() {
    polyscope::init();
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;

    std::vector<glm::vec3> positions;
    for (auto& d : sim->getDrones()) {
        positions.push_back(d.getPosition());
    }

    cloud = polyscope::registerPointCloud("drones", positions);

    std::vector<glm::vec3> colors(positions.size(), glm::vec3(0.0f, 1.0f, 0.2f));
    cloud->addColorQuantity("statusColor", colors);
    cloud->setAllQuantitiesEnabled(true);

    sim->getNetwork().setVisCallback([this](const Message& msg,
        double sendT,
        double recvT)
        {
            // Determine packet kind
            PacketKind kind = (msg.type == MessageType::HEARTBEAT)
                ? PacketKind::Heartbeat
                : PacketKind::Broadcast;

            // Sender position
            glm::vec3 srcPos =
                sim->getDrones()[msg.senderId].getPosition();

            // Receiver position
            glm::vec3 dstPos;

            if (msg.receiverId == -1) {
                // Broadcast packet -> simple upward spike for now
                dstPos = srcPos + glm::vec3(0, 0.4f, 0);
            }
            else {
                dstPos = sim->getDrones()[msg.receiverId].getPosition();
            }

            // Store the packet for animation
            activePackets.push_back(ActivePacket{
                srcPos,
                dstPos,
                sendT,
                recvT,
                kind,
                "pkt_" + std::to_string(packetCounter++)
                });
        });

    // Initialize neighbor link network
    buildNeighborCurveNetwork();

    // Set callback
    polyscope::state::userCallback = [this]() {
        sim->step();

        updateDronePositions();
        updateDroneColors();
        updateNeighborLinks();
        drawPackets();

        drawUI();
        };
}

void PolyscopeRenderer::renderLoop() {
    polyscope::show();
}

//
// ======================================================
//   UI Panel
// ======================================================
void PolyscopeRenderer::drawUI() {

    ImGui::Begin("Info");
	ImGui::Text("Simulation Time: %.2f s", sim->getTime());
	ImGui::Text("Num Drones: %d", (int)sim->getDrones().size());    
	ImGui::Text("Active Packets: %d", (int)activePackets.size());

    // --- Consistency Check ---
    auto [good, gid, memCount] = sim->checkConsistency();

    ImVec4 col = good ? ImVec4(0, 1, 0, 1) : ImVec4(1, 0, 0, 1);
    std::string label = good ? "CONSISTENT" : "INCONSISTENT";

    ImGui::TextColored(col, "Membership Consistency: %s", label.c_str());
    ImGui::Text("GroupId: %d  |  Members: %d", gid, memCount);
    double t = sim->getTime();
    double hb = sim->getConfig().heartbeatInterval;
    int intervals = (hb > 0 ? int(t / hb) : 0);
    ImGui::Text("Heartbeat Intervals Elapsed: %d", intervals);

    ImGui::End();

    ImGui::Begin("Controls");

    if (ImGui::Button("Randomize Drone Positions")) {

        std::vector<int> perm(sim->getDrones().size());
        std::iota(perm.begin(), perm.end(), 0);

        // Shuffle using GLOBAL_RNG
        std::shuffle(perm.begin(), perm.end(), GLOBAL_RNG);
        // Create drones
        float radius = sim->getConfig().numDrones * sim->getConfig().droneSpacing / (2.0f * 3.14159f);
        float angleStep = 2.0f * 3.14159f / sim->getConfig().numDrones;

        for (int i = 0; i < sim->getConfig().numDrones; ++i) {
            float angle = i * angleStep;

            glm::vec3 pos{
                radius * std::cos(angle),
                0.0f,
                radius * std::sin(angle)
            };

			sim->getDrones()[perm[i]].setPosition(pos);
        }

        // Rebuild neighbor graph dynamically
        //for (auto& d : sim->getDrones()) {
        //    auto mem = d.getMembership();
        //    mem.updateClosestNeighbors(mem.getActiveMembers());
        //}

        // Update visuals immediately
        updateDronePositions();
        updateNeighborLinks();
    }

    if (ImGui::Button("Force Closest-Neighbor Reconfig")) {
        // For every UP drone, recompute closest neighbors
        for (auto& d : sim->getDrones()) {
            if (d.getStatus() == DroneStatus::UP) {
                d.getMembership().updateClosestNeighbors(d.getPosition());
            }
        }

        // Rebuild neighbor curve network so links reflect new neighbors
        polyscope::removeCurveNetwork("neighbors");
        buildNeighborCurveNetwork();
    }

    ImGui::Checkbox("Show Neighbor Links", &showNeighborLinks);
    ImGui::Checkbox("Show Heartbeats", &showHeartbeats);
    ImGui::Checkbox("Show Broadcasts", &showBroadcasts);

    auto& config = sim->getConfig();
    if (ImGui::CollapsingHeader("Simulation Config")) {

        auto& config = sim->getConfig();

        ImGui::SeparatorText("Timing");
        ImGui::InputDouble("Time Step", &config.timeStep);
        ImGui::InputDouble("Heartbeat Interval", &config.heartbeatInterval);
        ImGui::InputDouble("Datagram Timeout Delta", &config.deltaSmall);
        ImGui::InputDouble("Broadcast Timeout Delta", &config.deltaLarge);

        const char* colorModeNames[] = {
        "Status (UP/DOWN)",
        "Drone Show (Formation)"
        };
        ImGui::Combo("Drone Color Mode", (int*) & colorMode,
            colorModeNames, IM_ARRAYSIZE(colorModeNames));

        ImGui::SeparatorText("Network Latency");
        ImGui::InputDouble("Base Latency", &config.baseLatency);
        ImGui::InputDouble("Alpha Latency (load growth)", &config.alphaLatency);
        ImGui::InputDouble("Distance Latency Factor", &config.distanceLatencyFactor);

        ImGui::SeparatorText("Loss Model");
        ImGui::Checkbox("Enable Burst Loss", &config.enableBursts);
        ImGui::InputDouble("Burst Start Prob", &config.burstStartProb);
        ImGui::InputDouble("Burst Drop Prob", &config.burstDropProb);
        ImGui::InputDouble("Burst Duration", &config.burstDuration);
        ImGui::InputDouble("Base Loss Prob", &config.baseLossProb);
        ImGui::InputDouble("Beta Loss", &config.betaLoss);
        ImGui::InputDouble("Duplication Prob", &config.duplicationProb);

        ImGui::SeparatorText("Seeding");
        ImGui::InputInt("Seed", (int*)&config.seed);

        ImGui::Separator();
        if (ImGui::Button("Restart Simulation")) {
            reseedRNG(config.seed);
            sim = new Simulation(config);
            rebuildAllVisuals();
        }
    }


    ImGui::End();
}

void PolyscopeRenderer::updateDroneColors() {
    std::vector<glm::vec3> colors;
    colors.reserve(sim->getDrones().size());

    if ((int)colorMode == 0 || sim->getController() == nullptr) {
        // Original: UP = green, DOWN = red.
        for (auto& d : sim->getDrones()) {
            DroneStatus st = d.getStatus();
            if (st == DroneStatus::UP) {
                colors.push_back({ 0.0f, 1.0f, 0.2f });
            }
            else {
                colors.push_back({ 1.0f, 0.1f, 0.1f });
            }
        }
    }
    else {
        // Drone show mode: use controller's formation-based colors.
        auto* ctrl = sim->getController();
        for (auto& d : sim->getDrones()) {
            colors.push_back(ctrl->getColorForDrone(d.getId(), d.getStatus()));
        }
    }

    cloud->addColorQuantity("statusColor", colors);
}

//
// ======================================================
//   POSITION + COLOR UPDATES
// ======================================================
void PolyscopeRenderer::updateDronePositions() {
    std::vector<glm::vec3> positions;
    positions.reserve(sim->getDrones().size());
    for (auto& d : sim->getDrones()) {
        positions.push_back(d.getPosition());
    }
    cloud->updatePointPositions(positions);
}


//
// ======================================================
//   NEIGHBOR LINK VISUALIZATION
// ======================================================
void PolyscopeRenderer::buildNeighborCurveNetwork() {
    std::vector<glm::vec3> pts;
    std::vector<std::array<size_t, 2>> edges;

    auto& drones = sim->getDrones();
    pts.reserve(drones.size());

    // Points
    for (auto& d : drones) {
        pts.push_back(d.getPosition());
    }

    // Edges: (i -> right neighbor)
    int N = (int)drones.size();
    for (int i = 0; i < N; i++) {
        int right = drones[i].getMembership().getRightNeighbor();
        if (right != -1) {
            edges.push_back({ (size_t)i, (size_t)right });
        }
    }

    neighborNet = polyscope::registerCurveNetwork("neighbors", pts, edges);
    neighborNet->setRadius(0.005f);
    neighborNet->setColor(glm::vec3(0.3f, 0.3f, 1.0f));
}

void PolyscopeRenderer::updateNeighborLinks() {
    if (!showNeighborLinks) {
        if (neighborNet) {
            neighborNet->setEnabled(false);
        }
        return;
    }

    auto& drones = sim->getDrones();

    // Positions
    std::vector<glm::vec3> pts;
    pts.reserve(drones.size());
    for (auto& d : drones) {
        pts.push_back(d.getPosition());
    }

    // Edges from current rightNeighbor
    std::vector<std::array<size_t, 2>> edges;
    int N = (int)drones.size();
    for (int i = 0; i < N; ++i) {
        int right = drones[i].getMembership().getRightNeighbor();
		int left = drones[i].getMembership().getLeftNeighbor();
        if (right != -1) {
            edges.push_back({ (size_t)i, (size_t)right });
        }
        if (left != -1) {
			edges.push_back({ (size_t)i, (size_t)left });
        }
    }

    // Blow away old network and re-register so edges are always current
    if (neighborNet) {
        polyscope::removeCurveNetwork("neighbors");
        neighborNet = nullptr;
    }

    neighborNet = polyscope::registerCurveNetwork("neighbors", pts, edges);
    neighborNet->setRadius(0.005f);
    neighborNet->setColor(glm::vec3(0.3f, 0.3f, 1.0f));
}


//
// ======================================================
//   PACKET VISUALIZATION
// ======================================================
void PolyscopeRenderer::drawPackets() {
    double currentTime = sim->getTime();

    for (auto it = activePackets.begin(); it != activePackets.end(); ) {

        double alpha = (currentTime - it->createdAt) /
            (it->deliverAt - it->createdAt);

        if (alpha >= 1.0) {
            polyscope::removeCurveNetwork(it->handle);
            it = activePackets.erase(it);
            continue;
        }

        // Compute interpolated position
        glm::vec3 pos = glm::mix(it->src, it->dst, (float)alpha);
        glm::vec3 dir = glm::normalize(it->dst - it->src);

        float scale = (it->kind == PacketKind::Broadcast) ? 0.08f : 0.03f;

        std::vector<glm::vec3> pts = {
            pos,
            pos + scale * dir
        };
        std::vector<std::array<size_t, 2>> edges = { {0, 1} };

        // Update or create
        auto net = polyscope::registerCurveNetwork(it->handle, pts, edges);
        net->setRadius(scale * 0.2f);

        if (it->kind == PacketKind::Broadcast)
            net->setColor(glm::vec3(0.1f, 1.0f, 1.0f));
        else
            net->setColor(glm::vec3(0.8f, 0.2f, 1.0f));

        ++it;
    }
}


//
// ======================================================
//  Support for Simulation Restart
// ======================================================
void PolyscopeRenderer::rebuildAllVisuals() {

    // Remove old networks
    polyscope::removeCurveNetwork("neighbors");
    polyscope::removePointCloud("drones");

    for (auto it = activePackets.begin(); it != activePackets.end(); it++) {
        polyscope::removeCurveNetwork(it->handle);
    }
    activePackets.clear();

    // Re-register the visualization callback
    sim->getNetwork().setVisCallback([this](const Message& msg,
        double sendT,
        double recvT)
        {
            // Determine packet kind
            PacketKind kind = (msg.type == MessageType::HEARTBEAT)
                ? PacketKind::Heartbeat
                : PacketKind::Broadcast;

            // Sender position
            glm::vec3 srcPos =
                sim->getDrones()[msg.senderId].getPosition();

            // Receiver position
            glm::vec3 dstPos;

            if (msg.receiverId == -1) {
                // Broadcast packet -> simple upward spike for now
                dstPos = srcPos + glm::vec3(0, 0.4f, 0);
            }
            else {
                dstPos = sim->getDrones()[msg.receiverId].getPosition();
            }

            // Store the packet for animation
            activePackets.push_back(ActivePacket{
                srcPos,
                dstPos,
                sendT,
                recvT,
                kind,
                "pkt_" + std::to_string(packetCounter++)
                });
        });

    // Rebuild point cloud
    std::vector<glm::vec3> positions;
    for (auto& d : sim->getDrones()) {
        positions.push_back(d.getPosition());
    }

    cloud = polyscope::registerPointCloud("drones", positions);

    std::vector<glm::vec3> colors(positions.size(), glm::vec3(0, 1, 0));
    cloud->addColorQuantity("statusColor", colors);

    // Rebuild neighbor links
    buildNeighborCurveNetwork();
}

