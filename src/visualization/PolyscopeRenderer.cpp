#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"

#include "control/Controller.h"
#include "visualization/PolyscopeRenderer.h"
#include "core/RNG.h"

#include <cstdlib>
#include <glm/glm.hpp>
#include <glm/gtx/compatibility.hpp>
#include <numeric>
#include <algorithm>
#include <random>
#include <string>
#include <vector>

// ============================================================================
//  Local state for drone show modes & timeline (kept CPP-only)
// ============================================================================

namespace {

    enum class ShowMode {
        Manual = 0,
        Auto = 1
    };

    struct Cue {
        double time;         // when to start
        int    patternIndex; // which pattern
        double duration;     // transition duration
    };

    ShowMode              gShowMode = ShowMode::Manual;
    std::vector<Cue>      gTimeline;
    int                   gNextCue = 0;
    bool                  gTimelineInitialized = false;
    double                gDefaultTransitionSec = 3.0;
    double                gAutoShowLengthSec = 20.0;  // total length of auto show

    // Build a very simple auto timeline using whatever patterns exist.
    void buildAutoTimeline(Controller* ctrl, double startTime) {
        gTimeline.clear();
        gNextCue = 0;
        gTimelineInitialized = true;

        if (!ctrl) return;
        int patternCount = ctrl->getPatternCount();
        if (patternCount <= 0) return;

        // Simple scheme: start with pattern 0, then walk through all patterns,
        // then back to 0, spreading them evenly over gAutoShowLengthSec.
        std::vector<int> order;
        for (int i = 0; i < patternCount; ++i) order.push_back(i);
        if (patternCount > 1) {
            // End by returning to the first pattern
            order.push_back(0);
        }

        int cueCount = (int)order.size();
        if (cueCount == 0) return;

        double t = 0.0;
        double stride = gAutoShowLengthSec / std::max(1, cueCount - 1);

        for (int i = 0; i < cueCount; ++i) {
            Cue c;
            c.time = t + startTime;
            c.patternIndex = order[i];
            // First cue is instant, others use default duration
            c.duration = (i == 0 ? 0.0 : gDefaultTransitionSec);
            gTimeline.push_back(c);
            t += stride;
        }
    }

} // anonymous namespace


// ============================================================================
//  PolyscopeRenderer implementation
// ============================================================================

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

    // Packet visualization callback
    sim->getNetwork().setVisCallback([this](const Message& msg,
        double sendT,
        double recvT) {
            // Determine packet kind
            PacketKind kind;
            if (msg.type == MessageType::HEARTBEAT) {
                kind = PacketKind::Heartbeat;
            }
            else if (msg.type == MessageType::CONTROL) {
                kind = PacketKind::Control;
            }
            else {
                kind = PacketKind::Broadcast;
            }

            glm::vec3 srcPos = sim->getDrones()[msg.senderId].getPosition();
            glm::vec3 dstPos;

            if (msg.receiverId == -1) {
                // Broadcast packet -> upward spike
                dstPos = srcPos + glm::vec3(0, 0.4f, 0);
            }
            else {
                dstPos = sim->getDrones()[msg.receiverId].getPosition();
            }

            activePackets.push_back(ActivePacket{
                srcPos,
                dstPos,
                sendT,
                recvT,
                kind,
                "pkt_" + std::to_string(packetCounter++)
                });
        });

    // Neighbor links
    buildNeighborCurveNetwork();

    // Main loop callback
    polyscope::state::userCallback = [this]() {
        // Step the simulation
        sim->step();

        // Drone show auto timeline (if enabled)
        if (gShowMode == ShowMode::Auto) {
            auto* ctrl = sim->getController();
            if (ctrl) {
                if (!gTimelineInitialized) {
                    buildAutoTimeline(ctrl, sim->getTime());
                }

                double t = sim->getTime();
                while (gNextCue < (int)gTimeline.size() &&
                    t >= gTimeline[gNextCue].time) {
                    const Cue& c = gTimeline[gNextCue];
                    ctrl->startTransitionTo(c.patternIndex, t, c.duration);
                    gNextCue++;
                }

                if (gNextCue >= (int)gTimeline.size()) {
                    // Restart timeline
                    gNextCue = 0;
                    buildAutoTimeline(ctrl, sim->getTime());
				}
            }
        }

        // Update visuals
        updateDronePositions();
        updateDroneColors();
        updateNeighborLinks();
        drawPackets();

        // UI
        drawUI();
        };
}

void PolyscopeRenderer::renderLoop() {
    polyscope::show();
}

// ============================================================================
//   UI Panel
// ============================================================================

void PolyscopeRenderer::drawUI() {

    // ------------------------------------------------------------------------
    // Info panel (unchanged core info + consistency, heartbeat intervals)
    // ------------------------------------------------------------------------
    ImGui::Begin("Info");
    ImGui::Text("Simulation Time: %.2f s", sim->getTime());
    ImGui::Text("Num Drones: %d", (int)sim->getDrones().size());
    ImGui::Text("Active Packets: %d", (int)activePackets.size());

    // Membership consistency
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

    // ------------------------------------------------------------------------
    // Controls panel (existing stuff + Kill Random Drone button)
    // ------------------------------------------------------------------------
    ImGui::Begin("Controls");

    // Randomize positions on circle
    if (ImGui::Button("Randomize Drone Positions")) {

        std::vector<int> perm(sim->getDrones().size());
        std::iota(perm.begin(), perm.end(), 0);
        std::shuffle(perm.begin(), perm.end(), GLOBAL_RNG);

        float radius = sim->getConfig().numDrones * sim->getConfig().droneSpacing
            / (2.0f * 3.14159f);
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

        updateDronePositions();
        updateNeighborLinks();
    }

    // Force recompute closest neighbors
    if (ImGui::Button("Force Closest-Neighbor Reconfig")) {
        for (auto& d : sim->getDrones()) {
            if (d.getStatus() == DroneStatus::UP) {
                d.getMembership().updateClosestNeighbors(d.getPosition());
            }
        }
        polyscope::removeCurveNetwork("neighbors");
        buildNeighborCurveNetwork();
    }

    // NEW: Kill a random UP drone
    if (ImGui::Button("Kill Random Drone")) {
        std::vector<int> alive;
        alive.reserve(sim->getDrones().size());
        for (auto& d : sim->getDrones()) {
            if (d.getStatus() == DroneStatus::UP) {
                alive.push_back(d.getId());
            }
        }
        if (!alive.empty()) {
            std::uniform_int_distribution<int> dist(0, (int)alive.size() - 1);
            int victimId = alive[dist(GLOBAL_RNG)];
            sim->killDrone(victimId);
        }
    }

    ImGui::Checkbox("Show Neighbor Links", &showNeighborLinks);
    ImGui::Checkbox("Show Heartbeats", &showHeartbeats);
    ImGui::Checkbox("Show Broadcasts", &showBroadcasts);

    // Simulation config (same as before)
    auto& config = sim->getConfig();
    if (ImGui::CollapsingHeader("Simulation Config")) {

        ImGui::SeparatorText("Timing");
        ImGui::InputDouble("Time Step", &config.timeStep);
        ImGui::InputDouble("Heartbeat Interval", &config.heartbeatInterval);
        ImGui::InputDouble("Datagram Timeout Delta", &config.deltaSmall);
        ImGui::InputDouble("Broadcast Timeout Delta", &config.deltaLarge);

        const char* colorModeNames[] = {
            "Status (UP/DOWN)",
            "Drone Show (Formation)"
        };
        ImGui::Combo("Drone Color Mode", (int*)&colorMode,
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

            // Reset drone show timeline as well
            gTimelineInitialized = false;
            gTimeline.clear();
            gNextCue = 0;
        }
    }

    ImGui::End();

    // ------------------------------------------------------------------------
    // Drone Show panel: manual vs auto cueing, color effects tuning
    // ------------------------------------------------------------------------
    ImGui::Begin("Drone Show");

    auto* ctrl = sim->getController();
    int patternCount = ctrl ? ctrl->getPatternCount() : 0;

    if (!ctrl || patternCount == 0) {
        ImGui::TextWrapped("No formation controller or patterns available.");
        ImGui::End();
        return;
    }

    // Mode selection
    int modeInt = (gShowMode == ShowMode::Manual ? 0 : 1);
    ImGui::SeparatorText("Mode");

    ImGui::RadioButton("Manual Control", &modeInt, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Auto Timeline", &modeInt, 1);

    gShowMode = (modeInt == 0 ? ShowMode::Manual : ShowMode::Auto);

    // Shared parameters
    ImGui::SeparatorText("Transition Settings");
    ImGui::InputDouble("Default Transition Duration (s)", &gDefaultTransitionSec);
    ImGui::InputDouble("Auto Show Total Length (s)", &gAutoShowLengthSec);// , 5.0, 120.0);

    // If we tweak auto show length or duration while in Auto, rebuild timeline next frame
    if (ImGui::Button("Rebuild Auto Timeline")) {
        gTimelineInitialized = false;
        gTimeline.clear();
        gNextCue = 0;
    }

    // ------------------------------------------------------------------------
    // Manual mode: user clicks formations & morphs
    // ------------------------------------------------------------------------
    if (gShowMode == ShowMode::Manual) {
        ImGui::SeparatorText("Manual Formation Control");

        static double manualDuration = 3.0;
        ImGui::InputDouble("Manual Morph Duration (s)", &manualDuration);

        int activePat = ctrl->getActivePattern();
        ImGui::Text("Active Pattern: %d", activePat);

        // List patterns with buttons
        for (int i = 0; i < patternCount; ++i) {
            ImGui::PushID(i);
            ImGui::Text("Pattern %d", i);
            ImGui::SameLine();
            if (ImGui::Button("Set Instant")) {
                ctrl->setActivePattern(i);
            }
            ImGui::SameLine();
            if (ImGui::Button("Morph")) {
                ctrl->startTransitionTo(i, sim->getTime(), manualDuration);
            }
            ImGui::PopID();
        }
    }

    // ------------------------------------------------------------------------
    // Auto mode: timeline overview (read-only status)
    // ------------------------------------------------------------------------
    if (gShowMode == ShowMode::Auto) {
        ImGui::SeparatorText("Auto Timeline");

        if (!gTimelineInitialized) {
            ImGui::TextWrapped(
                "Timeline not built yet; it will be created automatically "
                "on the next frame using the available patterns.");
        }
        else {
            double now = sim->getTime();
            ImGui::Text("Time: %.2f s", now);
            ImGui::Text("Cues: %d  |  Next Cue Index: %d",
                (int)gTimeline.size(), gNextCue);

            for (int i = 0; i < (int)gTimeline.size(); ++i) {
                const Cue& c = gTimeline[i];
                ImGui::PushID(i);
                if (i == gNextCue) {
                    ImGui::TextColored(ImVec4(0, 1, 0, 1),
                        ">> Cue %d: t=%.2f, pattern=%d, dur=%.2f",
                        i, c.time, c.patternIndex, c.duration);
                }
                else {
                    ImGui::Text("Cue %d: t=%.2f, pattern=%d, dur=%.2f",
                        i, c.time, c.patternIndex, c.duration);
                }
                ImGui::PopID();
            }
        }

        if (ImGui::Button("Restart Auto Show")) {
            gTimelineInitialized = false;
            gTimeline.clear();
            gNextCue = 0;
            // Also reset active pattern to 0 immediately, for determinism
            ctrl->setActivePattern(0);
        }
    }

    ImGui::End();
}

// ============================================================================
//   POSITION + COLOR UPDATES
// ============================================================================

void PolyscopeRenderer::updateDronePositions() {
    std::vector<glm::vec3> positions;
    positions.reserve(sim->getDrones().size());
    for (auto& d : sim->getDrones()) {
        positions.push_back(d.getPosition());
    }
    cloud->updatePointPositions(positions);
}

void PolyscopeRenderer::updateDroneColors() {
    std::vector<glm::vec3> colors;
    colors.reserve(sim->getDrones().size());

    if ((int)colorMode == 0 || sim->getController() == nullptr) {
        // Status mode: UP = green, DOWN = red.
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
        // Drone show mode: use controller's formation-based colors, plus sparkle.
        auto* ctrl = sim->getController();
        double time = sim->getTime();

        for (auto& d : sim->getDrones()) {
            glm::vec3 base = ctrl->getColorForDrone(d.getId(), d.getStatus());

            // Sparkle / pulsing brightness based on time + id
            float phase = 0.35f * (float)d.getId();
            float sparkle = 0.75f + 0.25f * std::sin(5.0f * (float)time + phase);

            glm::vec3 c = base * sparkle;
            c.x = std::min(c.x, 1.0f);
            c.y = std::min(c.y, 1.0f);
            c.z = std::min(c.z, 1.0f);

            colors.push_back(c);
        }
    }

    cloud->addColorQuantity("statusColor", colors);
}

// ============================================================================
//   NEIGHBOR LINK VISUALIZATION
// ============================================================================

void PolyscopeRenderer::buildNeighborCurveNetwork() {
    std::vector<glm::vec3> pts;
    std::vector<std::array<size_t, 2>> edges;

    auto& drones = sim->getDrones();
    pts.reserve(drones.size());

    for (auto& d : drones) {
        pts.push_back(d.getPosition());
    }

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

    std::vector<glm::vec3> pts;
    pts.reserve(drones.size());
    for (auto& d : drones) {
        pts.push_back(d.getPosition());
    }

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

    if (neighborNet) {
        polyscope::removeCurveNetwork("neighbors");
        neighborNet = nullptr;
    }

    neighborNet = polyscope::registerCurveNetwork("neighbors", pts, edges);
    neighborNet->setRadius(0.005f);
    neighborNet->setColor(glm::vec3(0.3f, 0.3f, 1.0f));
}

// ============================================================================
//   PACKET VISUALIZATION
// ============================================================================

void PolyscopeRenderer::drawPackets() {
    double currentTime = sim->getTime();

    for (auto it = activePackets.begin(); it != activePackets.end(); ) {

        double denom = (it->deliverAt - it->createdAt);
        if (denom <= 0.0) {
            polyscope::removeCurveNetwork(it->handle);
            it = activePackets.erase(it);
            continue;
        }

        double alpha = (currentTime - it->createdAt) / denom;

        if (alpha >= 1.0) {
            polyscope::removeCurveNetwork(it->handle);
            it = activePackets.erase(it);
            continue;
        }

        // Respect UI toggles
        if ((it->kind == PacketKind::Heartbeat && !showHeartbeats) ||
            (it->kind == PacketKind::Broadcast && !showBroadcasts)) {
            ++it;
            continue;
        }

        glm::vec3 pos = glm::mix(it->src, it->dst, (float)alpha);
        glm::vec3 dir = glm::normalize(it->dst - it->src);

        float scale = (it->kind == PacketKind::Broadcast) ? 0.08f : 0.03f;

        std::vector<glm::vec3> pts = {
            pos,
            pos + scale * dir
        };
        std::vector<std::array<size_t, 2>> edges = { {0, 1} };

        auto net = polyscope::registerCurveNetwork(it->handle, pts, edges);
        net->setRadius(scale * 0.2f);

        if (it->kind == PacketKind::Broadcast)
            net->setColor(glm::vec3(0.1f, 1.0f, 1.0f));
        else if (it->kind == PacketKind::Heartbeat)
            net->setColor(glm::vec3(0.8f, 0.2f, 1.0f));
        else
            net->setColor(glm::vec3(1.0f, 0.8f, 0.2f)); // control / other

        ++it;
    }
}

// ============================================================================
//  Support for Simulation Restart
// ============================================================================

void PolyscopeRenderer::rebuildAllVisuals() {

    polyscope::removeCurveNetwork("neighbors");
    polyscope::removePointCloud("drones");

    for (auto it = activePackets.begin(); it != activePackets.end(); it++) {
        polyscope::removeCurveNetwork(it->handle);
    }
    activePackets.clear();

    sim->getNetwork().setVisCallback([this](const Message& msg,
        double sendT,
        double recvT) {
            PacketKind kind;
            if (msg.type == MessageType::HEARTBEAT) {
                kind = PacketKind::Heartbeat;
            }
            else if (msg.type == MessageType::CONTROL) {
                kind = PacketKind::Control;
            }
            else {
                kind = PacketKind::Broadcast;
            }

            glm::vec3 srcPos = sim->getDrones()[msg.senderId].getPosition();
            glm::vec3 dstPos;

            if (msg.receiverId == -1) {
                dstPos = srcPos + glm::vec3(0, 0.4f, 0);
            }
            else {
                dstPos = sim->getDrones()[msg.receiverId].getPosition();
            }

            activePackets.push_back(ActivePacket{
                srcPos,
                dstPos,
                sendT,
                recvT,
                kind,
                "pkt_" + std::to_string(packetCounter++)
                });
        });

    std::vector<glm::vec3> positions;
    for (auto& d : sim->getDrones()) {
        positions.push_back(d.getPosition());
    }

    cloud = polyscope::registerPointCloud("drones", positions);

    std::vector<glm::vec3> colors(positions.size(), glm::vec3(0, 1, 0));
    cloud->addColorQuantity("statusColor", colors);

    buildNeighborCurveNetwork();
}
