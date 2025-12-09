#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"

#include "control/Controller.h"
#include "visualization/PolyscopeRenderer.h"
#include "core/RNG.h"
#include "util/Logger.h"

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

    ShowMode              gShowMode = ShowMode::Auto;  // Start in auto mode for impressive first launch
    std::vector<Cue>      gTimeline;
    int                   gNextCue = 0;
    bool                  gTimelineInitialized = false;
    double                gDefaultTransitionSec = 1.5;   // Faster transitions
    double                gAutoShowLengthSec = 12.0;     // Shorter cycle for more action

    // Build an exciting auto timeline with varied transitions
    void buildAutoTimeline(Controller* ctrl, double startTime) {
        gTimeline.clear();
        gNextCue = 0;
        gTimelineInitialized = true;

        if (!ctrl) return;
        int patternCount = ctrl->getPatternCount();
        if (patternCount <= 0) return;

        // Dynamic show sequence: circle -> star -> square -> triangle -> star -> circle
        // Mix of fast and slower transitions for visual interest
        std::vector<int> order;
        std::vector<double> durations;

        if (patternCount >= 4) {
            // Full show: circle(0) -> star(1) -> square(2) -> triangle(3) -> star(1) -> circle(0)
            order = { 0, 1, 2, 3, 1, 0 };
            durations = { 0.0, 1.2, 1.5, 1.0, 1.8, 1.2 };  // Varied timing
        } else if (patternCount >= 2) {
            order = { 0, 1, 0 };
            durations = { 0.0, 1.5, 1.5 };
        } else {
            order = { 0 };
            durations = { 0.0 };
        }

        int cueCount = (int)order.size();
        if (cueCount == 0) return;

        double holdTime = 1.5;  // Time to hold each shape before transitioning
        double t = 0.0;

        for (int i = 0; i < cueCount; ++i) {
            Cue c;
            c.time = t + startTime;
            c.patternIndex = order[i];
            c.duration = durations[i];
            gTimeline.push_back(c);

            // Time for this cue: transition duration + hold time
            t += durations[i] + holdTime;
        }

        gAutoShowLengthSec = t;  // Update total length
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
    polyscope::options::buildGui = false;

    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;

    // Default to drone show mode: black background for dramatic effect
    polyscope::view::bgColor = { 0.0f, 0.0f, 0.0f, 1.0f };

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
//   UI Panel - Organized for clarity and ease of use
// ============================================================================

void PolyscopeRenderer::drawUI() {

    auto& config = sim->getConfig();
    double t = sim->getTime();

    // ========================================================================
    // MAIN CONTROL PANEL - One window to rule them all
    // ========================================================================
    ImGui::Begin("Drone Swarm Simulator");

    // Quick mode switcher at the top
    ImGui::SeparatorText("Quick Mode Switch");

    if (ImGui::Button("Drone Show Mode", ImVec2(150, 0))) {
        polyscope::view::bgColor = { 0.0f, 0.0f, 0.0f, 1.0f };
        showNeighborLinks = false;
        showHeartbeats = false;
        showBroadcasts = false;
        colorMode = PolyscopeRenderer::ColorMode::FORMATION;
    }
    ImGui::SameLine();
    if (ImGui::Button("Membership Mode", ImVec2(150, 0))) {
        polyscope::view::bgColor = { 0.12f, 0.12f, 0.12f, 1.0f };
        showNeighborLinks = true;
        showHeartbeats = true;
        showBroadcasts = true;
        colorMode = PolyscopeRenderer::ColorMode::STATUS;
    }

    // ------------------------------------------------------------------------
    // STATUS DISPLAY - Always visible
    // ------------------------------------------------------------------------
    ImGui::SeparatorText("Status");

    ImGui::Text("Time: %.2f s", t);
    ImGui::SameLine(150);
    int upCount = 0;
    for (auto& d : sim->getDrones()) {
        if (d.getStatus() == DroneStatus::UP) upCount++;
    }
    ImGui::Text("Drones: %d/%d UP", upCount, (int)sim->getDrones().size());

    // Membership consistency with prominent display
    auto [good, gid, memCount] = sim->checkConsistency();
    if (good) {
        ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "CONSISTENT");
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.2f, 0.2f, 1.0f), "INCONSISTENT");
    }
    ImGui::SameLine();
    ImGui::Text("Group: %d  Members: %d", gid, memCount);

    // Heartbeat miss indicator - prominent warning when something happens
    double latestMissTime = -1.0;
    int latestMissDroneId = -1;
    int latestMissNeighborId = -1;
    for (auto& d : sim->getDrones()) {
        if (d.getStatus() == DroneStatus::UP) {
            double missTime = d.getMembership().getLastMissedHeartbeatTime();
            if (missTime > latestMissTime) {
                latestMissTime = missTime;
                latestMissDroneId = d.getId();
                latestMissNeighborId = d.getMembership().getLastMissedNeighborId();
            }
        }
    }

    if (latestMissTime > 0.0 && (t - latestMissTime) < 5.0) {
        float fade = 1.0f - (float)(t - latestMissTime) / 5.0f;
        ImGui::TextColored(ImVec4(1.0f, 0.3f * fade, 0.3f * fade, 1.0f),
            "HEARTBEAT MISS: Drone %d missed from %d (t=%.2f)",
            latestMissDroneId, latestMissNeighborId, latestMissTime);
    }

    // ------------------------------------------------------------------------
    // VISUALIZATION CONTROLS
    // ------------------------------------------------------------------------
    if (ImGui::CollapsingHeader("Visualization", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Neighbor Links", &showNeighborLinks);
        ImGui::SameLine();
        ImGui::Checkbox("Heartbeats", &showHeartbeats);
        ImGui::SameLine();
        ImGui::Checkbox("Broadcasts", &showBroadcasts);

        const char* colorModeNames[] = { "Status (UP/DOWN)", "Formation (Rainbow)" };
        ImGui::Combo("Color Mode", (int*)&colorMode, colorModeNames, IM_ARRAYSIZE(colorModeNames));
    }

    // ------------------------------------------------------------------------
    // FAILURE INJECTION - For testing the protocol
    // ------------------------------------------------------------------------
    if (ImGui::CollapsingHeader("Failure Injection", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Kill Random Drone")) {
            std::vector<int> alive;
            for (auto& d : sim->getDrones()) {
                if (d.getStatus() == DroneStatus::UP) {
                    alive.push_back(d.getId());
                }
            }
            if (!alive.empty()) {
                std::uniform_int_distribution<int> dist(0, (int)alive.size() - 1);
                sim->killDrone(alive[dist(GLOBAL_RNG)]);
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Force Neighbor Reconfig")) {
            for (auto& d : sim->getDrones()) {
                if (d.getStatus() == DroneStatus::UP) {
                    d.getMembership().updateClosestNeighbors(d.getPosition());
                }
            }
            polyscope::removeCurveNetwork("neighbors");
            buildNeighborCurveNetwork();
        }

        // Scenario buttons
        ImGui::Separator();
        ImGui::Text("Scenarios:");

        if (ImGui::Button("Cascade Failure")) {
            // Kill a drone and its neighbors to demonstrate worst-case
            auto& drones = sim->getDrones();
            std::vector<int> alive;
            for (auto& d : drones) {
                if (d.getStatus() == DroneStatus::UP) alive.push_back(d.getId());
            }
            if (alive.size() >= 3) {
                int victim = alive[0];
                sim->killDrone(victim);
                int left = drones[victim].getMembership().getLeftNeighbor();
                int right = drones[victim].getMembership().getRightNeighbor();
                if (left != -1 && drones[left].getStatus() == DroneStatus::UP)
                    sim->killDrone(left);
                if (right != -1 && right != left && drones[right].getStatus() == DroneStatus::UP)
                    sim->killDrone(right);
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Kill Half")) {
            auto& drones = sim->getDrones();
            int count = 0;
            for (auto& d : drones) {
                if (d.getStatus() == DroneStatus::UP && count % 2 == 0) {
                    sim->killDrone(d.getId());
                }
                count++;
            }
        }
    }

    // ------------------------------------------------------------------------
    // PROTOCOL PARAMETERS
    // ------------------------------------------------------------------------
    if (ImGui::CollapsingHeader("Protocol Parameters")) {
        ImGui::Text("Timing");
        float hbInterval = (float)config.heartbeatInterval;
        if (ImGui::SliderFloat("Heartbeat Interval", &hbInterval, 0.1f, 5.0f, "%.2f s"))
            config.heartbeatInterval = hbInterval;

        float deltaSmall = (float)config.deltaSmall;
        if (ImGui::SliderFloat("Timeout Delta (small)", &deltaSmall, 0.05f, 2.0f, "%.2f s"))
            config.deltaSmall = deltaSmall;

        float deltaLarge = (float)config.deltaLarge;
        if (ImGui::SliderFloat("Timeout Delta (large)", &deltaLarge, 0.1f, 3.0f, "%.2f s"))
            config.deltaLarge = deltaLarge;

        float reconfigInterval = (float)config.reconfigMinInterval;
        if (ImGui::SliderFloat("Reconfig Min Interval", &reconfigInterval, 0.5f, 5.0f, "%.2f s"))
            config.reconfigMinInterval = reconfigInterval;
    }

    // ------------------------------------------------------------------------
    // FAULT MODEL
    // ------------------------------------------------------------------------
    if (ImGui::CollapsingHeader("Fault Model")) {
        ImGui::Checkbox("Enable Random Crashes", &config.enableCrashes);
        if (config.enableCrashes) {
            float crashRate = (float)config.crashRate;
            if (ImGui::SliderFloat("Crash Rate", &crashRate, 0.0f, 0.1f, "%.4f"))
                config.crashRate = crashRate;

            float permCrash = (float)config.pPermanentCrash;
            if (ImGui::SliderFloat("Permanent Crash Prob", &permCrash, 0.0f, 1.0f, "%.2f"))
                config.pPermanentCrash = permCrash;
        }

        ImGui::Separator();
        ImGui::Text("Message Omissions");
        float omissionProb = (float)config.omissionProb;
        if (ImGui::SliderFloat("Omission Probability", &omissionProb, 0.0f, 0.5f, "%.2f"))
            config.omissionProb = omissionProb;

        float sendJitter = (float)config.sendJitter;
        if (ImGui::SliderFloat("Send Jitter", &sendJitter, 0.0f, 0.5f, "%.3f s"))
            config.sendJitter = sendJitter;
    }

    // ------------------------------------------------------------------------
    // NETWORK MODEL
    // ------------------------------------------------------------------------
    if (ImGui::CollapsingHeader("Network Model")) {
        float baseLatency = (float)config.baseLatency;
        if (ImGui::SliderFloat("Base Latency", &baseLatency, 0.0f, 0.5f, "%.3f s"))
            config.baseLatency = baseLatency;

        float alphaLatency = (float)config.alphaLatency;
        if (ImGui::SliderFloat("Alpha (Load Growth)", &alphaLatency, 0.0f, 0.5f, "%.3f"))
            config.alphaLatency = alphaLatency;

        float distLatency = (float)config.distanceLatencyFactor;
        if (ImGui::SliderFloat("Distance Factor", &distLatency, 0.0f, 0.1f, "%.4f"))
            config.distanceLatencyFactor = distLatency;

        ImGui::Separator();
        ImGui::Text("Packet Loss");
        float baseLoss = (float)config.baseLossProb;
        if (ImGui::SliderFloat("Base Loss Prob", &baseLoss, 0.0f, 0.3f, "%.3f"))
            config.baseLossProb = baseLoss;

        ImGui::Checkbox("Enable Burst Loss", &config.enableBursts);
        if (config.enableBursts) {
            float burstStart = (float)config.burstStartProb;
            if (ImGui::SliderFloat("Burst Start Prob", &burstStart, 0.0f, 0.1f, "%.4f"))
                config.burstStartProb = burstStart;

            float burstDrop = (float)config.burstDropProb;
            if (ImGui::SliderFloat("Burst Drop Prob", &burstDrop, 0.0f, 1.0f, "%.2f"))
                config.burstDropProb = burstDrop;

            float burstDur = (float)config.burstDuration;
            if (ImGui::SliderFloat("Burst Duration", &burstDur, 0.0f, 2.0f, "%.2f s"))
                config.burstDuration = burstDur;
        }

        float dupProb = (float)config.duplicationProb;
        if (ImGui::SliderFloat("Duplication Prob", &dupProb, 0.0f, 0.2f, "%.3f"))
            config.duplicationProb = dupProb;
    }

    // ------------------------------------------------------------------------
    // CONSOLE LOGGING
    // ------------------------------------------------------------------------
    if (ImGui::CollapsingHeader("Console Logging")) {
        ImGui::Checkbox("Enable Logging", &Log::enabled);
        if (Log::enabled) {
            ImGui::Checkbox("Heartbeats", &Log::showHeartbeats);
            ImGui::SameLine();
            ImGui::Checkbox("Reconfig", &Log::showReconfig);
            ImGui::Checkbox("Failures", &Log::showFailures);
            ImGui::SameLine();
            ImGui::Checkbox("Messages", &Log::showMessages);
        }
    }

    // ------------------------------------------------------------------------
    // SIMULATION CONTROL
    // ------------------------------------------------------------------------
    if (ImGui::CollapsingHeader("Simulation")) {
        float timeStep = (float)config.timeStep;
        if (ImGui::SliderFloat("Time Step", &timeStep, 0.005f, 0.1f, "%.3f s"))
            config.timeStep = timeStep;

        ImGui::InputInt("Seed", (int*)&config.seed);

        if (ImGui::Button("Restart Simulation")) {
            reseedRNG(config.seed);
            sim = new Simulation(config);
            rebuildAllVisuals();
            gTimelineInitialized = false;
            gTimeline.clear();
            gNextCue = 0;
        }
        ImGui::SameLine();
        if (ImGui::Button("Randomize Positions")) {
            std::vector<int> perm(sim->getDrones().size());
            std::iota(perm.begin(), perm.end(), 0);
            std::shuffle(perm.begin(), perm.end(), GLOBAL_RNG);

            float radius = config.numDrones * config.droneSpacing / (2.0f * 3.14159f);
            float angleStep = 2.0f * 3.14159f / config.numDrones;

            for (int i = 0; i < config.numDrones; ++i) {
                float angle = i * angleStep;
                glm::vec3 pos{ radius * std::cos(angle), 0.0f, radius * std::sin(angle) };
                sim->getDrones()[perm[i]].setPosition(pos);
            }
            updateDronePositions();
            updateNeighborLinks();
        }
    }

    ImGui::End();

    // ========================================================================
    // DRONE SHOW PANEL - Formation control (only when relevant)
    // ========================================================================
    auto* ctrl = sim->getController();
    int patternCount = ctrl ? ctrl->getPatternCount() : 0;

    if (ctrl && patternCount > 0) {
        ImGui::Begin("Formation Control");

        int modeInt = (gShowMode == ShowMode::Manual ? 0 : 1);
        ImGui::RadioButton("Manual", &modeInt, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Auto Timeline", &modeInt, 1);
        gShowMode = (modeInt == 0 ? ShowMode::Manual : ShowMode::Auto);

        if (gShowMode == ShowMode::Manual) {
            static double manualDuration = 2.0;
            float dur = (float)manualDuration;
            if (ImGui::SliderFloat("Morph Duration", &dur, 0.5f, 5.0f, "%.1f s"))
                manualDuration = dur;

            ImGui::Text("Active: Pattern %d", ctrl->getActivePattern());

            for (int i = 0; i < patternCount; ++i) {
                ImGui::PushID(i);
                if (ImGui::Button(("Pattern " + std::to_string(i)).c_str(), ImVec2(80, 0))) {
                    ctrl->startTransitionTo(i, sim->getTime(), manualDuration);
                }
                if (i < patternCount - 1) ImGui::SameLine();
                ImGui::PopID();
            }
        } else {
            if (!gTimelineInitialized) {
                ImGui::TextWrapped("Timeline builds on next frame...");
            } else {
                ImGui::Text("Cue %d/%d", gNextCue, (int)gTimeline.size());
                if (gNextCue < (int)gTimeline.size()) {
                    const Cue& c = gTimeline[gNextCue];
                    ImGui::SameLine();
                    ImGui::Text("-> Pattern %d at t=%.1f", c.patternIndex, c.time);
                }
            }

            if (ImGui::Button("Restart Show")) {
                gTimelineInitialized = false;
                gTimeline.clear();
                gNextCue = 0;
                ctrl->setActivePattern(0);
            }
        }

        ImGui::End();
    }
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
        // Only draw edges between UP drones in their membership views
        if (drones[i].getStatus() != DroneStatus::UP) continue;

        int right = drones[i].getMembership().getRightNeighbor();
        int left = drones[i].getMembership().getLeftNeighbor();

        // Only draw if neighbor is also UP
        if (right != -1 && right < N && drones[right].getStatus() == DroneStatus::UP) {
            edges.push_back({ (size_t)i, (size_t)right });
        }
        if (left != -1 && left < N && drones[left].getStatus() == DroneStatus::UP) {
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
    neighborNet->setEnabled(true);  // Ensure it's enabled when we rebuild
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
