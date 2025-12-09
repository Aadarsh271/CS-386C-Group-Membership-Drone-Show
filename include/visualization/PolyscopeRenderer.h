#pragma once
#include <vector>
#include <string>

#include <glm/glm.hpp>

#include "visualization/PacketVis.h"
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"

#include "core/Simulation.h"


class PolyscopeRenderer {
public:
    PolyscopeRenderer(Simulation* sim);
    PolyscopeRenderer() = default;

    // Build all polyscope structures
    void initialize();

    // Block until window closes
    void renderLoop();

    // Rebuild visuals after Simulation restart
    void rebuildAllVisuals();

private:
    Simulation* sim = nullptr;

    polyscope::PointCloud* cloud = nullptr;
    polyscope::CurveNetwork* neighborNet = nullptr;

    // UI toggles - default to drone show mode (hide membership visuals)
    bool showNeighborLinks = false;
    bool showHeartbeats = false;
    bool showBroadcasts = false;

    struct ActivePacket {
        glm::vec3 src;
        glm::vec3 dst;
        double createdAt;
        double deliverAt;
        PacketKind kind;
        std::string handle; // unique polyscope name
    };

    std::vector<ActivePacket> activePackets;

    // === Internal updates ===
    void drawUI();
    void updateDronePositions();
    void updateDroneColors();
    void buildNeighborCurveNetwork();
    void updateNeighborLinks();
    void drawPackets();

    int packetCounter = 0;

    enum class ColorMode {
        STATUS = 0,
        FORMATION = 1
    };

    // Default to FORMATION (Drone Show) mode for a more impressive first launch
    ColorMode colorMode = ColorMode::FORMATION; 
};
