#pragma once
#include <vector>
#include "visualization/PacketVis.h"

class PolyscopeRenderer {
public:
    void initialize(Simulation* sim);
    void drawFrame();

private:
    Simulation* sim;
    std::vector<PacketVis> activePackets;

    bool showHeartbeats = true;
    bool showBroadcasts = true;
    bool showNeighborGraph = false;

    void drawPackets(double currentTime);
};
