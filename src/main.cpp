// =============================================================================
// Drone Swarm Membership Simulator
//
// A visualization and simulation tool for distributed group membership
// protocols in drone swarms. Features both a "drone show" mode for visual
// appeal and a "membership mode" for protocol analysis.
//
// Press buttons, break things, watch the protocol recover. Have fun!
// =============================================================================

#include "core/Simulation.h"
#include "core/RNG.h"
#include "visualization/PolyscopeRenderer.h"

// Global RNG for reproducibility across all components
std::mt19937_64 GLOBAL_RNG;

void reseedRNG(uint64_t seed) {
    GLOBAL_RNG.seed(seed);
}

int main() {
    // Default configuration - tuned for a nice visual demo
    SimulationConfig config;
    config.deltaSmall = 0.5;
    config.heartbeatInterval = 2;
    config.alphaLatency = 0;

    // Create and run the simulation
    Simulation sim(config);
    PolyscopeRenderer renderer(&sim);
    renderer.initialize();
    renderer.renderLoop();

    return 0;
}
