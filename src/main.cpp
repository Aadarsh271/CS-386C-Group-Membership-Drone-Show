#include "core/Simulation.h"
#include "core/RNG.h"
#include "visualization/PolyscopeRenderer.h"

std::mt19937_64 GLOBAL_RNG;

void reseedRNG(uint64_t seed) {
    GLOBAL_RNG.seed(seed);
}

int main() {

    SimulationConfig config;
    config.deltaSmall = 1;
    config.alphaLatency = 0;
    config.deltaLarge = config.baseLatency + 2 * config.distanceLatencyFactor * config.maximalDistance;
    /*config.baseLossProb = 0.0;
    config.betaLoss = 0.0;

    config.enableBursts = false;
    config.burstStartProb = 0.0;
    config.burstDropProb = 0.0;
    config.burstDuration = 0.0;
    */
    Simulation sim(config);

    PolyscopeRenderer renderer(&sim);
    renderer.initialize();
    renderer.renderLoop();

    return 0;
}
