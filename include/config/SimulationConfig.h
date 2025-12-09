#pragma once
#include <core/RNG.h>
#include <cstdint>

// =============================================================================
// SimulationConfig - All the knobs and dials for the simulator
//
// This structure contains every configurable parameter for the drone swarm
// simulation. Parameters are organized into logical groups:
//   - Global: timestep, random seed
//   - Protocol: heartbeat timing, failure detection thresholds
//   - Faults: crash injection for stress testing
//   - Network: latency, loss, and all manner of network nastiness
// =============================================================================

struct SimulationConfig {

    // -------------------------------------------------------------------------
    // Global Simulation Parameters
    // -------------------------------------------------------------------------
    uint64_t seed = 1;             // RNG seed for reproducibility
    double timeStep = 0.02;        // Seconds per simulation update

    // -------------------------------------------------------------------------
    // Swarm Configuration
    // -------------------------------------------------------------------------
    int numDrones = 20;            // Number of drones in the swarm
    float droneSpacing = 0.1f;     // Spacing factor for initial layout

    // -------------------------------------------------------------------------
    // Membership Protocol Timing
    // These parameters directly affect failure detection speed vs reliability.
    // -------------------------------------------------------------------------
    double heartbeatInterval = 0.2;    // How often drones send heartbeats
    double timeoutDelta = 0.6;         // Delay bound for timeout detection
    double deltaSmall = 0.12;          // Max datagram delay (Δ₁)
    double deltaLarge = 0.50;          // Atomic broadcast stabilization (Δ₂)
    double reconfigMinInterval = 1.0;  // Min time between reconfigurations

    int kMissTolerance = 2;            // Misses before declaring failure
    double joinCooldown = 2.0;         // Cooldown before rejoin attempt

    // -------------------------------------------------------------------------
    // Fault Injection Model
    // For stress testing the protocol under adverse conditions.
    // -------------------------------------------------------------------------
    bool enableCrashes = false;        // Enable random drone crashes
    double crashRate = 0.0;            // Probability of crash per tick
    double pPermanentCrash = 0.05;     // Probability crash is permanent

    double recoveryMin = 3.0;          // Min recovery time (seconds)
    double recoveryMax = 8.0;          // Max recovery time (seconds)

    double omissionProb = 0.0;         // Per-message omission probability
    double sendJitter = 0.0;           // Timestamp jitter (seconds)

    // Clock drift (for future use)
    double driftMin = 1.0;
    double driftMax = 1.0;

    // -------------------------------------------------------------------------
    // Network Model
    // Simulates realistic network behavior including latency and packet loss.
    // -------------------------------------------------------------------------
    double baseLatency = 0.05;         // Base network latency (seconds)
    double alphaLatency = 0.1;         // Latency growth with network load
    double distanceLatencyFactor = 0.0; // Extra latency based on distance
    double maximalDistance = 10;       // Reference distance for calculations

    double baseLossProb = 0.0;         // Base packet loss probability
    double betaLoss = 0.01;            // Loss growth with load

    // Burst loss model (correlated losses)
    bool enableBursts = false;
    double burstStartProb = 0.0;
    double burstDropProb = 0.0;
    double burstDuration = 0.0;

    double duplicationProb = 0.0;      // Packet duplication probability

    // -------------------------------------------------------------------------
    // Visualization Defaults
    // -------------------------------------------------------------------------
    bool showMembershipViz = false;
};
