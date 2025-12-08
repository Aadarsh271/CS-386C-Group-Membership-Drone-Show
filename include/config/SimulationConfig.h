#pragma once
#include <core/RNG.h>
#include <cstdint>

struct SimulationConfig {

    // --- Global simulation ---
    uint64_t seed = 1;
    double timeStep = 0.02;  // seconds per simulation update

    // --- Drone counts / initial layout ---
    int numDrones = 20;
    float droneSpacing = 0.1f;

    // --- Membership protocol parameters ---
    double heartbeatInterval = 0.2; // seconds
    double timeoutDelta = 0.6;      // delay bound Δ
    // --- Membership timing constraints ---
    double deltaSmall = 0.12;   // maximum datagram time delay (Δ1)
    double deltaLarge = 0.50;   // atomic broadcast stabilizing window (Δ2)

    int kMissTolerance = 2;         // suspect after 1, dead after k
    double joinCooldown = 2.0;      // seconds before rejoin

    // --- Fault model parameters ---
    bool enableCrashes = false;
    double crashRate = 0.0;
    double pPermanentCrash = 0.05;

    double recoveryMin = 3.0;
    double recoveryMax = 8.0;

    double omissionProb = 0.0;     // per message omission
    double sendJitter = 0.0;       // additive jitter on send timestamps

    // Clock drift
    double driftMin = 1.0;         // scale factor (no drift)
    double driftMax = 1.0;

    // --- Network parameters ---
    double baseLatency = 0.05;
    double alphaLatency = 0.1;     // latency growth with load
    double baseLossProb = 0.0;
    double betaLoss = 0.01;        // loss growth with load

    // Distance-based latency
    double distanceLatencyFactor = 0.0;
	double maximalDistance = 10; // for distance-based latency calculations

    // Burst loss
    bool enableBursts = false;
    double burstStartProb = 0.0;
    double burstDropProb = 0.0;
    double burstDuration = 0.0;

    // Duplication
    double duplicationProb = 0.0;

    // Visualization (optional defaults)
    bool showMembershipViz = false;

    double reconfigMinInterval = 1.0;  // r seconds between consecutive atomic broadcasts

};
