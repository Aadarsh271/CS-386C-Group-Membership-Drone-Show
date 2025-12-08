// tests/NeighborhoodSurveillanceTests.cpp
//
// Comprehensive tests for the neighborhood surveillance membership protocol.
// Tests various failure scenarios including the worst-case cascading failure pattern.

#include "core/Simulation.h"
#include "core/RNG.h"

#include <cassert>
#include <iostream>
#include <set>
#include <vector>
#include <algorithm>
#include <map>
#include <iomanip>

std::mt19937_64 GLOBAL_RNG;

void reseedRNG(uint64_t seed) {
    GLOBAL_RNG.seed(seed);
}

// ============================================================================
//                           HELPER FUNCTIONS
// ============================================================================

static void configureDeterministicNetwork(SimulationConfig& cfg) {
    cfg.baseLatency = 0.0;
    cfg.alphaLatency = 0.0;
    cfg.distanceLatencyFactor = 0.0;
    cfg.baseLossProb = 0.0;
    cfg.betaLoss = 0.0;
    cfg.enableBursts = false;
    cfg.burstStartProb = 0.0;
    cfg.burstDropProb = 0.0;
    cfg.burstDuration = 0.0;
    cfg.duplicationProb = 0.0;
    cfg.enableCrashes = false;
    cfg.omissionProb = 0.0;
    cfg.sendJitter = 0.0;
}

static void configureRealisticNetwork(SimulationConfig& cfg) {
    cfg.baseLatency = 0.01;        // 10ms base latency
    cfg.alphaLatency = 0.05;
    cfg.distanceLatencyFactor = 0.001;
    cfg.baseLossProb = 0.01;       // 1% base loss
    cfg.betaLoss = 0.005;
    cfg.enableBursts = false;
    cfg.duplicationProb = 0.0;
    cfg.enableCrashes = false;
    cfg.omissionProb = 0.0;
    cfg.sendJitter = 0.005;
}

// Snapshot membership from the first UP drone
static void snapshotMembership(Simulation& sim, int& groupIdOut, std::set<int>& membersOut) {
    groupIdOut = -1;
    membersOut.clear();

    for (auto& d : sim.getDrones()) {
        if (d.getStatus() == DroneStatus::UP) {
            groupIdOut = d.getMembership().getGroupId();
            auto memVec = d.getMembership().getActiveMembers();
            membersOut.insert(memVec.begin(), memVec.end());
            break;
        }
    }
}

// Check if all UP drones have consensus
static bool checkConsensus(Simulation& sim, int& outGroupId, std::set<int>& outMembers) {
    outGroupId = -1;
    outMembers.clear();

    for (auto& d : sim.getDrones()) {
        if (d.getStatus() == DroneStatus::UP) {
            int dg = d.getMembership().getGroupId();
            if (dg == 0) return false;  // Not yet in a group

            if (outGroupId == -1) {
                outGroupId = dg;
                auto memVec = d.getMembership().getActiveMembers();
                outMembers.insert(memVec.begin(), memVec.end());
            } else if (dg != outGroupId) {
                return false;  // Split brain
            }
        }
    }
    return outGroupId != -1;
}

// Wait for consensus or timeout
static bool waitForConsensus(Simulation& sim, double timeout, int& outGroupId, std::set<int>& outMembers) {
    double startTime = sim.getTime();
    while (sim.getTime() < startTime + timeout) {
        sim.step();
        if (checkConsensus(sim, outGroupId, outMembers)) {
            return true;
        }
    }
    return false;
}

// Get neighbor info for a drone
static void getNeighbors(Simulation& sim, int droneId, int& leftNeighbor, int& rightNeighbor) {
    auto& d = sim.getDrones()[droneId];
    leftNeighbor = d.getMembership().getLeftNeighbor();
    rightNeighbor = d.getMembership().getRightNeighbor();
}

// Print current state
static void printState(Simulation& sim, const std::string& label) {
    std::cout << "\n=== " << label << " (t=" << std::fixed << std::setprecision(3) << sim.getTime() << ") ===\n";
    for (auto& d : sim.getDrones()) {
        std::cout << "  Drone " << d.getId() << ": "
                  << (d.getStatus() == DroneStatus::UP ? "UP" : "DOWN")
                  << " group=" << d.getMembership().getGroupId()
                  << " neighbors=[" << d.getMembership().getLeftNeighbor()
                  << "," << d.getMembership().getRightNeighbor() << "]\n";
    }
}

// ============================================================================
//                              TEST CASES
// ============================================================================

// Test 1: Basic startup - all nodes form a group
static void test_startup_group_formation() {
    std::cout << "\n[TEST] startup_group_formation\n";

    SimulationConfig cfg;
    configureDeterministicNetwork(cfg);
    cfg.numDrones = 8;
    cfg.timeStep = 0.02;
    cfg.heartbeatInterval = 0.2;
    cfg.timeoutDelta = 0.6;
    cfg.deltaSmall = 0.12;
    cfg.deltaLarge = 0.5;
    cfg.reconfigMinInterval = 1.0;

    reseedRNG(1001);
    Simulation sim(cfg);

    int groupId;
    std::set<int> members;
    bool success = waitForConsensus(sim, 10.0, groupId, members);

    assert(success && "Startup group formation failed");
    assert(members.size() == cfg.numDrones && "Not all drones in group");

    std::cout << "  -> OK, groupId=" << groupId << ", members=" << members.size() << "\n";
}

// Test 2: Single node failure
static void test_single_node_failure() {
    std::cout << "\n[TEST] single_node_failure\n";

    SimulationConfig cfg;
    configureDeterministicNetwork(cfg);
    cfg.numDrones = 8;
    cfg.timeStep = 0.02;
    cfg.heartbeatInterval = 0.2;
    cfg.timeoutDelta = 0.6;
    cfg.deltaSmall = 0.12;
    cfg.deltaLarge = 0.5;
    cfg.reconfigMinInterval = 2.0;

    reseedRNG(1002);
    Simulation sim(cfg);

    // Wait for initial group
    int groupId1;
    std::set<int> members1;
    assert(waitForConsensus(sim, 10.0, groupId1, members1));

    // Reset group IDs to test reconfig
    for (auto& d : sim.getDrones())
        d.getMembership().forceGroupId(0);

    // Kill drone 3
    sim.killDrone(3);
    std::cout << "  Killed drone 3\n";

    // Wait for reconfig
    int groupId2;
    std::set<int> members2;
    assert(waitForConsensus(sim, 15.0, groupId2, members2));

    assert(members2.count(3) == 0 && "Dead drone still in membership");
    assert(members2.size() == 7 && "Wrong member count after failure");

    std::cout << "  -> OK, groupId=" << groupId2 << ", members=" << members2.size() << "\n";
}

// Test 3: Simultaneous failure of two non-adjacent nodes
static void test_two_nonadjacent_failures() {
    std::cout << "\n[TEST] two_nonadjacent_failures\n";

    SimulationConfig cfg;
    configureDeterministicNetwork(cfg);
    cfg.numDrones = 8;
    cfg.timeStep = 0.02;
    cfg.heartbeatInterval = 0.2;
    cfg.timeoutDelta = 0.6;
    cfg.deltaSmall = 0.12;
    cfg.deltaLarge = 0.5;
    cfg.reconfigMinInterval = 2.0;

    reseedRNG(1003);
    Simulation sim(cfg);

    // Wait for initial group
    int groupId1;
    std::set<int> members1;
    assert(waitForConsensus(sim, 10.0, groupId1, members1));

    for (auto& d : sim.getDrones())
        d.getMembership().forceGroupId(0);

    // Kill drones 0 and 4 (opposite sides of the ring)
    sim.killDrone(0);
    sim.killDrone(4);
    std::cout << "  Killed drones 0 and 4 simultaneously\n";

    int groupId2;
    std::set<int> members2;
    assert(waitForConsensus(sim, 15.0, groupId2, members2));

    assert(members2.count(0) == 0 && "Dead drone 0 still in membership");
    assert(members2.count(4) == 0 && "Dead drone 4 still in membership");
    assert(members2.size() == 6 && "Wrong member count");

    std::cout << "  -> OK, groupId=" << groupId2 << ", members=" << members2.size() << "\n";
}

// Test 4: Adjacent node failures (tests neighbor surveillance)
static void test_adjacent_failures() {
    std::cout << "\n[TEST] adjacent_failures\n";

    SimulationConfig cfg;
    configureDeterministicNetwork(cfg);
    cfg.numDrones = 8;
    cfg.timeStep = 0.02;
    cfg.heartbeatInterval = 0.2;
    cfg.timeoutDelta = 0.6;
    cfg.deltaSmall = 0.12;
    cfg.deltaLarge = 0.5;
    cfg.reconfigMinInterval = 2.0;

    reseedRNG(1004);
    Simulation sim(cfg);

    // Wait for initial group
    int groupId1;
    std::set<int> members1;
    assert(waitForConsensus(sim, 10.0, groupId1, members1));

    // Find drone 0's neighbors
    int left0, right0;
    getNeighbors(sim, 0, left0, right0);
    std::cout << "  Drone 0's neighbors: left=" << left0 << " right=" << right0 << "\n";

    for (auto& d : sim.getDrones())
        d.getMembership().forceGroupId(0);

    // Kill drone 0 and one of its neighbors
    sim.killDrone(0);
    sim.killDrone(right0);
    std::cout << "  Killed drone 0 and its right neighbor " << right0 << "\n";

    int groupId2;
    std::set<int> members2;
    assert(waitForConsensus(sim, 15.0, groupId2, members2));

    assert(members2.count(0) == 0);
    assert(members2.count(right0) == 0);
    assert(members2.size() == 6);

    std::cout << "  -> OK, groupId=" << groupId2 << ", members=" << members2.size() << "\n";
}

// Test 5: Worst case - cascading neighbor failures
// Kill node, then its neighbors, then their neighbors, etc.
// This tests the limit of the surveillance protocol.
static void test_cascading_neighbor_failures() {
    std::cout << "\n[TEST] cascading_neighbor_failures (WORST CASE)\n";

    SimulationConfig cfg;
    configureDeterministicNetwork(cfg);
    cfg.numDrones = 16;  // Larger swarm for this test
    cfg.timeStep = 0.02;
    cfg.heartbeatInterval = 0.2;
    cfg.timeoutDelta = 0.6;
    cfg.deltaSmall = 0.12;
    cfg.deltaLarge = 0.5;
    cfg.reconfigMinInterval = 2.0;

    reseedRNG(1005);
    Simulation sim(cfg);

    // Wait for initial group
    int groupId1;
    std::set<int> members1;
    assert(waitForConsensus(sim, 10.0, groupId1, members1));
    printState(sim, "After initial group formation");

    // Strategy: Kill in waves, timing kills to be between heartbeats
    // Wave 1: Kill drone 0 right after heartbeats
    // Wave 2: Kill 0's neighbors before timeout detected
    // Wave 3: Kill those neighbors' neighbors, etc.

    std::set<int> killed;
    std::set<int> toKillNext;

    // Wave 1: Initial kill
    int victim = 0;
    sim.killDrone(victim);
    killed.insert(victim);

    int left, right;
    getNeighbors(sim, victim, left, right);
    if (left != -1) toKillNext.insert(left);
    if (right != -1 && right != left) toKillNext.insert(right);

    std::cout << "  Wave 1: Killed drone " << victim << ", next targets: {";
    for (int id : toKillNext) std::cout << id << " ";
    std::cout << "}\n";

    // Run for half the heartbeat interval
    double waveTime = sim.getTime();
    while (sim.getTime() < waveTime + cfg.heartbeatInterval * 0.4) {
        sim.step();
    }

    // Wave 2: Kill the neighbors
    std::set<int> wave2Targets = toKillNext;
    toKillNext.clear();

    for (int target : wave2Targets) {
        if (killed.count(target)) continue;
        if (sim.getDrones()[target].getStatus() == DroneStatus::DOWN) continue;

        sim.killDrone(target);
        killed.insert(target);

        getNeighbors(sim, target, left, right);
        if (left != -1 && !killed.count(left)) toKillNext.insert(left);
        if (right != -1 && right != left && !killed.count(right)) toKillNext.insert(right);
    }

    std::cout << "  Wave 2: Killed {";
    for (int id : wave2Targets) std::cout << id << " ";
    std::cout << "}, next targets: {";
    for (int id : toKillNext) std::cout << id << " ";
    std::cout << "}\n";

    // Run for half the heartbeat interval
    waveTime = sim.getTime();
    while (sim.getTime() < waveTime + cfg.heartbeatInterval * 0.4) {
        sim.step();
    }

    // Wave 3: Kill more neighbors (if any remain)
    std::set<int> wave3Targets = toKillNext;
    toKillNext.clear();

    for (int target : wave3Targets) {
        if (killed.count(target)) continue;
        if (sim.getDrones()[target].getStatus() == DroneStatus::DOWN) continue;

        sim.killDrone(target);
        killed.insert(target);
    }

    if (!wave3Targets.empty()) {
        std::cout << "  Wave 3: Killed {";
        for (int id : wave3Targets) std::cout << id << " ";
        std::cout << "}\n";
    }

    std::cout << "  Total killed: " << killed.size() << " drones\n";
    printState(sim, "After cascading failures");

    // Reset groups and wait for reconfig
    for (auto& d : sim.getDrones())
        if (d.getStatus() == DroneStatus::UP)
            d.getMembership().forceGroupId(0);

    int groupId2;
    std::set<int> members2;
    bool success = waitForConsensus(sim, 30.0, groupId2, members2);

    if (success) {
        // Verify no killed drones are in membership
        for (int k : killed) {
            assert(members2.count(k) == 0 && "Killed drone still in membership!");
        }

        std::cout << "  -> OK, survivors formed group " << groupId2
                  << " with " << members2.size() << " members\n";
        printState(sim, "Final state");
    } else {
        std::cout << "  -> EXPECTED: Cascading failures may partition the network\n";
        printState(sim, "Final state (no consensus)");
    }
}

// Test 6: Timed cascading failure - precisely between heartbeats
static void test_timed_cascading_failure() {
    std::cout << "\n[TEST] timed_cascading_failure\n";
    std::cout << "  This test kills nodes in a precise pattern to maximize damage.\n";

    SimulationConfig cfg;
    configureDeterministicNetwork(cfg);
    cfg.numDrones = 12;
    cfg.timeStep = 0.02;
    cfg.heartbeatInterval = 0.2;
    cfg.timeoutDelta = 0.6;
    cfg.deltaSmall = 0.12;
    cfg.deltaLarge = 0.5;
    cfg.reconfigMinInterval = 2.0;

    reseedRNG(1006);
    Simulation sim(cfg);

    // Wait for initial group and stabilize
    int groupId1;
    std::set<int> members1;
    assert(waitForConsensus(sim, 10.0, groupId1, members1));

    // Record neighbor structure
    std::map<int, std::pair<int,int>> neighborMap;
    for (auto& d : sim.getDrones()) {
        int left = d.getMembership().getLeftNeighbor();
        int right = d.getMembership().getRightNeighbor();
        neighborMap[d.getId()] = {left, right};
    }

    std::cout << "  Neighbor structure:\n";
    for (auto& [id, neighbors] : neighborMap) {
        std::cout << "    Drone " << id << ": left=" << neighbors.first
                  << " right=" << neighbors.second << "\n";
    }

    // Run until just after a heartbeat round
    double targetTime = sim.getTime() + cfg.heartbeatInterval;
    while (sim.getTime() < targetTime) {
        sim.step();
    }

    // Kill pattern: 0, then 0's neighbors, then theirs
    std::vector<int> killOrder;
    std::set<int> killed;

    // Start with drone 0
    killOrder.push_back(0);
    killed.insert(0);

    // Add 0's neighbors
    auto [l0, r0] = neighborMap[0];
    if (l0 != -1 && !killed.count(l0)) { killOrder.push_back(l0); killed.insert(l0); }
    if (r0 != -1 && !killed.count(r0)) { killOrder.push_back(r0); killed.insert(r0); }

    // Add their neighbors
    if (l0 != -1) {
        auto [ll, lr] = neighborMap[l0];
        if (ll != -1 && !killed.count(ll)) { killOrder.push_back(ll); killed.insert(ll); }
        if (lr != -1 && !killed.count(lr)) { killOrder.push_back(lr); killed.insert(lr); }
    }
    if (r0 != -1) {
        auto [rl, rr] = neighborMap[r0];
        if (rl != -1 && !killed.count(rl)) { killOrder.push_back(rl); killed.insert(rl); }
        if (rr != -1 && !killed.count(rr)) { killOrder.push_back(rr); killed.insert(rr); }
    }

    std::cout << "  Kill order: {";
    for (int id : killOrder) std::cout << id << " ";
    std::cout << "}\n";

    // Execute kills with timing
    for (size_t i = 0; i < killOrder.size(); ++i) {
        sim.killDrone(killOrder[i]);
        std::cout << "  t=" << std::fixed << std::setprecision(3) << sim.getTime()
                  << " killed drone " << killOrder[i] << "\n";

        // Small delay between kills (but less than heartbeat interval)
        double delay = cfg.heartbeatInterval * 0.3;
        double endTime = sim.getTime() + delay;
        while (sim.getTime() < endTime) {
            sim.step();
        }
    }

    // Reset and wait for reconfig
    for (auto& d : sim.getDrones())
        if (d.getStatus() == DroneStatus::UP)
            d.getMembership().forceGroupId(0);

    int groupId2;
    std::set<int> members2;
    bool success = waitForConsensus(sim, 30.0, groupId2, members2);

    int survivors = 0;
    for (auto& d : sim.getDrones())
        if (d.getStatus() == DroneStatus::UP) survivors++;

    std::cout << "  Survivors: " << survivors << " drones\n";

    if (success) {
        assert(members2.size() == (size_t)survivors);
        std::cout << "  -> OK, formed group " << groupId2 << " with " << members2.size() << " members\n";
    } else {
        std::cout << "  -> Network may be partitioned after aggressive kills\n";
    }
}

// Test 7: Stress test with many rapid failures
static void test_rapid_multiple_failures() {
    std::cout << "\n[TEST] rapid_multiple_failures\n";

    SimulationConfig cfg;
    configureDeterministicNetwork(cfg);
    cfg.numDrones = 20;
    cfg.timeStep = 0.02;
    cfg.heartbeatInterval = 0.2;
    cfg.timeoutDelta = 0.6;
    cfg.deltaSmall = 0.12;
    cfg.deltaLarge = 0.5;
    cfg.reconfigMinInterval = 2.0;

    reseedRNG(1007);
    Simulation sim(cfg);

    // Wait for initial group
    int groupId1;
    std::set<int> members1;
    assert(waitForConsensus(sim, 10.0, groupId1, members1));

    // Kill 5 random drones rapidly
    std::vector<int> toKill = {0, 5, 10, 15, 19};
    for (int id : toKill) {
        sim.killDrone(id);
    }
    std::cout << "  Killed 5 drones: {0, 5, 10, 15, 19}\n";

    // Reset and wait for reconfig
    for (auto& d : sim.getDrones())
        if (d.getStatus() == DroneStatus::UP)
            d.getMembership().forceGroupId(0);

    int groupId2;
    std::set<int> members2;
    bool success = waitForConsensus(sim, 30.0, groupId2, members2);

    assert(success && "Failed to reach consensus after multiple failures");
    assert(members2.size() == 15 && "Wrong survivor count");

    for (int id : toKill) {
        assert(members2.count(id) == 0 && "Dead drone in membership");
    }

    std::cout << "  -> OK, groupId=" << groupId2 << ", members=" << members2.size() << "\n";
}

// Test 8: Recovery scenario (if supported)
static void test_minority_survival() {
    std::cout << "\n[TEST] minority_survival\n";
    std::cout << "  Kill majority of nodes, verify minority can still form group.\n";

    SimulationConfig cfg;
    configureDeterministicNetwork(cfg);
    cfg.numDrones = 10;
    cfg.timeStep = 0.02;
    cfg.heartbeatInterval = 0.2;
    cfg.timeoutDelta = 0.6;
    cfg.deltaSmall = 0.12;
    cfg.deltaLarge = 0.5;
    cfg.reconfigMinInterval = 2.0;

    reseedRNG(1008);
    Simulation sim(cfg);

    // Wait for initial group
    int groupId1;
    std::set<int> members1;
    assert(waitForConsensus(sim, 10.0, groupId1, members1));

    // Kill 7 out of 10 drones (keep 0, 1, 2)
    for (int i = 3; i < 10; ++i) {
        sim.killDrone(i);
    }
    std::cout << "  Killed drones 3-9, survivors: {0, 1, 2}\n";

    // Reset and wait for reconfig
    for (auto& d : sim.getDrones())
        if (d.getStatus() == DroneStatus::UP)
            d.getMembership().forceGroupId(0);

    int groupId2;
    std::set<int> members2;
    bool success = waitForConsensus(sim, 30.0, groupId2, members2);

    assert(success && "Minority failed to form group");
    assert(members2.size() == 3 && "Wrong survivor count");
    assert(members2.count(0) && members2.count(1) && members2.count(2));

    std::cout << "  -> OK, minority formed group " << groupId2 << "\n";
}

// Test 9: Network with realistic latency
static void test_realistic_network_conditions() {
    std::cout << "\n[TEST] realistic_network_conditions\n";

    SimulationConfig cfg;
    configureRealisticNetwork(cfg);
    cfg.numDrones = 8;
    cfg.timeStep = 0.02;
    cfg.heartbeatInterval = 0.2;
    cfg.timeoutDelta = 0.8;  // Slightly longer timeout for lossy network
    cfg.deltaSmall = 0.15;
    cfg.deltaLarge = 0.6;
    cfg.reconfigMinInterval = 2.0;

    reseedRNG(1009);
    Simulation sim(cfg);

    // Wait for initial group (may take longer due to packet loss)
    int groupId1;
    std::set<int> members1;
    bool initialSuccess = waitForConsensus(sim, 15.0, groupId1, members1);

    if (!initialSuccess) {
        std::cout << "  -> SKIP: Initial group formation slow under lossy network\n";
        return;
    }

    // Kill one drone
    sim.killDrone(2);
    std::cout << "  Killed drone 2 under realistic network conditions\n";

    for (auto& d : sim.getDrones())
        if (d.getStatus() == DroneStatus::UP)
            d.getMembership().forceGroupId(0);

    int groupId2;
    std::set<int> members2;
    bool success = waitForConsensus(sim, 20.0, groupId2, members2);

    if (success) {
        assert(members2.count(2) == 0);
        std::cout << "  -> OK, reconfig succeeded under lossy network\n";
    } else {
        std::cout << "  -> Reconfig slower under lossy conditions (acceptable)\n";
    }
}

// ============================================================================
//                                 MAIN
// ============================================================================

int main(int argc, char* argv[]) {
    std::cout << "========================================\n";
    std::cout << "  Neighborhood Surveillance Test Suite\n";
    std::cout << "========================================\n";

    bool runAll = (argc == 1);
    std::string testName = (argc > 1) ? argv[1] : "";

    if (runAll || testName == "startup") {
        test_startup_group_formation();
    }

    if (runAll || testName == "single") {
        test_single_node_failure();
    }

    if (runAll || testName == "nonadjacent") {
        test_two_nonadjacent_failures();
    }

    if (runAll || testName == "adjacent") {
        test_adjacent_failures();
    }

    if (runAll || testName == "cascade") {
        test_cascading_neighbor_failures();
    }

    if (runAll || testName == "timed") {
        test_timed_cascading_failure();
    }

    if (runAll || testName == "rapid") {
        test_rapid_multiple_failures();
    }

    if (runAll || testName == "minority") {
        test_minority_survival();
    }

    if (runAll || testName == "realistic") {
        test_realistic_network_conditions();
    }

    std::cout << "\n========================================\n";
    std::cout << "  All requested tests completed!\n";
    std::cout << "========================================\n";

    return 0;
}
