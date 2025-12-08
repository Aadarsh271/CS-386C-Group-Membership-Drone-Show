// tests/MembershipReconfigHarness.cpp

#include "core/Simulation.h"
#include "core/RNG.h"

#include <cassert>
#include <iostream>
#include <set>
#include <vector>
#include <algorithm>

std::mt19937_64 GLOBAL_RNG;

void reseedRNG(uint64_t seed) {
    GLOBAL_RNG.seed(seed);
}


// Make the network as deterministic and "nice" as possible
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

// Helper: snapshot membership from the first UP drone
static void snapshotMembership(Simulation& sim,
    int& groupIdOut,
    std::set<int>& membersOut) {
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

// Test 1: single crash → single reconfig, consistent group & membership
static void test_single_reconfig() {
    std::cout << "[TEST] single_reconfig\n";

    SimulationConfig cfg;
    configureDeterministicNetwork(cfg);

    cfg.numDrones = 8;
    cfg.timeStep = 0.02;          // default
    cfg.heartbeatInterval = 0.2;  // default
    cfg.timeoutDelta = 0.6;       // default
    cfg.deltaSmall = 0.12;
    cfg.deltaLarge = 0.5;
    cfg.reconfigMinInterval = 2.0;

    reseedRNG(1234);
    Simulation sim(cfg);

    // --- Add message-send logging ---
    sim.getNetwork().setVisCallback(
        [](const Message& msg, double sendT, double recvT)
        {
            std::string type;
            switch (msg.type) {
            case MessageType::HEARTBEAT:       type = "HEARTBEAT"; break;
            case MessageType::RECONFIG_INIT:   type = "INIT"; break;
            case MessageType::RECONFIG_ACK:    type = "ACK"; break;
            case MessageType::RECONFIG_COMMIT: type = "COMMIT"; break;
            default:                           type = "OTHER"; break;
            }

            if (msg.receiverId == -1)
                std::cout << "[SEND] t=" << sendT
                << "  " << type
                << "  from=" << msg.senderId
                << "  to=BROADCAST"
                << "  deliverAt=" << recvT << "\n";
            else
                std::cout << "[SEND] t=" << sendT
                << "  " << type
                << "  from=" << msg.senderId
                << "  to=" << msg.receiverId
                << "  deliverAt=" << recvT << "\n";
        }
    );


    // Let heartbeats propagate so everyone learns their neighbors
    while (sim.getTime() < 5.0) {
        sim.step();
    }

    // Kill drone 0 to trigger a reconfig
    sim.killDrone(0);

    std::cout << "----------\nKilled Drone\n---------\n";

    bool reconfigDone = false;
    int finalGroupId = -1;
    std::set<int> finalMembers;

    for (auto& d : sim.getDrones())
        d.getMembership().forceGroupId(0);


    const double maxSimTime = 30.0; //2 * cfg.heartbeatInterval + cfg.deltaSmall + 3 * cfg.deltaLarge + 1e-12; // 30.0;
    while (sim.getTime() < maxSimTime) {
        sim.step();

        // Check if all UP drones have a non-zero groupId and agree on it
        int gid = -1;
        bool allAssigned = true;

        for (auto& d : sim.getDrones()) {
            if (d.getStatus() == DroneStatus::UP) {
                int dg = d.getMembership().getGroupId();
                if (dg == 0) {
                    allAssigned = false;
                    break;
                }
                if (gid == -1) gid = dg;
                else if (dg != gid) {
                    std::cerr << "Split-brain: drone " << d.getId()
                        << " has group " << dg
                        << " vs others " << gid << "\n";
                    assert(false && "Split-brain detected");
                }
            }
        }

        if (allAssigned && gid != -1) {
            reconfigDone = true;
            finalGroupId = gid;
            snapshotMembership(sim, finalGroupId, finalMembers);
            break;
        }
    }

    assert(reconfigDone && "Reconfig did not complete in time");

    // Final membership should not include crashed node 0
    assert(finalMembers.count(0) == 0);

    // All UP drones must appear in finalMembers
    for (auto& d : sim.getDrones()) {
        if (d.getStatus() == DroneStatus::UP) {
            std::cout << (d.getId()) << " Count: " <<  finalMembers.count(d.getId()) << std::endl;
            assert(finalMembers.count(d.getId()) == 1);
        }
    }

    std::cout << "  -> OK, groupId = " << finalGroupId << ", members = { ";
    for (int id : finalMembers) std::cout << id << " ";
    std::cout << "}\n";
}

// Test 2: two reconfigs → second groupId strictly larger than first
static void test_two_reconfigs_monotone_group_id() {
    std::cout << "[TEST] two_reconfigs_monotone_group_id\n";

    SimulationConfig cfg;
    configureDeterministicNetwork(cfg);

    cfg.numDrones = 8;
    cfg.timeStep = 0.02;
    cfg.heartbeatInterval = 0.2;
    cfg.timeoutDelta = 0.6;
    cfg.deltaSmall = 0.12;
    cfg.deltaLarge = 0.5;
    cfg.reconfigMinInterval = 2.0;

    reseedRNG(5678);
    Simulation sim(cfg);

    // --- Add message-send logging ---
    sim.getNetwork().setVisCallback(
        [](const Message& msg, double sendT, double recvT)
        {
            std::string type;
            switch (msg.type) {
            case MessageType::HEARTBEAT:       type = "HEARTBEAT"; break;
            case MessageType::RECONFIG_INIT:   type = "INIT"; break;
            case MessageType::RECONFIG_ACK:    type = "ACK"; break;
            case MessageType::RECONFIG_COMMIT: type = "COMMIT"; break;
            default:                           type = "OTHER"; break;
            }

            if (msg.receiverId == -1)
                std::cout << "[SEND] t=" << sendT
                << "  " << type
                << "  from=" << msg.senderId
                << "  to=BROADCAST"
                << "  deliverAt=" << recvT << "\n";
            else
                std::cout << "[SEND] t=" << sendT
                << "  " << type
                << "  from=" << msg.senderId
                << "  to=" << msg.receiverId
                << "  deliverAt=" << recvT << "\n";
        }
    );


    // Warm-up
    while (sim.getTime() < 5.0) {
        sim.step();
    }

    // First crash: drone 0
    sim.killDrone(0);

    int group1 = -1;
    std::set<int> members1;
    bool reconfig1Done = false;

    const double maxSimTime = 60.0;

    while (sim.getTime() < maxSimTime) {
        sim.step();

        int gid = -1;
        bool allAssigned = true;

        for (auto& d : sim.getDrones()) {
            if (d.getStatus() == DroneStatus::UP) {
                int dg = d.getMembership().getGroupId();
                if (dg == 0) {
                    allAssigned = false;
                    break;
                }
                if (gid == -1) gid = dg;
                else if (dg != gid) {
                    std::cerr << "Split-brain during first reconfig\n";
                    assert(false);
                }
            }
        }

        if (allAssigned && gid != -1) {
            reconfig1Done = true;
            group1 = gid;
            snapshotMembership(sim, group1, members1);
            break;
        }
    }

    assert(reconfig1Done && "First reconfig did not complete");

    // Let the system run a bit with the new membership
    double t0 = sim.getTime();
    while (sim.getTime() < t0 + 5.0) {
        sim.step();
    }

    // Second crash: pick a surviving member (e.g. highest id in members1)
    int toCrash = *members1.rbegin();
	std::cout << "Killing drone " << toCrash << " for second reconfig\n";
    sim.killDrone(toCrash);

    int group2 = -1;
    std::set<int> members2;
    bool reconfig2Done = false;

    while (sim.getTime() < maxSimTime) {
        sim.step();

        int gid = -1;
        bool allAssigned = true;

        for (auto& d : sim.getDrones()) {
            if (d.getStatus() == DroneStatus::UP) {
                int dg = d.getMembership().getGroupId();
                if (dg == 0) {
                    allAssigned = false;
                    break;
                }
                if (gid == -1) gid = dg;
                else if (dg != gid) {
                    std::cerr << "Split-brain during second reconfig\n";
                    assert(false);
                }
            }
        }

        if (allAssigned && gid != group1) {
            reconfig2Done = true;
            group2 = gid;
            snapshotMembership(sim, group2, members2);
            break;
        }
    }

    assert(reconfig2Done && "Second reconfig did not complete");
    assert(group2 > group1 && "groupId should be monotone increasing over time");

    // Sanity: members2 is members1 minus crashed node
    assert(members2.count(toCrash) == 0);

    std::cout << "  -> OK, group1 = " << group1
        << ", group2 = " << group2
        << ", crashed " << toCrash << "\n";
}

int main() {
    //test_single_reconfig();
    test_two_reconfigs_monotone_group_id();
    std::cout << "All membership tests passed.\n";
    return 0;
}
