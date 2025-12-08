#pragma once

#include <unordered_map>
#include <vector>
#include <algorithm>

#include <string>
#include <glm/glm.hpp>
#include "membership/MemberInfo.h"
#include "network/Message.h"
#include <unordered_set>
#include <cstdint>
// Forward declaration to avoid circular include
class Drone;

class MembershipState {
public:
    MembershipState(int selfId, double heartbeatInterval, double timeoutDelta);

    // Simulation passes &drones to each MembershipState once
    void setDroneList(std::vector<Drone>* dronesPtr);

    // Called every tick by Drone::update()
    void tick(double logicalTime, const glm::vec3& selfPos);

    // Incoming heartbeat (with sender's position)
    void receiveHeartbeat(int fromId, double time, const glm::vec3& pos);

    // Failure detection
    bool detectFailures(double time);

    // Outgoing heartbeats
    std::vector<Message> generateHeartbeatMessages(double sendTime,
        const glm::vec3& selfPos);

    // Reconfiguration - 3 phase protocol
    std::vector<Message> buildReconfigBroadcast(double currentTime, const glm::vec3& selfPos);
    void sendAck(double time, const glm::vec3& selfPos);
    void installReconfig(const std::vector<uint8_t>& payload,
        double time,
        const glm::vec3& selfPos);

    std::vector<Message> consumePendingReconfigs();
    std::vector<Message> consumePendingHeartbeats();

    // Explicit neighbor setting (startup only)
    void setNeighbors(int leftId, int rightId);

    // Dynamic closest-neighbor selection
    void updateClosestNeighbors(const glm::vec3& selfPos);

    // Queries
    int getGroupId() const;
    std::vector<int> getActiveMembers() const;
    bool isMemberAlive(int id) const;

    void forceDead() { view[selfId].status = DroneStatus::DOWN; }

	int getLeftNeighbor() const { return leftNeighbor; }
	int getRightNeighbor() const { return rightNeighbor; }
	int getSelfId() const { return selfId; }

    void processInit(const Message& m, double time, const glm::vec3& selfPos);
    void processAck(const Message& m, double time);  // time used for position tracking
    void processCommit(const Message& m, double time, const glm::vec3& selfPos);

    void maybeSendAck(double time);
    void maybeSendCommit(double time);

    void forceGroupId(int g) { groupId = g; }

    // Prune members based on formation requirements (future: use distance, formation needs)
    std::vector<int> pruneMembers(const std::unordered_set<int>& acks,
                                   const std::unordered_map<int, glm::vec3>& positions) {
        // For now, return all members who ACKed
        std::vector<int> result(acks.begin(), acks.end());
        std::sort(result.begin(), result.end());  // Deterministic ordering
        return result;
    }

    bool isInGroup() const {
        return groupId != 0;
    }


private:
    int selfId;
    int leftNeighbor = -1;
    int rightNeighbor = -1;

    int groupId = 0;

    double heartbeatInterval;
    double timeoutDelta;
    double nextHeartbeatDue = 0.0;

    // Membership view: ID → member info (pos, time, status)
    std::unordered_map<int, MemberInfo> view;

    // Outgoing message queues
    std::vector<Message> pendingReconfigs;
    std::vector<Message> pendingHeartbeats;

    // Pointer to the global drone list (not owned)
    std::vector<Drone>* allDrones = nullptr;

    // ============ Reconfig protocol state ============
    bool inReconfig = false;
    uint32_t pendingReconfigId = 0;
    double initReceivedTime = 0.0;        // when INIT arrived
    double commitDueTime = 0.0;           // INIT time + 2*Δ₂

    // Members are discovered via ACKs, not proposed in INIT
    std::unordered_set<int> acksReceived;              // who has ACKed
    std::unordered_map<int, glm::vec3> ackPositions;   // positions from ACKs

    int initiatorId = -1;
    double lastCommitTime = -1.0;     // when the last COMMIT was finalized
    double lastInitTime = -1.0;       // when we last sent an INIT

    bool commitSent = false;
    bool ackSent = false;             // have we sent our ACK for current reconfig?
};
