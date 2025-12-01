#pragma once

#include <unordered_map>
#include <vector>
#include <string>
#include "membership/MemberInfo.h"
#include "network/Message.h"

class MembershipState {
public:
    MembershipState(int selfId, double heartbeatInterval, double timeout);

    void tick(double currentTime); // generate heartbeats if time
    void receiveHeartbeat(int fromId, double time);
    bool detectFailures(double currentTime);

    int getGroupId() const;
    std::vector<int> getActiveMembers() const;
    bool isMemberAlive(int id) const;

    // Called on failure detection or join
    std::vector<Message> buildReconfigBroadcast(double currentTime);

    std::vector<Message> generateHeartbeatMessages(double currentTime);

    void installReconfig(const std::vector<uint8_t>& payload, double time);

    std::vector<Message> consumePendingReconfigs();

    void forceDead() {
        view[selfId].status = DroneStatus::DEAD;
    }


private:
    int selfId;
    int groupId; // time-based epoch

    double heartbeatInterval;
    double timeout;
    double nextHeartbeatDue = 0.0;

    std::unordered_map<int, MemberInfo> view;

    std::vector<Message> pendingReconfigs;

};
