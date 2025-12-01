#include "membership/MembershipState.h"
#include <algorithm>

MembershipState::MembershipState(int selfId, double hbInt, double timeout)
    : selfId(selfId), heartbeatInterval(hbInt), timeout(timeout) {

    // Initialize group ID to join time or 0 initially
    groupId = 0;
    view[selfId] = { DroneStatus::ALIVE, 0.0 };
}

int MembershipState::getGroupId() const {
    return groupId;
}

std::vector<int> MembershipState::getActiveMembers() const {
    std::vector<int> alive;
    for (auto& [id, info] : view)
        if (info.status == DroneStatus::ALIVE)
            alive.push_back(id);
    return alive;
}

bool MembershipState::isMemberAlive(int id) const {
    if (auto it = view.find(id); it != view.end())
        return it->second.status == DroneStatus::ALIVE;
    return false;
}

void MembershipState::tick(double time) {
    if (time >= nextHeartbeatDue) {
        nextHeartbeatDue = time + heartbeatInterval;
        // heartbeat generation handled externally
    }

    if (detectFailures(time)) {
        // Build & request a reconfig broadcast on failure
        pendingReconfigs = buildReconfigBroadcast(time);
    }
}

std::vector<Message> MembershipState::consumePendingReconfigs() {
    auto msgs = pendingReconfigs;
    pendingReconfigs.clear();
    return msgs;
}


void MembershipState::receiveHeartbeat(int fromId, double time) {
    view[fromId].status = DroneStatus::ALIVE;
    view[fromId].lastHeartbeatTime = time;
}

bool MembershipState::detectFailures(double time) {
    bool changed = false;
    for (auto& [id, info] : view) {
        if (id == selfId) continue;

        if (info.status == DroneStatus::ALIVE &&
            (time - info.lastHeartbeatTime) > timeout)
        {
            info.status = DroneStatus::DEAD;
            changed = true;
        }
    }
    return changed;
}

std::vector<Message> MembershipState::generateHeartbeatMessages(double time) {
    Message msg;
    msg.senderId = selfId;
    msg.type = MessageType::HEARTBEAT;
    msg.timestamp = time;

    // no payload for now
    msg.receiverId = -1; // broadcast locally filtered in Drone

    return { msg };
}

std::vector<Message> MembershipState::buildReconfigBroadcast(double currentTime) {
    groupId = (int)(currentTime * 1000);

    std::vector<uint8_t> payload;
    auto alive = getActiveMembers();
    payload.reserve(alive.size());

    for (int id : alive)
        payload.push_back((uint8_t)id);

    Message m;
    m.senderId = selfId;
    m.receiverId = -1; // broadcast
    m.type = MessageType::BROADCAST;
    m.timestamp = currentTime;
    m.payload = payload;

    return { m };
}

void MembershipState::installReconfig(const std::vector<uint8_t>& payload, double time) {
    groupId = (int)(time * 1000);
    view.clear();
    for (uint8_t id : payload) {
        view[id].status = DroneStatus::ALIVE;
        view[id].lastHeartbeatTime = time;
    }
}

