
#include "membership/MembershipState.h"
#include "core/Drone.h"
#include <algorithm>
#include <cstdio>
#include <unordered_set>
#include <iostream>
#include <cstdint>

static inline void encodeUint16(std::vector<uint8_t>& out, uint16_t v) {
    out.push_back(uint8_t((v >> 8) & 0xFF));
    out.push_back(uint8_t(v & 0xFF));
}

static inline uint16_t decodeUint16(const std::vector<uint8_t>& in, size_t pos) {
    return (uint16_t(in[pos]) << 8) | uint16_t(in[pos + 1]);
}

// New: 32-bit big-endian helpers for reconfigId / groupId
static inline void encodeUint32(std::vector<uint8_t>& out, uint32_t v) {
    out.push_back(uint8_t((v >> 24) & 0xFF));
    out.push_back(uint8_t((v >> 16) & 0xFF));
    out.push_back(uint8_t((v >> 8) & 0xFF));
    out.push_back(uint8_t(v & 0xFF));
}

static inline uint32_t decodeUint32(const std::vector<uint8_t>& in, size_t pos) {
    return (uint32_t(in[pos]) << 24) |
        (uint32_t(in[pos + 1]) << 16) |
        (uint32_t(in[pos + 2]) << 8) |
        uint32_t(in[pos + 3]);
}


MembershipState::MembershipState(int selfId, double hbInt, double timeout)
    : selfId(selfId),
    heartbeatInterval(hbInt),
    timeoutDelta(timeout)
{
    view[selfId] = { DroneStatus::UP, -1e9, glm::vec3(0,0,0) };
}

void MembershipState::setDroneList(std::vector<Drone>* ptr) {
    allDrones = ptr;

    // Initialize membership view for every known drone
    view.clear();
    for (auto& d : *allDrones) {
        MemberInfo info;
        info.status = (d.getId() == selfId) ? DroneStatus::UP : DroneStatus::DOWN;
        info.lastHeartbeatTime = -1;
        info.lastKnownPosition = glm::vec3(0, 0, 0);
        info.groupHint = 0;// m.groupHint;   // store neighbor’s group id

        view[d.getId()] = info;
    }
}

//
// =============================
//            TICK
// =============================
void MembershipState::tick(double logicalTime, const glm::vec3& selfPos)
{
    // Send heartbeats
    if (logicalTime >= nextHeartbeatDue) {
        nextHeartbeatDue = logicalTime + heartbeatInterval;

        auto hb = generateHeartbeatMessages(logicalTime, selfPos);
        pendingHeartbeats.insert(pendingHeartbeats.end(), hb.begin(), hb.end());
    }

    if (inReconfig) {
        maybeSendCommit(logicalTime);
    }

    if (detectFailures(logicalTime))
    {
        double r = allDrones->at(selfId).config.reconfigMinInterval;

        // Rate limit: do not initiate if too soon after last COMMIT or last INIT
        if (logicalTime >= lastCommitTime + r &&
            logicalTime >= lastInitTime + r)
        {
            auto rc = buildReconfigBroadcast(logicalTime);
            pendingReconfigs.insert(pendingReconfigs.end(), rc.begin(), rc.end());
            lastInitTime = logicalTime;
        }
    }

    // 1. Not in a group?
    if (!isInGroup()) {
        double r = allDrones->at(selfId).config.reconfigMinInterval;
        if (lastInitTime + r <= logicalTime) {
            std::cout << "[DEBUG] Drone " << selfId << " initiating reconfig at t=" << logicalTime << "\n";
            auto msgs = buildReconfigBroadcast(logicalTime);
            std::cout << "[DEBUG] Drone " << selfId << " proposing " << pendingMembers.size() << " members\n";
			lastInitTime = logicalTime;
        }
    }

}

//
// =============================
//    HEARTBEAT RECEIVE
// =============================
void MembershipState::receiveHeartbeat(int fromId,
    double time,
    const glm::vec3& pos)
{
    auto& info = view[fromId];
    info.status = DroneStatus::UP;
    info.lastHeartbeatTime = time;
    info.lastKnownPosition = pos;
}

//
// =============================
//    FAILURE DETECTION
// =============================
bool MembershipState::detectFailures(double time)
{
    bool changed = false;
    int neigh[2] = { leftNeighbor, rightNeighbor };

    for (int n : neigh) {
        if (n == -1) continue;

        auto it = view.find(n);
        if (it == view.end()) continue;

        auto& info = it->second;

        if (info.lastHeartbeatTime < 0.0) continue;

        if (info.status == DroneStatus::UP &&
            (time - info.lastHeartbeatTime - heartbeatInterval) > timeoutDelta)
        {
            info.status = DroneStatus::DOWN;
            changed = true;
        }
    }

    return changed;
}

//
// =============================
//    HEARTBEAT GENERATION
// =============================
std::vector<Message>
MembershipState::generateHeartbeatMessages(double sendTime,
    const glm::vec3& selfPos)
{
    std::vector<Message> msgs;

    if (leftNeighbor != -1) {
        Message m;
        m.type = MessageType::HEARTBEAT;
        m.senderId = selfId;
        m.receiverId = leftNeighbor;
        m.timestamp = sendTime;
        m.pos = selfPos;
        m.groupHint = groupId;
        msgs.push_back(m);
    }

    if (rightNeighbor != -1) {
        Message m;
        m.type = MessageType::HEARTBEAT;
        m.senderId = selfId;
        m.receiverId = rightNeighbor;
        m.timestamp = sendTime;
        m.pos = selfPos; 
        m.groupHint = groupId;
        msgs.push_back(m);
    }

    return msgs;
}

//
// =============================
//   RECONFIG BROADCAST
// =============================
std::vector<Message>
MembershipState::buildReconfigBroadcast(double currentTime)
{
    std::vector<Message> out;

    // List all UP drones
    // Use *membership view* not physical drone status
    std::cout << "[BUILD RECONFIG] Drone " << selfId << " building INIT at t=" << currentTime << "\n";

    // List all UP drones
    std::vector<int> members;
    for (auto& [id, info] : view) {
        if (info.status == DroneStatus::UP)
            members.push_back(id);
    }

    std::cout << "[BUILD RECONFIG] Found " << members.size() << " UP members\n";



    acksReceived.clear();
    acksReceived.insert(selfId);


    // New reconfigId
    // New 32-bit reconfigId based on time in ms
    uint32_t reconfigId = static_cast<uint32_t>(currentTime * 1000.0);
    pendingReconfigId = reconfigId;
    inReconfig = true;

    initReceivedTime = currentTime;
    commitDueTime = currentTime + 2 * allDrones->at(selfId).config.deltaLarge;
    pendingMembers = members;
    initiatorId = selfId;

    std::vector<uint8_t> payload;

    // Encode 32-bit reconfigId
    encodeUint32(payload, reconfigId);

    // Encode initiatorId (still 16-bit; we assume < 65535 drones)
    encodeUint16(payload, uint16_t(initiatorId));


    // Encode members list
    for (int id : members)
        encodeUint16(payload, uint16_t(id));

    Message init;
    init.type = MessageType::RECONFIG_INIT;
    init.senderId = selfId;
    init.receiverId = -1;
    init.timestamp = currentTime;
    init.payload = std::move(payload);

    out.push_back(init);

    std::cout << "[BUILD RECONFIG] Returning " << out.size() << " messages\n";

    return out;
}

//
// =============================
//      INSTALL RECONFIG
// =============================
void MembershipState::installReconfig(const std::vector<uint8_t>& payload,
    double time,
    const glm::vec3& selfPos)
{
    view.clear();

    size_t idx = 0;
    while (idx + 1 < payload.size()) {
        uint16_t id = decodeUint16(payload, idx);
        idx += 2;

        MemberInfo mi;
        mi.status = DroneStatus::UP;
        mi.lastHeartbeatTime = time;
        mi.lastKnownPosition = glm::vec3(0, 0, 0);

        view[int(id)] = mi;
    }

    view[selfId].lastKnownPosition = selfPos;
    updateClosestNeighbors(selfPos);
}

//
// =============================
//    GETTERS / CONSUMERS
// =============================
std::vector<Message> MembershipState::consumePendingReconfigs() {
    auto out = pendingReconfigs;
    pendingReconfigs.clear();
    return out;
}

std::vector<Message> MembershipState::consumePendingHeartbeats() {
    auto out = pendingHeartbeats;
    pendingHeartbeats.clear();
    return out;
}

int MembershipState::getGroupId() const {
    return groupId;
}

std::vector<int> MembershipState::getActiveMembers() const {
    std::vector<int> alive;
    for (auto& [id, info] : view)
        if (info.status == DroneStatus::UP)
            alive.push_back(id);
    return alive;
}

bool MembershipState::isMemberAlive(int id) const {
    auto it = view.find(id);
    return it != view.end() && it->second.status == DroneStatus::UP;
}

void MembershipState::setNeighbors(int leftId, int rightId) {
    leftNeighbor = leftId;
    rightNeighbor = rightId;
}

//
// =============================
//   CLOSEST NEIGHBORS
// =============================
void MembershipState::updateClosestNeighbors(const glm::vec3& selfPos)
{
    if (!allDrones) return;

    std::vector<std::pair<float, int>> dist; // (distance, id)

    for (const auto& d : *allDrones) {
        int id = d.getId();
        if (id == selfId) continue;

        // Only consider drones that are actually UP
        if (d.getStatus() != DroneStatus::UP)
            continue;

        float d2 = glm::distance(selfPos, d.getPosition());
        dist.emplace_back(d2, id);
    }

    if (dist.empty()) {
        leftNeighbor = rightNeighbor = -1;
        return;
    }

    std::sort(dist.begin(), dist.end(),
        [](const auto& a, const auto& b) {
            return a.first < b.first;
        });

    if (dist.size() == 1) {
        // Only one other drone: both neighbors = that drone
        leftNeighbor = dist[0].second;
        rightNeighbor = dist[0].second;
    }
    else {
        // Two distinct closest drones
        leftNeighbor = dist[0].second;
        rightNeighbor = dist[1].second;
    }
}

void MembershipState::processInit(const Message& m,
    double time,
    const glm::vec3& selfPos)
{
    const auto& p = m.payload;
    //if (p.size() < 6) return;

    size_t idx = 0;
    uint32_t recId = decodeUint32(p, idx);
    idx += 4;
    uint16_t initId = decodeUint16(p, idx);
    idx += 2;

    // Tie-breaking: accept if this INIT is "better"
    if (inReconfig) {
        // Keep the INIT with higher reconfigId
        if (recId < pendingReconfigId) return;

        // If same reconfigId, keep the one with HIGHER initiatorId
        if (recId == pendingReconfigId && initId <= initiatorId) return;

        // This INIT is better (higher recId, or same recId but higher initId)
    }

    // Accept this INIT as the current reconfig
    inReconfig = true;
    pendingReconfigId = recId;
    initiatorId = initId;

    initReceivedTime = time;
    commitDueTime = time + 2 * allDrones->at(selfId).config.deltaLarge;

    // Parse member list from payload
    pendingMembers.clear();
    while (idx + 1 < p.size()) {
        uint16_t id = decodeUint16(p, idx);
        idx += 2;
        pendingMembers.push_back(int(id));
    }

    // Reset ACKs for this new reconfig
    acksReceived.clear();

    // Install provisional membership
    view.clear();
    for (int id : pendingMembers) {
        MemberInfo mi;
        mi.status = DroneStatus::UP;
        mi.lastHeartbeatTime = time;
        mi.lastKnownPosition = glm::vec3(0, 0, 0);
        view[id] = mi;
    }
    view[selfId].lastKnownPosition = selfPos;

    // Immediately emit ACK
    Message ack;
    ack.type = MessageType::RECONFIG_ACK;
    ack.senderId = selfId;
    ack.receiverId = -1;
    ack.timestamp = time;

    std::vector<uint8_t> ackp;
    encodeUint32(ackp, recId);
    encodeUint16(ackp, uint16_t(selfId));

    ack.payload = std::move(ackp);
    pendingReconfigs.push_back(ack);
    std::cout << "[ACK QUEUED] Drone " << selfId
        << " queued ACK for recId=" << recId
        << " from initiator=" << initId << "\n";

}

void MembershipState::processAck(const Message& m, double /*time*/)
{
    if (!inReconfig) return;
    std::cout << "Drone " << selfId << " processing ACK from "
        << m.senderId << " for recId=" << pendingReconfigId << "\n";

    
    const auto& p = m.payload;
    // 4 bytes reconfigId + 2 bytes senderId
    if (p.size() < 6) return;

    size_t idx = 0;

    uint32_t recId = decodeUint32(p, idx);
    idx += 4;

    uint16_t sender = decodeUint16(p, idx);
    idx += 2;

    if (recId != pendingReconfigId) return;


    if (initiatorId != selfId) return; // only initiator processes ACKs

    acksReceived.insert(int(sender));
}


void MembershipState::maybeSendCommit(double time)
{
    if (!inReconfig) return;

    if (commitSent) return;

    // Only initiator may commit
    if (selfId != initiatorId) return;

    
    std::cout << "[COMMIT CHECK] Drone " << selfId
        << " checking commit at t=" << time
        << ", commitDueTime=" << commitDueTime << "\n";

    if (time >= commitDueTime)
    {
        std::cout << "[COMMIT READY] Drone " << selfId
            << " sending COMMIT, acksReceived=" << acksReceived.size() << "\n";
    
        if (selfId == initiatorId) {
            std::cout << "Initiator " << selfId
                << " building COMMIT for recId=" << pendingReconfigId << "\n";
            std::cout << "  pendingMembers: ";
            for (int id : pendingMembers) std::cout << id << " ";
            std::cout << "\n  acksReceived: ";
            for (int id : acksReceived) std::cout << id << " ";
            std::cout << "\n";
        }

        std::vector<uint8_t> payload;
        encodeUint32(payload, pendingReconfigId);          // 32-bit reconfigId
        encodeUint16(payload, uint16_t(initiatorId));      // 16-bit initiatorId

        // pruneMembers should return set of drones required for initiator drone's formation
        auto new_members = pruneMembers(acksReceived);

        // Commit should include ONLY ACKed members
        for (int id : new_members)
            if (acksReceived.count(id))         // <-- crucial
                encodeUint16(payload, uint16_t(id));



        Message c;
        c.type = MessageType::RECONFIG_COMMIT;
        c.senderId = selfId;
        c.receiverId = -1;
        c.timestamp = time;
        c.payload = std::move(payload);

        pendingReconfigs.push_back(c);

        commitSent = true;

        std::cout << "[COMMIT DEBUG] acksReceived = { ";
        for (int id : acksReceived) std::cout << id << " ";
        std::cout << "}\n";

        std::cout << "[COMMIT DEBUG] final payload members = { ";
        for (int id : new_members) std::cout << id << " ";
        std::cout << "}\n";

    }
}


void MembershipState::processCommit(const Message& m,
    double time,
    const glm::vec3& selfPos)
{
    const auto& p = m.payload;
    // 4 bytes reconfigId + 2 bytes initiatorId
    if (p.size() < 6) return;

    size_t idx = 0;

    uint32_t recId = decodeUint32(p, idx);
    idx += 4;

    uint16_t initId = decodeUint16(p, idx);
    idx += 2;

	std::cout << "Drone " << selfId << " processing COMMIT " << recId << "\n";
	std::cout << "  pendingReconfigId = " << pendingReconfigId << "\n";
    
    if (!inReconfig || recId != pendingReconfigId) {
        return;
    }

    // Group ID is now 32-bit on the wire; we store it as int
    groupId = static_cast<int>(recId);

    // Parse final member list
    std::vector<int> finalMembers;
    while (idx + 1 < p.size()) {
        uint16_t id = decodeUint16(p, idx);
        idx += 2;
        finalMembers.push_back(int(id));
    }

    // Install final membership
    view.clear();
    for (int id : finalMembers) {
        MemberInfo mi;
        mi.status = DroneStatus::UP;
        mi.lastHeartbeatTime = time;
        mi.lastKnownPosition = glm::vec3(0, 0, 0);
        view[id] = mi;
    }
    view[selfId].lastKnownPosition = selfPos;

    // Update neighbors using FINAL membership
    updateClosestNeighbors(selfPos);

    // Cleanup reconfig state
    inReconfig = false;
    commitSent = false;
    pendingReconfigId = 0;
    initiatorId = -1;
    pendingMembers.clear();
    acksReceived.clear();

    lastCommitTime = time;

}
