
#include "membership/MembershipState.h"
#include "core/Drone.h"
#include <algorithm>
#include <cstdio>
#include <unordered_set>
#include <iostream>
#include <cstdint>

// =============================
//   ENCODING/DECODING HELPERS
// =============================

static inline void encodeUint16(std::vector<uint8_t>& out, uint16_t v) {
    out.push_back(uint8_t((v >> 8) & 0xFF));
    out.push_back(uint8_t(v & 0xFF));
}

static inline uint16_t decodeUint16(const std::vector<uint8_t>& in, size_t pos) {
    return (uint16_t(in[pos]) << 8) | uint16_t(in[pos + 1]);
}

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

// Encode a float as 4 bytes (IEEE 754)
static inline void encodeFloat(std::vector<uint8_t>& out, float v) {
    uint32_t bits;
    std::memcpy(&bits, &v, sizeof(float));
    encodeUint32(out, bits);
}

static inline float decodeFloat(const std::vector<uint8_t>& in, size_t pos) {
    uint32_t bits = decodeUint32(in, pos);
    float v;
    std::memcpy(&v, &bits, sizeof(float));
    return v;
}

static inline void encodeVec3(std::vector<uint8_t>& out, const glm::vec3& v) {
    encodeFloat(out, v.x);
    encodeFloat(out, v.y);
    encodeFloat(out, v.z);
}

static inline glm::vec3 decodeVec3(const std::vector<uint8_t>& in, size_t pos) {
    return glm::vec3(
        decodeFloat(in, pos),
        decodeFloat(in, pos + 4),
        decodeFloat(in, pos + 8)
    );
}

// =============================
//      CONSTRUCTOR
// =============================

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
    // At startup, assume all drones are UP (they just started)
    view.clear();
    for (auto& d : *allDrones) {
        MemberInfo info;
        info.status = DroneStatus::UP;
        info.lastHeartbeatTime = 0.0;
        info.lastKnownPosition = d.getPosition();
        info.groupHint = 0;

        view[d.getId()] = info;
    }

    // Initialize neighbors based on initial positions
    if (!allDrones->empty()) {
        updateClosestNeighbors(allDrones->at(selfId).getPosition());
    }
}

// =============================
//            TICK
// =============================
void MembershipState::tick(double logicalTime, const glm::vec3& selfPos)
{
    // Send heartbeats periodically
    if (logicalTime >= nextHeartbeatDue) {
        nextHeartbeatDue = logicalTime + heartbeatInterval;
        auto hb = generateHeartbeatMessages(logicalTime, selfPos);
        pendingHeartbeats.insert(pendingHeartbeats.end(), hb.begin(), hb.end());
    }

    // If in reconfig, check if we should send COMMIT (initiator only)
    if (inReconfig) {
        maybeSendCommit(logicalTime);
    }

    // Check for failures that might trigger a reconfig
    if (detectFailures(logicalTime)) {
        double r = allDrones->at(selfId).config.reconfigMinInterval;
        if (logicalTime >= lastCommitTime + r &&
            logicalTime >= lastInitTime + r &&
            !inReconfig)
        {
            auto rc = buildReconfigBroadcast(logicalTime, selfPos);
            pendingReconfigs.insert(pendingReconfigs.end(), rc.begin(), rc.end());
            lastInitTime = logicalTime;
        }
    }

    // Not in a group and not already in a reconfig? Initiate one.
    if (!isInGroup() && !inReconfig) {
        double r = allDrones->at(selfId).config.reconfigMinInterval;
        if (lastInitTime + r <= logicalTime) {
            std::cout << "[DEBUG] Drone " << selfId << " initiating reconfig at t=" << logicalTime << "\n";
            auto msgs = buildReconfigBroadcast(logicalTime, selfPos);
            pendingReconfigs.insert(pendingReconfigs.end(), msgs.begin(), msgs.end());
            lastInitTime = logicalTime;
        }
    }
}

// =============================
//    HEARTBEAT RECEIVE
// =============================
void MembershipState::receiveHeartbeat(int fromId, double time, const glm::vec3& pos)
{
    auto& info = view[fromId];
    info.status = DroneStatus::UP;
    info.lastHeartbeatTime = time;
    info.lastKnownPosition = pos;
}

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

// =============================
//    HEARTBEAT GENERATION
// =============================
std::vector<Message> MembershipState::generateHeartbeatMessages(double sendTime, const glm::vec3& selfPos)
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

    if (rightNeighbor != -1 && rightNeighbor != leftNeighbor) {
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

// =============================
//   PHASE 1: INIT BROADCAST
// =============================
// INIT now contains only: reconfigId, initiatorId
// Members are discovered via ACKs, not proposed upfront
std::vector<Message> MembershipState::buildReconfigBroadcast(double currentTime, const glm::vec3& selfPos)
{
    std::vector<Message> out;

    std::cout << "[INIT] Drone " << selfId << " building INIT at t=" << currentTime << "\n";

    // Generate reconfigId from timestamp
    uint32_t reconfigId = static_cast<uint32_t>(currentTime * 1000.0);

    // Enter reconfig state
    inReconfig = true;
    pendingReconfigId = reconfigId;
    initiatorId = selfId;
    initReceivedTime = currentTime;
    commitDueTime = currentTime + 2 * allDrones->at(selfId).config.deltaLarge;
    commitSent = false;
    ackSent = false;

    // Clear ACK tracking - members will be discovered via ACKs
    acksReceived.clear();
    ackPositions.clear();

    // Build INIT payload: [reconfigId(4)][initiatorId(2)]
    std::vector<uint8_t> payload;
    encodeUint32(payload, reconfigId);
    encodeUint16(payload, uint16_t(selfId));

    Message init;
    init.type = MessageType::RECONFIG_INIT;
    init.senderId = selfId;
    init.receiverId = -1;  // Broadcast
    init.timestamp = currentTime;
    init.pos = selfPos;
    init.payload = std::move(payload);

    out.push_back(init);

    // Initiator also sends its own ACK immediately
    sendAck(currentTime, selfPos);

    std::cout << "[INIT] Drone " << selfId << " sent INIT with reconfigId=" << reconfigId << "\n";

    return out;
}

// =============================
//   HELPER: Send ACK
// =============================
void MembershipState::sendAck(double time, const glm::vec3& selfPos)
{
    if (ackSent) return;

    // Build ACK payload: [reconfigId(4)][senderId(2)][pos.x(4)][pos.y(4)][pos.z(4)]
    std::vector<uint8_t> payload;
    encodeUint32(payload, pendingReconfigId);
    encodeUint16(payload, uint16_t(selfId));
    encodeVec3(payload, selfPos);

    Message ack;
    ack.type = MessageType::RECONFIG_ACK;
    ack.senderId = selfId;
    ack.receiverId = -1;  // Broadcast
    ack.timestamp = time;
    ack.pos = selfPos;
    ack.payload = std::move(payload);

    pendingReconfigs.push_back(ack);
    ackSent = true;

    std::cout << "[ACK] Drone " << selfId << " sent ACK for reconfigId=" << pendingReconfigId
              << " pos=(" << selfPos.x << "," << selfPos.y << "," << selfPos.z << ")\n";
}

// =============================
//      INSTALL RECONFIG (legacy)
// =============================
void MembershipState::installReconfig(const std::vector<uint8_t>& payload, double time, const glm::vec3& selfPos)
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

// =============================
//   CLOSEST NEIGHBORS
// =============================
void MembershipState::updateClosestNeighbors(const glm::vec3& selfPos)
{
    if (!allDrones) return;

    std::vector<std::pair<float, int>> dist;

    for (const auto& d : *allDrones) {
        int id = d.getId();
        if (id == selfId) continue;

        // Only consider drones that are both physically UP and in our membership view
        if (d.getStatus() != DroneStatus::UP)
            continue;

        auto it = view.find(id);
        if (it == view.end() || it->second.status != DroneStatus::UP)
            continue;

        float d2 = glm::distance(selfPos, d.getPosition());
        dist.emplace_back(d2, id);
    }

    if (dist.empty()) {
        leftNeighbor = rightNeighbor = -1;
        return;
    }

    std::sort(dist.begin(), dist.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });

    if (dist.size() == 1) {
        leftNeighbor = dist[0].second;
        rightNeighbor = dist[0].second;
    }
    else {
        leftNeighbor = dist[0].second;
        rightNeighbor = dist[1].second;
    }
}

// =============================
//   PHASE 1: PROCESS INIT
// =============================
// Upon receiving INIT, node enters reconfig and sends ACK
void MembershipState::processInit(const Message& m, double time, const glm::vec3& selfPos)
{
    const auto& p = m.payload;
    if (p.size() < 6) return;

    size_t idx = 0;
    uint32_t recId = decodeUint32(p, idx);
    idx += 4;
    uint16_t initId = decodeUint16(p, idx);
    idx += 2;

    std::cout << "[INIT RECV] Drone " << selfId << " received INIT from " << initId
              << " recId=" << recId << "\n";

    // Tie-breaking for concurrent INITs:
    // - Higher reconfigId wins
    // - If equal, LOWER initiatorId wins (first come, first served based on id ordering)
    if (inReconfig) {
        if (recId < pendingReconfigId) {
            std::cout << "[INIT RECV] Drone " << selfId << " rejecting: lower recId\n";
            return;
        }
        if (recId == pendingReconfigId && initId >= initiatorId) {
            std::cout << "[INIT RECV] Drone " << selfId << " rejecting: same recId, higher/equal initId\n";
            return;
        }
        // This INIT is better - switch to it
        std::cout << "[INIT RECV] Drone " << selfId << " switching to better INIT\n";
    }

    // Accept this INIT
    inReconfig = true;
    pendingReconfigId = recId;
    initiatorId = initId;
    initReceivedTime = time;
    commitDueTime = time + 2 * allDrones->at(selfId).config.deltaLarge;
    commitSent = false;
    ackSent = false;

    // Clear ACK tracking for new reconfig
    acksReceived.clear();
    ackPositions.clear();

    // Send our ACK
    sendAck(time, selfPos);
}

// =============================
//   PHASE 2: PROCESS ACK
// =============================
// All nodes collect ACKs to learn about participants and their positions
void MembershipState::processAck(const Message& m, double time)
{
    if (!inReconfig) return;

    const auto& p = m.payload;
    if (p.size() < 18) return;  // 4 + 2 + 12 (reconfigId + senderId + vec3)

    size_t idx = 0;
    uint32_t recId = decodeUint32(p, idx);
    idx += 4;
    uint16_t sender = decodeUint16(p, idx);
    idx += 2;
    glm::vec3 pos = decodeVec3(p, idx);

    if (recId != pendingReconfigId) return;

    std::cout << "[ACK RECV] Drone " << selfId << " received ACK from " << sender
              << " pos=(" << pos.x << "," << pos.y << "," << pos.z << ")\n";

    // Store position from ACK - all nodes track this for neighbor determination
    ackPositions[int(sender)] = pos;

    // Update view with position
    auto it = view.find(int(sender));
    if (it != view.end()) {
        it->second.lastKnownPosition = pos;
        it->second.lastHeartbeatTime = time;
        it->second.status = DroneStatus::UP;
    } else {
        // New member discovered via ACK
        MemberInfo mi;
        mi.status = DroneStatus::UP;
        mi.lastHeartbeatTime = time;
        mi.lastKnownPosition = pos;
        view[int(sender)] = mi;
    }

    // Track ACK for initiator's commit decision
    acksReceived.insert(int(sender));
}

// =============================
//   PHASE 3: SEND COMMIT
// =============================
// Initiator waits for ACKs, then broadcasts COMMIT with final member list
void MembershipState::maybeSendCommit(double time)
{
    if (!inReconfig) return;
    if (commitSent) return;
    if (selfId != initiatorId) return;

    if (time < commitDueTime) return;

    std::cout << "[COMMIT] Drone " << selfId << " sending COMMIT at t=" << time
              << ", acksReceived=" << acksReceived.size() << "\n";

    // Determine final membership from ACKs received
    auto finalMembers = pruneMembers(acksReceived, ackPositions);

    std::cout << "[COMMIT] Final members: ";
    for (int id : finalMembers) std::cout << id << " ";
    std::cout << "\n";

    // Build COMMIT payload: [reconfigId(4)][initiatorId(2)][numMembers(2)]
    //                       [member1_id(2)][member1_pos(12)]...
    std::vector<uint8_t> payload;
    encodeUint32(payload, pendingReconfigId);
    encodeUint16(payload, uint16_t(initiatorId));
    encodeUint16(payload, uint16_t(finalMembers.size()));

    for (int id : finalMembers) {
        encodeUint16(payload, uint16_t(id));
        auto posIt = ackPositions.find(id);
        if (posIt != ackPositions.end()) {
            encodeVec3(payload, posIt->second);
        } else {
            encodeVec3(payload, glm::vec3(0, 0, 0));
        }
    }

    Message c;
    c.type = MessageType::RECONFIG_COMMIT;
    c.senderId = selfId;
    c.receiverId = -1;  // Broadcast
    c.timestamp = time;
    c.payload = std::move(payload);

    pendingReconfigs.push_back(c);
    commitSent = true;
}

// =============================
//   PHASE 3: PROCESS COMMIT
// =============================
// All nodes install final membership and update neighbors using received positions
void MembershipState::processCommit(const Message& m, double time, const glm::vec3& selfPos)
{
    const auto& p = m.payload;
    if (p.size() < 8) return;  // 4 + 2 + 2

    size_t idx = 0;
    uint32_t recId = decodeUint32(p, idx);
    idx += 4;
    uint16_t initId = decodeUint16(p, idx);
    idx += 2;
    uint16_t numMembers = decodeUint16(p, idx);
    idx += 2;

    std::cout << "[COMMIT RECV] Drone " << selfId << " received COMMIT recId=" << recId
              << " from initiator=" << initId << " numMembers=" << numMembers << "\n";

    if (!inReconfig || recId != pendingReconfigId) {
        std::cout << "[COMMIT RECV] Drone " << selfId << " rejecting: not in matching reconfig\n";
        return;
    }

    // Set group ID
    groupId = static_cast<int>(recId);

    // Parse and install final membership with positions
    view.clear();
    for (uint16_t i = 0; i < numMembers && idx + 14 <= p.size(); ++i) {
        uint16_t memberId = decodeUint16(p, idx);
        idx += 2;
        glm::vec3 memberPos = decodeVec3(p, idx);
        idx += 12;

        MemberInfo mi;
        mi.status = DroneStatus::UP;
        mi.lastHeartbeatTime = time;
        mi.lastKnownPosition = memberPos;
        view[int(memberId)] = mi;

        std::cout << "[COMMIT RECV] Drone " << selfId << " added member " << memberId
                  << " pos=(" << memberPos.x << "," << memberPos.y << "," << memberPos.z << ")\n";
    }

    // Make sure we're in the view with our current position
    view[selfId].lastKnownPosition = selfPos;

    // Update neighbors using final membership positions
    updateClosestNeighbors(selfPos);

    std::cout << "[COMMIT RECV] Drone " << selfId << " final neighbors: left=" << leftNeighbor
              << " right=" << rightNeighbor << "\n";

    // Clear reconfig state
    inReconfig = false;
    commitSent = false;
    ackSent = false;
    pendingReconfigId = 0;
    initiatorId = -1;
    acksReceived.clear();
    ackPositions.clear();

    lastCommitTime = time;

    std::cout << "[COMMIT RECV] Drone " << selfId << " completed reconfig, groupId=" << groupId << "\n";
}

// Legacy function - not used in new protocol
void MembershipState::maybeSendAck(double time) {
    // ACKs are now sent immediately upon receiving INIT
}
