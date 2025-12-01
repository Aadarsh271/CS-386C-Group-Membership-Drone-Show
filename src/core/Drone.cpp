#include "core/Drone.h"

Drone::Drone(int id, const Vec3& startPos)
    : id(id), position(startPos) {
}

int Drone::getId() const { return id; }
const Vec3& Drone::getPosition() const { return position; }
DroneStatus Drone::getStatus() const { return status; }

void Drone::tick(double time) {
    membership.tick(time);
}

std::vector<Message> Drone::generateMessages(double time) {
    std::vector<Message> msgs;

    // Include heartbeat messages
    auto hb = membership.generateHeartbeatMessages(time);
    msgs.insert(msgs.end(), hb.begin(), hb.end());

    // Include reconfig messages if any
    auto rc = membership.consumePendingReconfigs();
    msgs.insert(msgs.end(), rc.begin(), rc.end());

    return msgs;
}

void Drone::receiveMessages(const std::vector<Message>& msgs) {
    for (auto& m : msgs) {
        if (m.type == MessageType::HEARTBEAT)
            membership.receiveHeartbeat(m.senderId, m.timestamp);
        else if (m.type == MessageType::BROADCAST) {
            membership.installReconfig(m.payload, m.timestamp);
        }

    }
}


void Drone::applyControl(double /*dt*/) {
    // TODO: motion controller
}

void Drone::integrate(double dt) {
    position += velocity * (float)dt;
}
