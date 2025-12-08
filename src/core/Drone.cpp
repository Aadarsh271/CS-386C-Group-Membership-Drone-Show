#include <iostream>
#include "core/Drone.h"

Drone::Drone(int id, const Vec3& startPos, const SimulationConfig& cfg)
    : id(id),
    position(startPos),
    config(cfg),
    membership(id, cfg.heartbeatInterval, cfg.deltaSmall)
{
}

int Drone::getId() const { return id; }
const Vec3& Drone::getPosition() const { return position; }
DroneStatus Drone::getStatus() const { return status; }

//
// ================================
//   INTERNAL FAULT STEP FUNCTION
// ================================
//
void Drone::stepFaults(double simTime) {
    using FS = InternalFaultState;

    std::uniform_real_distribution<double> uni(0.0, 1.0);

    // PERMANENT DEATH
    if (internalState == FS::PERMANENTLY_DEAD) {
        status = DroneStatus::DOWN;
        return;
    }

    // TEMPORARY CRASH (waiting to recover)
    if (internalState == FS::TEMPORARILY_CRASHED) {
        if (simTime >= recoveryTime) {
            internalState = FS::RUNNING;
        }
        else {
            // Still down
            status = DroneStatus::DOWN;
            return;
        }
    }

    // RUNNING → possible crash
    if (internalState == FS::RUNNING && config.enableCrashes) {
        if (uni(GLOBAL_RNG) < config.crashRate) {
            // Crash now
            crashStartTime = simTime;
            internalState = FS::TEMPORARILY_CRASHED;

            // Permanent?
            if (uni(GLOBAL_RNG) < config.pPermanentCrash) {
                internalState = FS::PERMANENTLY_DEAD;
            }
            else {
                // Assign recovery time
                std::uniform_real_distribution<double> recDist(
                    config.recoveryMin, config.recoveryMax);
                recoveryTime = simTime + recDist(GLOBAL_RNG);
            }

            status = DroneStatus::DOWN;
            return;
        }
    }

    // If we reach here, drone is alive
    status = DroneStatus::UP;
}

//
// ================================
//       UPDATE (called each tick)
// ================================
//
void Drone::update(double simTime) {
    stepFaults(simTime);

    // If physically dead or crashed: no messages, no membership activity
    if (status == DroneStatus::DOWN) return;

    // Membership heartbeat + failure detection
    membership.tick(simTime, position);
}

//
// ================================
//        MESSAGE GENERATION
// ================================
//
/*
std::vector<Message> Drone::generateMessages(double simTime) {

    std::vector<Message> msgs;
    if (status == DroneStatus::DOWN) return msgs;

    // Omission check (node-level)
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    bool omit = (uni(GLOBAL_RNG) < config.omissionProb);

    // Jitter on the timestamp
    double sendTime = simTime;
    if (config.sendJitter > 0.0) {
        std::uniform_real_distribution<double> jitter(
            -config.sendJitter, config.sendJitter);
        sendTime += jitter(GLOBAL_RNG);
    }

    if (!omit) {
        // Heartbeats
        auto hb = membership.generateHeartbeatMessages(sendTime);
        msgs.insert(msgs.end(), hb.begin(), hb.end());

        // Reconfig messages
        auto rc = membership.consumePendingReconfigs();
        msgs.insert(msgs.end(), rc.begin(), rc.end());
    }

    return msgs;
}*/
std::vector<Message> Drone::generateMessages(double time) {
    if (status == DroneStatus::DOWN)
        return {};

    std::vector<Message> out;

    // Heartbeats scheduled by MembershipState::tick()
    auto hb = membership.consumePendingHeartbeats();
    out.insert(out.end(), hb.begin(), hb.end());

    // Reconfigs (due to failure detection)
    auto rc = membership.consumePendingReconfigs();
    out.insert(out.end(), rc.begin(), rc.end());

    for (const auto& msg : rc) {
        std::cout << "[SEND] Drone " << id << " sending ";
        if (msg.type == MessageType::RECONFIG_INIT) std::cout << "INIT";
        else if (msg.type == MessageType::RECONFIG_ACK) std::cout << "ACK";
        else if (msg.type == MessageType::RECONFIG_COMMIT) std::cout << "COMMIT";
        std::cout << "\n";
    }

    return out;
}


//
// ================================
//        MESSAGE RECEPTION
// ================================
//
void Drone::receiveMessages(const std::vector<Message>& msgs) {
    if (status == DroneStatus::DOWN) return;

    for (const auto& m : msgs) {
        std::string type;
        if (m.type == MessageType::HEARTBEAT) type = "HEARTBEAT";
        else if (m.type == MessageType::RECONFIG_INIT) type = "INIT";
        else if (m.type == MessageType::RECONFIG_ACK) type = "ACK";
        else if (m.type == MessageType::RECONFIG_COMMIT) type = "COMMIT";

        std::cout << "[RECEIVE] Drone " << id << " received " << type
            << " from " << m.senderId << " at simTime (approx)\n";
        if (m.type == MessageType::HEARTBEAT) {
            membership.receiveHeartbeat(m.senderId, m.timestamp, m.pos);
        }
        else if (m.type == MessageType::BROADCAST) {
            membership.installReconfig(m.payload, m.timestamp, position);
        } 
        else if (m.type == MessageType::RECONFIG_INIT) {
            membership.processInit(m, m.timestamp, position);
        }
        else if (m.type == MessageType::RECONFIG_ACK) {
            membership.processAck(m, m.timestamp);
        }
        else if (m.type == MessageType::RECONFIG_COMMIT) {
            membership.processCommit(m, m.timestamp, position);
        }

    }
}

//
// ================================
//          CONTROL + MOTION
// ================================
//
void Drone::applyControl(double /*dt*/) {
    // TODO: motion control later
}

void Drone::integrate(double dt) {
    position += velocity * (float)dt;
}
