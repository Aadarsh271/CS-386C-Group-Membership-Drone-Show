#pragma once
#include "membership/Types.h"
#include "network/Message.h"
#include "membership/MembershipState.h"
#include "config/SimulationConfig.h"
#include <vector>

// Forward declaration
struct SimulationConfig;

class Drone {
public:
    Drone(int id, const Vec3& startPos, const SimulationConfig& cfg);

    void setMembershipDroneList(std::vector<Drone>* dronesPtr) {
        membership.setDroneList(dronesPtr);
    }

    int getId() const;
    const Vec3& getPosition() const;
    DroneStatus getStatus() const;

    // Called once per simulation tick
    void update(double simTime);

    // Message handling
    std::vector<Message> generateMessages(double simTime);
    void receiveMessages(const std::vector<Message>& msgs);

    // Movement (future)
    void applyControl(double dt);
    void integrate(double dt);

    void forceDead() {
        internalState = InternalFaultState::PERMANENTLY_DEAD;
        status = DroneStatus::DOWN;
        membership.forceDead();
    }

    MembershipState& getMembership() {
        return membership;
    }

    void setNeighbors(int leftId, int rightId) {
        membership.setNeighbors(leftId, rightId);
    }

    void setPosition(const Vec3& pos) {
        position = pos;
	}

    const Vec3& getVelocity() const { return velocity; }
    void setVelocity(const Vec3& v) { velocity = v; }

    const SimulationConfig& config;   // pointer to global config

private:
    // ================
    //  Drone Identity
    // ================
    int id;
    Vec3 position;
    Vec3 velocity{ 0,0,0 };
    DroneStatus status = DroneStatus::UP;


    // =======================
    //  Group Membership Core
    // =======================
    MembershipState membership;

    // ================
    //  Fault Model
    // ================
    enum class InternalFaultState {
        RUNNING,
        TEMPORARILY_CRASHED,
        PERMANENTLY_DEAD
    };

    InternalFaultState internalState = InternalFaultState::RUNNING;

    double crashStartTime = 0.0;
    double recoveryTime = 0.0;

    // Fault stepping
    void stepFaults(double simTime);
};
