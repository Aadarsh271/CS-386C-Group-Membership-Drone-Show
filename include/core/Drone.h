#pragma once
#include "core/Types.h"
#include "network/Message.h"
#include "membership/MembershipState.h"
#include <vector>

class Drone {
public:
    Drone(int id, const Vec3& startPos);

    int getId() const;
    const Vec3& getPosition() const;
    DroneStatus getStatus() const;

    void tick(double time); // membership heartbeat etc (future)
    void applyControl(double dt); // movement (future)
    void integrate(double dt); // apply velocity

    // Message handling
    std::vector<Message> generateMessages(double time);
    void receiveMessages(const std::vector<Message>& msgs);

    void forceDead() { status = DroneStatus::DEAD; membership.forceDead(); }

private:
    int id;
    Vec3 position;
    Vec3 velocity{ 0,0,0 };
    DroneStatus status = DroneStatus::ALIVE;
    MembershipState membership{ id, 0.5 /*hb*/, 1.0 /*timeout*/ };
};
#pragma once
