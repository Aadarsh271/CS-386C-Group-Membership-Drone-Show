#include "core/Simulation.h"

Simulation::Simulation(double dt)
    : dt(dt), network(0.05, 0.0) {
}

void Simulation::addDrone(const Drone& drone) {
    drones.push_back(drone);
}

void Simulation::killDrone(int id) {
    if (id >= 0 && id < drones.size()) {
        drones[id].forceDead();
    }
}

Vec3 Simulation::getDronePosition(int id) {
    if (id >= 0 && id < drones.size()) {
        return drones[id].getPosition();
    }
	return Vec3{ 0,0,0 };
}


void Simulation::step() {
    currentTime += dt;

    // Each drone generates outgoing messages
    for (auto& d : drones) {
        auto msgs = d.generateMessages(currentTime);
        for (auto& m : msgs)
            network.send(m, currentTime);
    }

    // Deliver to intended recipients
    auto dmsgs = network.receive(currentTime);

    for (auto& msg : dmsgs) {
        if (msg.receiverId == -1) {
            // deliver to all drones
            for (auto& d : drones)
                d.receiveMessages({ msg });
        }
        else if (msg.receiverId >= 0 && msg.receiverId < (int)drones.size()) {
            drones[msg.receiverId].receiveMessages({ msg });
        }
    }


    // Drone logic and movement
    for (auto& d : drones) {
        d.tick(currentTime);
        d.applyControl(dt);
        d.integrate(dt);
    }
}

const std::vector<Drone>& Simulation::getDrones() const {
    return drones;
}

double Simulation::getTime() const {
    return currentTime;
}
