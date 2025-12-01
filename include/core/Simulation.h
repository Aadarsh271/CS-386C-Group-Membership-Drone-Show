#pragma once

#include <vector>
#include "core/Drone.h"
#include "network/NetworkSimulator.h"

class Simulation {
public:
    Simulation(double dt);

    void addDrone(const Drone& drone);
    void killDrone(int id);
    void step();

    const std::vector<Drone>& getDrones() const;
    double getTime() const;

    Vec3 getDronePosition(int id);


private:
    double dt;
    double currentTime = 0.0;

    std::vector<Drone> drones;
    NetworkSimulator network;
};
#pragma once
