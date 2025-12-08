#pragma once

#include <vector>
#include "core/Drone.h"
#include "config/SimulationConfig.h"
#include "network/NetworkSimulator.h"

#include "control/Controller.h"
//class Controller;                // forward declaration


class Simulation {
public:
    Simulation(const SimulationConfig& config);

    void reset(const SimulationConfig& config);

    void addDrone(const Drone& drone);
    void killDrone(int id);
    void step();

     std::vector<Drone>& getDrones();
    double getTime() const;

    Vec3 getDronePosition(int id);
	NetworkSimulator& getNetwork() { return network; }

	SimulationConfig& getConfig() { return cfg; }

    // Return {isConsistent, referenceGroupId, referenceMemberSetSize}
    std::tuple<bool, int, int> checkConsistency() const;


    Controller* getController() { return controller.get(); }


private:
    SimulationConfig cfg;
    double currentTime = 0.0;

    std::vector<Drone> drones;
    NetworkSimulator network;
    std::unique_ptr<Controller> controller; 
};
#pragma once
