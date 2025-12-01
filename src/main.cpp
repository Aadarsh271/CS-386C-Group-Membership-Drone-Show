#include <iostream>
#include <vector>
#include <cmath>
#include "core/Simulation.h"
#include <visualization/PacketVis.h>

// Polyscope
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"

// For specifying per-point quantities (color/values)
#include "polyscope/quantity.h"


static Simulation sim(0.02);
static polyscope::PointCloud* cloud = nullptr;
static std::vector<PacketVis> activePackets;


glm::vec3 stateColor(DroneStatus st, bool reconfigInProgress) {
    switch (st) {
    case DroneStatus::ALIVE: return reconfigInProgress ? glm::vec3(0.0, 1.0, 1.0) : glm::vec3(0.0, 1.0, 0.2);
    case DroneStatus::SUSPECT: return glm::vec3(1.0, 0.9, 0.0);
    case DroneStatus::DEAD: return glm::vec3(1.0, 0.1, 0.1);
    }
}

void updateVisualization() {
    sim.step();

    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> colors;

    for (auto& d : sim.getDrones()) {
        positions.push_back(d.getPosition());

        DroneStatus st = d.getStatus();
        if (st == DroneStatus::ALIVE) colors.push_back({ 0.0f, 1.0f, 0.2f });
        else if (st == DroneStatus::SUSPECT) colors.push_back({ 1.0f, 0.8f, 0.0f });
        else colors.push_back({ 1.0f, 0.0f, 0.0f });
    }

    cloud->updatePointPositions(positions);
    cloud->addColorQuantity("statusColor", colors);

}

int main() {
    std::cout << "Drone Show Simulation with Network / Core... \n";

    // Initialize drones
    for (int i = 0; i < 20; i++) {
        sim.addDrone(Drone(i, { float(i) * 0.1f, 0.0f, 0.0f }));
    }


    std::vector<glm::vec3> positions;
    positions.reserve(20);

    for (auto& d : sim.getDrones())
        positions.push_back(d.getPosition());

    // Polyscope setup
    polyscope::init();
    cloud = polyscope::registerPointCloud("drones", positions);



    polyscope::state::userCallback = []() {
        static bool killed = false;
        if (!killed && ImGui::Button("Kill Drone 0")) {
            sim.killDrone(0);
            killed = true;
        }
        updateVisualization();
        };

    polyscope::show();
    return 0;
}
