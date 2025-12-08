#include "core/Simulation.h"
#include "control/Controller.h"

Simulation::Simulation(const SimulationConfig& config)
    : cfg(config), network(config)
{
    // Create drones
    float radius = cfg.numDrones * cfg.droneSpacing / (2.0f * 3.14159f);
    float angleStep = 2.0f * 3.14159f / cfg.numDrones;

    for (int i = 0; i < cfg.numDrones; ++i) {
        float angle = i * angleStep;

        glm::vec3 pos{
            radius * std::cos(angle),
            0.0f,
            radius * std::sin(angle)
        };

        drones.emplace_back(i, pos, cfg);
    }

    // >>> ADD THIS BLOCK <<<

    /*int N = static_cast<int>(drones.size());
    for (int i = 0; i < N; ++i) {
        int left = (i == 0 ? N - 1 : i - 1);
        int right = (i == N - 1 ? 0 : i + 1);
        drones[i].setNeighbors(left, right);
    }*/

    // Give each MembershipState a pointer to the global drone list
    
    for (auto& d : drones) {
        d.setMembershipDroneList(&drones);
    }


    network.setDistanceFunction(
        [this](int senderId, int receiverId) -> double {
            if (receiverId == -1) {
                return 0.0;
            }
            const Vec3& a = drones[senderId].getPosition();
            const Vec3& b = drones[receiverId].getPosition();
            Vec3 diff = a - b;
            return glm::length(diff);
        }
    );

    controller = std::make_unique<Controller>(*this);
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

    currentTime += cfg.timeStep;

    // 1. Each drone updates logic (heartbeats, failure detect, etc)
    for (auto& d : drones) {
        d.update(currentTime);
    }

    // 2. Each drone generates outgoing messages
    for (auto& d : drones) {
        auto msgs = d.generateMessages(currentTime);

        for (auto& m : msgs) {
            // m.receiverId == -1 => broadcast (handled inside network)
            network.send(m, currentTime);
        }
    }

    // 3. Deliver messages whose time has come
    auto delivered = network.receive(currentTime);

    // 4. Fan-out broadcast messages (receiverId == -1)
    for (auto& m : delivered) {
        if (m.receiverId == -1) {
            for (auto& d : drones) {
                //if (d.getId() == m.senderId) continue;

                Message copy = m;
                copy.receiverId = d.getId();
                d.receiveMessages({ copy });
            }
        }
        else {
            // unicast deliver
            drones[m.receiverId].receiveMessages({ m });
        }
    }

    // 3. NEW: formation controller update.
    if (controller) {
        controller->update(currentTime);
    }

    // 4. physical motion
    for (auto& d : drones) {
        if (controller && d.getStatus() == DroneStatus::UP) {
            Vec3 vel = controller->computeVelocityForDrone(
                d.getId(), d.getPosition(), cfg.timeStep);
            d.setVelocity(vel);
        }
        else {
            d.setVelocity(Vec3(0.0f));
        }

        d.integrate(cfg.timeStep);
    }
}


 std::vector<Drone>& Simulation::getDrones() {
    return drones;
}

double Simulation::getTime() const {
    return currentTime;
}


std::tuple<bool, int, int> Simulation::checkConsistency() const {

    // 1. Find reference drone (UP + MEMBER)
    int refGroup = -1;
    std::vector<int> refMembers;

    bool found = false;

    for (auto d : drones) {
        if (d.getStatus() != DroneStatus::UP) continue;

        const auto mem = d.getMembership();

        // Must be MEMBER in its own view
        if (!mem.isMemberAlive(d.getId())) continue;

        refGroup = mem.getGroupId();
        refMembers = mem.getActiveMembers();
        found = true;
        break;
    }

    if (!found) {
        return { true, -1, 0 };  // trivially consistent
    }

    // 2. Check each drone
    std::unordered_set<int> refSet(refMembers.begin(), refMembers.end());

    for (auto d : drones) {
        if (d.getStatus() != DroneStatus::UP) continue;

        const auto mem = d.getMembership();

        if (!mem.isMemberAlive(d.getId()))
            continue;

        // Group ID must match
        if (mem.getGroupId() != refGroup)
            return { false, refGroup, (int)refSet.size() };

        // Membership sets must match exactly
        auto cur = mem.getActiveMembers();
        std::unordered_set<int> curSet(cur.begin(), cur.end());

        if (curSet != refSet)
            return { false, refGroup, (int)refSet.size() };
    }

    return { true, refGroup, (int)refSet.size() };
}
