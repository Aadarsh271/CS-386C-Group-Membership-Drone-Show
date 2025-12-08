#pragma once
#include <vector>
#include "membership/Types.h"
#include "control/FormationPattern.h"

// Simple data for "where should drone i go on the shape?"
struct DroneAssignment {
    int  droneId;
    Vec3 targetPos;
    int  logicalIndex; // 0..N-1 rank among active drones
};

namespace AssignmentStrategy {

    // Given a sorted list of activeDroneIds and a shape,
    // assign each drone to an equally spaced slot along the shape.
    std::vector<DroneAssignment>
        assignDronesToPattern(const std::vector<int>& activeDroneIds,
            const FormationPattern& pattern);

}
