#include "control/AssignmentStrategy.h"
#include <algorithm>

namespace AssignmentStrategy {

    std::vector<DroneAssignment>
        assignDronesToPattern(const std::vector<int>& activeDroneIds,
            const FormationPattern& pattern)
    {
        std::vector<DroneAssignment> out;
        if (activeDroneIds.empty() || pattern.empty()) {
            return out;
        }

        // Make a copy so we can ensure deterministic ordering.
        std::vector<int> ids = activeDroneIds;
        std::sort(ids.begin(), ids.end());

        const int N = static_cast<int>(ids.size());
        out.reserve(N);

        for (int i = 0; i < N; ++i) {
            float u = (static_cast<float>(i) + 0.5f) / static_cast<float>(N); // center of slot
            Vec3 pos = pattern.sampleNormalized(u);
            out.push_back(DroneAssignment{ ids[i], pos, i });
        }

        return out;
    }

} // namespace AssignmentStrategy
