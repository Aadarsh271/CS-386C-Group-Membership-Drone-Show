#pragma once
#include <vector>
#include <unordered_map>
#include <glm/glm.hpp>

#include "membership/Types.h"
#include "control/FormationPattern.h"
#include "control/AssignmentStrategy.h"

class Simulation;

// Simple P-controller target. (We only need the position for now.)
struct DroneTarget {
    Vec3 pos;
};

class Controller {
public:
    explicit Controller(Simulation& sim);

    // Pattern management
    int  addPattern(const FormationPattern& pattern);
    void clearPatterns();
    int  getPatternCount() const { return static_cast<int>(patterns.size()); }

    void setActivePattern(int idx);
    int  getActivePattern() const { return activePatternIndex; }

    // Start a morph from current pattern to nextIdx.
    void startTransitionTo(int nextIdx,
        double startTime,
        double duration);

    // Called from Simulation::step once per tick.
    void update(double simTime);

    // Compute velocity that nudges drone towards its current target.
    Vec3 computeVelocityForDrone(int droneId,
        const Vec3& currentPos,
        double dt) const;

    // For visualization: bright “drone show” colors by logical index.
    glm::vec3 getColorForDrone(int droneId,
        DroneStatus status) const;

private:
    Simulation& sim;

    std::vector<FormationPattern> patterns;

    int activePatternIndex = -1;
    int prevPatternIndex = -1;
    int nextPatternIndex = -1;

    bool   inTransition = false;
    double transitionStartTime = 0.0;
    double transitionDuration = 1.0;
    double currentBlendT = 1.0; // 0..1

    // Cached assignments for the *current* frame.
    std::vector<int> activeIds;                // sorted active drone ids
    std::unordered_map<int, int> idToRank;    // droneId -> logicalIndex
    std::vector<Vec3> targetsPrev;            // pattern(prevPatternIndex)
    std::vector<Vec3> targetsNext;            // pattern(next or active)

    void rebuildAssignments(const FormationPattern& pattern,
        std::vector<Vec3>& outTargets);

    glm::vec3 hsvToRgb(float h, float s, float v) const;
};
