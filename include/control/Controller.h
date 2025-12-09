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
    // Uses acceleration-limited smooth motion (not instantaneous velocity changes).
    Vec3 computeVelocityForDrone(int droneId,
        const Vec3& currentPos,
        const Vec3& currentVel,
        double dt);

    // For visualization: bright �drone show� colors by logical index.
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

    // Orbit animation: drones move CW/CCW along formation when holding
    double lastUpdateTime = 0.0;
    float orbitOffset = 0.0f;                 // accumulated offset in normalized arc-length [0,1)
    float orbitSpeed = 0.03f;                 // orbit speed (normalized arc-length per second)

    void rebuildAssignments(const FormationPattern& pattern,
        std::vector<Vec3>& outTargets,
        float arcOffset = 0.0f);              // offset for orbit animation

    glm::vec3 hsvToRgb(float h, float s, float v) const;
};
