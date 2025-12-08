#include "control/Controller.h"
#include "core/Simulation.h"
#include "core/Drone.h"

#include <algorithm>
#include <cmath>

Controller::Controller(Simulation& sim)
    : sim(sim)
{
    // Build some default patterns based on simulation scale.
    const auto& cfg = sim.getConfig();
    float radius = cfg.numDrones * cfg.droneSpacing / (2.0f * 3.14159265f);
    if (radius <= 0.0f) radius = 3.0f;

    // Circle roughly matching initial placement.
    FormationPattern circle = FormationPattern::makeCircle("circle", radius, 512);
    addPattern(circle);

    // Star slightly larger than the circle.
    FormationPattern star = FormationPattern::makeStar("star",
        radius * 1.2f,
        radius * 0.5f,
        5,
        32);
    addPattern(star);

    // Start on pattern 0 by default.
    setActivePattern(0);
}

int Controller::addPattern(const FormationPattern& pattern) {
    patterns.push_back(pattern);
    if (activePatternIndex < 0) {
        activePatternIndex = 0;
    }
    return static_cast<int>(patterns.size()) - 1;
}

void Controller::clearPatterns() {
    patterns.clear();
    activePatternIndex = -1;
    prevPatternIndex = -1;
    nextPatternIndex = -1;
    inTransition = false;
}

// Immediately jump to a pattern (no interpolation).
void Controller::setActivePattern(int idx) {
    if (idx < 0 || idx >= static_cast<int>(patterns.size())) return;

    activePatternIndex = idx;
    prevPatternIndex = idx;
    nextPatternIndex = idx;
    inTransition = false;
    currentBlendT = 1.0;
}

void Controller::startTransitionTo(int nextIdx,
    double startTime,
    double duration)
{
    if (nextIdx < 0 || nextIdx >= static_cast<int>(patterns.size())) return;
    if (activePatternIndex < 0) {
        setActivePattern(nextIdx);
        return;
    }

    prevPatternIndex = activePatternIndex;
    nextPatternIndex = nextIdx;
    transitionStartTime = startTime;
    transitionDuration = (duration > 0.0) ? duration : 1.0;
    inTransition = true;
    currentBlendT = 0.0;
}

// Rebuild targetsPrev/targetsNext and logical indices for *current* UP drones.
void Controller::update(double simTime) {
    auto& drones = sim.getDrones();

    // 1) Gather active (UP) drone ids.
    activeIds.clear();
    for (auto& d : drones) {
        if (d.getStatus() == DroneStatus::UP) {
            activeIds.push_back(d.getId());
        }
    }
    std::sort(activeIds.begin(), activeIds.end());

    // Rebuild idToRank.
    idToRank.clear();
    for (int i = 0; i < static_cast<int>(activeIds.size()); ++i) {
        idToRank[activeIds[i]] = i;
    }

    if (activeIds.empty() || patterns.empty()) {
        targetsPrev.clear();
        targetsNext.clear();
        return;
    }

    // If no active pattern, default to 0.
    if (activePatternIndex < 0) {
        activePatternIndex = 0;
        prevPatternIndex = 0;
        nextPatternIndex = 0;
    }

    // 2) Determine blend parameter.
    if (inTransition) {
        double t = (simTime - transitionStartTime) / transitionDuration;
        if (t >= 1.0) {
            // Transition complete.
            inTransition = false;
            activePatternIndex = nextPatternIndex;
            currentBlendT = 1.0;
        }
        else if (t <= 0.0) {
            currentBlendT = 0.0;
        }
        else {
            currentBlendT = t;
        }
    }
    else {
        currentBlendT = 1.0;
        prevPatternIndex = activePatternIndex;
        nextPatternIndex = activePatternIndex;
    }

    // 3) Rebuild assignments for the patterns that matter this frame.
    targetsPrev.clear();
    targetsNext.clear();

    const FormationPattern& prevPat = patterns[prevPatternIndex];
    const FormationPattern& nextPat = patterns[nextPatternIndex];

    rebuildAssignments(prevPat, targetsPrev);
    if (nextPatternIndex == prevPatternIndex) {
        targetsNext = targetsPrev; // identical
    }
    else {
        rebuildAssignments(nextPat, targetsNext);
    }

    // defensiveness: ensure sizes match activeIds.size()
    if (targetsPrev.size() != activeIds.size()) {
        targetsPrev.resize(activeIds.size(), Vec3(0.0f));
    }
    if (targetsNext.size() != activeIds.size()) {
        targetsNext.resize(activeIds.size(), Vec3(0.0f));
    }
}

void Controller::rebuildAssignments(const FormationPattern& pattern,
    std::vector<Vec3>& outTargets)
{
    outTargets.clear();
    if (activeIds.empty() || pattern.empty()) return;

    // Use the same core logic as AssignmentStrategy to ensure
    // the test harness matches the runtime behavior.
    std::vector<DroneAssignment> assignments =
        AssignmentStrategy::assignDronesToPattern(activeIds, pattern);

    outTargets.reserve(assignments.size());
    for (const auto& a : assignments) {
        outTargets.push_back(a.targetPos);
    }
}

Vec3 Controller::computeVelocityForDrone(int droneId,
    const Vec3& currentPos,
    double dt) const
{
    if (dt <= 0.0 || activeIds.empty()) {
        return Vec3(0.0f);
    }

    auto it = idToRank.find(droneId);
    if (it == idToRank.end()) {
        return Vec3(0.0f);
    }

    int idx = it->second;
    if (idx < 0 || idx >= static_cast<int>(targetsNext.size())) {
        return Vec3(0.0f);
    }

    Vec3 pPrev = (idx < static_cast<int>(targetsPrev.size()))
        ? targetsPrev[idx]
        : targetsNext[idx];
    Vec3 pNext = targetsNext[idx];

    // Blend between previous and next target if inTransition.
    Vec3 target = glm::mix(pPrev, pNext, static_cast<float>(currentBlendT));

    Vec3 delta = target - currentPos;
    float dist = glm::length(delta);
    if (dist < 1e-4f) {
        return Vec3(0.0f);
    }

    // Simple speed-limited motion.
    const float maxSpeed = 1.0f; // units per second
    float maxStep = maxSpeed * static_cast<float>(dt);

    if (dist <= maxStep) {
        // close enough: go directly there this frame.
        return delta / static_cast<float>(dt);
    }
    else {
        Vec3 dir = delta / dist;
        return dir * maxSpeed;
    }
}

glm::vec3 Controller::hsvToRgb(float h, float s, float v) const {
    // h in [0,1), s,v in [0,1]
    float r, g, b;

    float i = std::floor(h * 6.0f);
    float f = h * 6.0f - i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - f * s);
    float t = v * (1.0f - (1.0f - f) * s);

    int ii = static_cast<int>(i) % 6;
    switch (ii) {
    case 0: r = v; g = t; b = p; break;
    case 1: r = q; g = v; b = p; break;
    case 2: r = p; g = v; b = t; break;
    case 3: r = p; g = q; b = v; break;
    case 4: r = t; g = p; b = v; break;
    case 5: r = v; g = p; b = q; break;
    default: r = g = b = v; break;
    }
    return glm::vec3(r, g, b);
}

glm::vec3 Controller::getColorForDrone(int droneId,
    DroneStatus status) const
{
    if (status == DroneStatus::DOWN) {
        // Internally down = black.
        return glm::vec3(0.0f);
    }

    auto it = idToRank.find(droneId);
    if (it == idToRank.end() || activeIds.empty()) {
        // Not taking part in the pattern -> dim gray.
        return glm::vec3(0.3f, 0.3f, 0.3f);
    }

    int idx = it->second;
    int N = static_cast<int>(activeIds.size());
    float u = (N > 1) ? static_cast<float>(idx) / (N - 1)
        : 0.0f;

    // Nice neon-ish rainbow.
    glm::vec3 rgb = hsvToRgb(u, 0.9f, 1.0f);
    return rgb;
}
