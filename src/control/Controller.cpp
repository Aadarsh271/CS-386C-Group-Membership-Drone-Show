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

    // Star - iconic shape
    FormationPattern star = FormationPattern::makeStar("star",
        radius * 1.2f,
        radius * 0.5f,
        5,
        32);
    addPattern(star);

    // Square
    FormationPattern square = FormationPattern::makeSquare("square",
        radius * 1.6f,
        64);
    addPattern(square);

    // Triangle
    FormationPattern triangle = FormationPattern::makeTriangle("triangle",
        radius * 1.3f,
        64);
    addPattern(triangle);

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

    // Update orbit animation offset
    double dt = simTime - lastUpdateTime;
    lastUpdateTime = simTime;

    // Only orbit when not transitioning (holding a shape)
    if (!inTransition && dt > 0.0 && dt < 1.0) {
        orbitOffset += orbitSpeed * static_cast<float>(dt);
        // Keep in [0, 1)
        while (orbitOffset >= 1.0f) orbitOffset -= 1.0f;
        while (orbitOffset < 0.0f) orbitOffset += 1.0f;
    }

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

    // Apply orbit offset only when not transitioning
    float offset = inTransition ? 0.0f : orbitOffset;

    rebuildAssignments(prevPat, targetsPrev, offset);
    if (nextPatternIndex == prevPatternIndex) {
        targetsNext = targetsPrev; // identical
    }
    else {
        rebuildAssignments(nextPat, targetsNext, offset);
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
    std::vector<Vec3>& outTargets,
    float arcOffset)
{
    outTargets.clear();
    if (activeIds.empty() || pattern.empty()) return;

    int N = static_cast<int>(activeIds.size());
    outTargets.reserve(N);

    // Each drone gets a slot along the arc, with alternating CW/CCW direction
    // based on their rank (even = CW, odd = CCW)
    for (int i = 0; i < N; ++i) {
        // Base position: evenly spaced along the formation
        float baseU = static_cast<float>(i) / static_cast<float>(N);

        // Direction alternation: even drones go CW (+), odd go CCW (-)
        float direction = (i % 2 == 0) ? 1.0f : -1.0f;

        // Apply orbit offset with direction
        float u = baseU + arcOffset * direction;

        // Wrap to [0, 1)
        while (u >= 1.0f) u -= 1.0f;
        while (u < 0.0f) u += 1.0f;

        Vec3 pos = pattern.sampleNormalized(u);
        outTargets.push_back(pos);
    }
}

Vec3 Controller::computeVelocityForDrone(int droneId,
    const Vec3& currentPos,
    const Vec3& currentVel,
    double dt)
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

    // Smooth motion parameters
    const float maxSpeed = 2.5f;       // units per second
    const float maxAccel = 4.0f;       // units per second squared
    const float arrivalRadius = 0.05f; // slow down when this close

    if (dist < 1e-4f) {
        // At target - decelerate to stop
        float speed = glm::length(currentVel);
        if (speed < 0.01f) {
            return Vec3(0.0f);
        }
        // Decelerate
        Vec3 decel = -glm::normalize(currentVel) * maxAccel * static_cast<float>(dt);
        if (glm::length(decel) > speed) {
            return Vec3(0.0f);
        }
        return currentVel + decel;
    }

    Vec3 dir = delta / dist;

    // Desired velocity towards target
    // Use arrival behavior: slow down as we approach
    float desiredSpeed = maxSpeed;
    if (dist < arrivalRadius * 10.0f) {
        desiredSpeed = maxSpeed * (dist / (arrivalRadius * 10.0f));
        desiredSpeed = std::max(desiredSpeed, 0.1f);
    }

    Vec3 desiredVel = dir * desiredSpeed;

    // Apply acceleration limit
    Vec3 velDiff = desiredVel - currentVel;
    float diffMag = glm::length(velDiff);

    if (diffMag < 1e-4f) {
        return desiredVel;
    }

    float maxDeltaV = maxAccel * static_cast<float>(dt);
    if (diffMag <= maxDeltaV) {
        return desiredVel;
    }

    // Can't reach desired velocity this frame - accelerate towards it
    Vec3 newVel = currentVel + (velDiff / diffMag) * maxDeltaV;

    // Clamp to max speed
    float newSpeed = glm::length(newVel);
    if (newSpeed > maxSpeed) {
        newVel = (newVel / newSpeed) * maxSpeed;
    }

    return newVel;
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
