#include "control/FormationPattern.h"
#include "control/AssignmentStrategy.h"
#include "core/RNG.h"
#include <iostream>
#include <numeric>
#include <algorithm>
#include <cmath>


std::mt19937_64 GLOBAL_RNG;

void reseedRNG(uint64_t seed) {
    GLOBAL_RNG.seed(seed);
}


static float avgNeighborDistance(const std::vector<DroneAssignment>& a) {
    if (a.size() < 2) return 0.0f;
    float sum = 0.0f;
    int N = static_cast<int>(a.size());
    for (int i = 0; i < N; ++i) {
        const Vec3& p0 = a[i].targetPos;
        const Vec3& p1 = a[(i + 1) % N].targetPos;
        sum += glm::distance(p0, p1);
    }
    return sum / N;
}

int main() {
    // Simple circle pattern to test distribution.
    FormationPattern circle = FormationPattern::makeCircle("testCircle", 10.0f, 720);

    // 1) Start with 20 drones
    std::vector<int> ids(20);
    std::iota(ids.begin(), ids.end(), 0);

    auto a1 = AssignmentStrategy::assignDronesToPattern(ids, circle);
    float d1 = avgNeighborDistance(a1);

    std::cout << "N=20, avg neighbor distance=" << d1 << "\n";

    // 2) "Kill" some drones: remove them from the active list.
    ids.erase(std::remove(ids.begin(), ids.end(), 3), ids.end());
    ids.erase(std::remove(ids.begin(), ids.end(), 7), ids.end());

    auto a2 = AssignmentStrategy::assignDronesToPattern(ids, circle);
    float d2 = avgNeighborDistance(a2);

    std::cout << "N=" << ids.size()
        << ", avg neighbor distance=" << d2 << "\n";

    // Circumference ~ N * avgDist should stay nearly constant.
    float c1 = d1 * static_cast<float>(a1.size());
    float c2 = d2 * static_cast<float>(a2.size());

    std::cout << "c1=" << c1 << ", c2=" << c2 << "\n";

    if (std::fabs(c1 - c2) / c1 > 0.1f) {
        std::cerr << "ERROR: circumference changed too much; "
            << "assignment may not be uniform.\n";
        return 1;
    }

    std::cout << "Formation assignment test passed.\n";
    return 0;
}
