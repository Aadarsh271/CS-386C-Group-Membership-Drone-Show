#include "control/FormationPattern.h"
#include <iostream>

int main() {
    auto circ = FormationPattern::makeCircle("c", 10.0f, 512);
    auto star = FormationPattern::makeStar("s", 10.0f, 5.0f, 5, 64);

    const int samples = 100;
    double totalErr = 0;

    for (int i = 0; i < samples; i++) {
        float u = float(i) / samples;
        Vec3 p1 = circ.sampleNormalized(u);
        Vec3 p2 = star.sampleNormalized(u);

        Vec3 mid = 0.5f * p1 + 0.5f * p2;

        // path length preservation test
        totalErr += glm::length(p1 - p2);
    }

    std::cout << "Average distance = "
        << (totalErr / samples) << "\n";

    return 0;
}
