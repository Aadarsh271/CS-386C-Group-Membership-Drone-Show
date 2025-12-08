#include "control/FormationPattern.h"
#include <cmath>
#include <stdexcept>

FormationPattern::FormationPattern(const std::string& n,
    const std::vector<Vec3>& pts,
    bool isClosed)
    : name(n), points(pts), closed(isClosed) {
    recomputeArcLength();
}

void FormationPattern::recomputeArcLength() {
    cumulativeLen.clear();
    totalLength = 0.0f;

    if (points.size() < 2) {
        cumulativeLen.push_back(0.0f);
        return;
    }

    cumulativeLen.reserve(points.size());
    cumulativeLen.push_back(0.0f);

    const int n = static_cast<int>(points.size());
    const int last = closed ? n : n - 1;

    float acc = 0.0f;
    for (int i = 0; i < last - 1; ++i) {
        float d = glm::distance(points[i], points[i + 1]);
        acc += d;
        cumulativeLen.push_back(acc);
    }

    if (closed) {
        float d = glm::distance(points[n - 1], points[0]);
        acc += d;
    }

    totalLength = acc;
    if (totalLength <= 0.0f) {
        totalLength = 1.0f; // avoid division by zero
    }
}

Vec3 FormationPattern::sampleAtArcLength(float s) const {
    if (points.empty()) return Vec3(0.0f);
    if (points.size() == 1) return points[0];

    if (closed) {
        s = std::fmod(s, totalLength);
        if (s < 0.0f) s += totalLength;
    }
    else {
        if (s <= 0.0f) return points.front();
        if (s >= totalLength) return points.back();
    }

    const int n = static_cast<int>(points.size());
    const int last = closed ? n : n - 1;

    // Find segment idx such that cumulativeLen[idx] <= s <= cumulativeLen[idx+1]
    int idx = 0;
    for (int i = 0; i < static_cast<int>(cumulativeLen.size()) - 1; ++i) {
        if (s >= cumulativeLen[i] && s <= cumulativeLen[i + 1]) {
            idx = i;
            break;
        }
    }

    float segStart = cumulativeLen[idx];
    float segLen = cumulativeLen[idx + 1] - segStart;
    float t = (segLen > 0.0f) ? (s - segStart) / segLen : 0.0f;

    int i0 = idx;
    int i1 = idx + 1;
    if (i1 >= last) {
        // final segment for closed polyline is [n-1 -> 0]
        i0 = n - 1;
        i1 = 0;
    }

    return (1.0f - t) * points[i0] + t * points[i1];
}

Vec3 FormationPattern::sampleNormalized(float u) const {
    if (points.empty()) return Vec3(0.0f);
    if (u < 0.0f) u = 0.0f;
    if (u > 1.0f) u = 1.0f;
    float s = u * totalLength;
    return sampleAtArcLength(s);
}

FormationPattern FormationPattern::makeCircle(const std::string& name,
    float radius,
    int samples) {
    std::vector<Vec3> pts;
    pts.reserve(samples);
    const float TWO_PI = 6.283185307179586f;

    for (int i = 0; i < samples; ++i) {
        float a = TWO_PI * (static_cast<float>(i) / samples);
        float x = radius * std::cos(a);
        float z = radius * std::sin(a);
        pts.emplace_back(x, 0.0f, z);
    }

    return FormationPattern(name, pts, true);
}

FormationPattern FormationPattern::makeStar(const std::string& name,
    float outerRadius,
    float innerRadius,
    int vertices,
    int samplesPerEdge) {
    if (vertices < 2) {
        throw std::runtime_error("FormationPattern::makeStar: vertices must be >= 2");
    }

    // Build the star's main vertices (alternating outer/inner)
    std::vector<Vec3> main;
    const float TWO_PI = 6.283185307179586f;

    for (int i = 0; i < vertices; ++i) {
        float baseAngle = TWO_PI * (static_cast<float>(i) / vertices);
        // outer
        main.emplace_back(outerRadius * std::cos(baseAngle),
            0.0f,
            outerRadius * std::sin(baseAngle));
        // inner
        float innerAngle = baseAngle + (TWO_PI / (2.0f * vertices));
        main.emplace_back(innerRadius * std::cos(innerAngle),
            0.0f,
            innerRadius * std::sin(innerAngle));
    }

    // Densify edges
    std::vector<Vec3> pts;
    const int m = static_cast<int>(main.size());
    for (int i = 0; i < m; ++i) {
        int j = (i + 1) % m;
        Vec3 a = main[i];
        Vec3 b = main[j];

        for (int k = 0; k < samplesPerEdge; ++k) {
            float t = static_cast<float>(k) / samplesPerEdge;
            pts.push_back((1.0f - t) * a + t * b);
        }
    }

    return FormationPattern(name, pts, true);
}

FormationPattern FormationPattern::fromPolyline(const std::string& name,
    const std::vector<Vec3>& pts,
    bool isClosed) {
    FormationPattern p;
    p.name = name;
    p.points = pts;
    p.closed = isClosed;
    p.recomputeArcLength();
    return p;
}

FormationPattern FormationPattern::loadFromImageOutline(const std::string& pngFile,
    int targetSamples) {
    // Stub: fill this in later with stb_image + contour tracing.
    // For now, just return a simple circle so callers still work.
    (void)pngFile;
    return makeCircle("placeholder_from_png", 3.0f, targetSamples);
}
