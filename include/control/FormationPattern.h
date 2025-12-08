#pragma once
#include <vector>
#include <string>
#include <glm/glm.hpp>
#include "membership/Types.h" // Vec3

// Represents a 2D closed curve in the XZ plane (y = 0),
// sampled as a polyline with arc-length parameterization.
class FormationPattern {
public:
    std::string name;
    std::vector<Vec3> points;          // polyline samples (x, 0, z)
    std::vector<float> cumulativeLen;  // prefix sums along polyline
    float totalLength = 0.0f;
    bool closed = true;

    FormationPattern() = default;
    FormationPattern(const std::string& n,
        const std::vector<Vec3>& pts,
        bool isClosed = true);

    bool empty() const { return points.empty(); }

    // Recompute cumulativeLen / totalLength after modifying points.
    void recomputeArcLength();

    // Sample along arc-length s in [0, totalLength).
    Vec3 sampleAtArcLength(float s) const;

    // Sample with normalized parameter u in [0,1).
    Vec3 sampleNormalized(float u) const;

    // --- Some built-in procedural shapes ---

    // Circle in XZ plane, centered at origin.
    static FormationPattern makeCircle(const std::string& name,
        float radius,
        int samples);

    // Simple 5-pointed star (like Texas star) in XZ plane.
    static FormationPattern makeStar(const std::string& name,
        float outerRadius,
        float innerRadius,
        int vertices,        // usually 5
        int samplesPerEdge);

    FormationPattern makeTriangle(const std::string& name,
        float radius,
        int samplesPerEdge);

    FormationPattern makeSquare(const std::string& name,
        float sideLength,
        int samplesPerEdge);

    // From arbitrary polyline; does NOT recompute on construction
    // unless you call recomputeArcLength().
    static FormationPattern fromPolyline(const std::string& name,
        const std::vector<Vec3>& pts,
        bool isClosed = true);

    // Stub for future PNG-based outline loading.
    // You can implement this using stb_image + simple contour tracing.
    static FormationPattern loadFromImageOutline(const std::string& pngFile,
        int targetSamples);
};
