#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "control/FormationPattern.h"
#include <vector>
#include <cmath>
#include <algorithm>

// --- Utility ---------------------------------------------------------------

static float dist2(const Vec3& a, const Vec3& b) {
    Vec3 d = a - b;
    return d.x * d.x + d.y * d.y + d.z * d.z;
}

// Douglas�Peucker simplification
static void simplifyDP(const std::vector<Vec3>& pts,
    float eps,
    std::vector<Vec3>& out)
{
    if (pts.size() < 3) {
        out = pts;
        return;
    }

    std::vector<int> stack;
    std::vector<char> keep(pts.size(), 0);

    stack.push_back(0);
    stack.push_back(pts.size() - 1);
    keep[0] = keep[pts.size() - 1] = 1;

    while (stack.size() >= 2) {
        int end = stack.back(); stack.pop_back();
        int start = stack.back(); stack.pop_back();

        float maxDist = 0.0f;
        int index = -1;

        Vec3 A = pts[start];
        Vec3 B = pts[end];
        Vec3 AB = B - A;
        float AB_len2 = dist2(A, B);

        for (int i = start + 1; i < end; i++) {
            Vec3 AP = pts[i] - A;
            float t = (AB_len2 > 0) ? glm::dot(AP, AB) / AB_len2 : 0.0f;
            t = std::clamp(t, 0.0f, 1.0f);

            Vec3 proj = A + t * AB;
            float d2 = dist2(pts[i], proj);

            if (d2 > maxDist) {
                maxDist = d2;
                index = i;
            }
        }

        if (std::sqrt(maxDist) >= eps) {
            keep[index] = 1;
            stack.push_back(start);
            stack.push_back(index);
            stack.push_back(index);
            stack.push_back(end);
        }
    }

    out.clear();
    for (int i = 0; i < pts.size(); i++)
        if (keep[i]) out.push_back(pts[i]);
}

// --- Extract outline from PNG ----------------------------------------------

FormationPattern FormationPattern::loadFromImageOutline(const std::string& file,
    int targetSamples)
{
    int w, h, channels;
    unsigned char* data = stbi_load(file.c_str(), &w, &h, &channels, 1);

    if (!data) {
        return FormationPattern::makeCircle("fallback", 5.0f, targetSamples);
    }

    // Step 1 � Edge detection via simple gradient (Sobel or approximated)
    std::vector<Vec3> edgePts;

    auto idx = [&](int x, int y) { return y * w + x; };

    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            int v = data[idx(x, y)];

            int dx =
                data[idx(x + 1, y)] - data[idx(x - 1, y)];
            int dy =
                data[idx(x, y + 1)] - data[idx(x, y - 1)];

            float mag = std::sqrt(dx * dx + dy * dy);

            if (mag > 30) {
                // Convert pixel coordinates to centered XZ plane
                float X = (x - w / 2) / float(w / 2);
                float Z = (y - h / 2) / float(h / 2);

                edgePts.emplace_back(X, 0.0f, Z);
            }
        }
    }

    stbi_image_free(data);

    if (edgePts.size() < 50) {
        return FormationPattern::makeCircle("fallback", 5.0f, targetSamples);
    }

    // Step 2 � Sort points by angle around centroid
    Vec3 centroid(0);
    for (auto& p : edgePts) centroid += p;
    centroid /= float(edgePts.size());

    std::sort(edgePts.begin(), edgePts.end(),
        [&](const Vec3& a, const Vec3& b) {
            float aa = std::atan2(a.z - centroid.z, a.x - centroid.x);
            float bb = std::atan2(b.z - centroid.z, b.x - centroid.x);
            return aa < bb;
        });

    // Step 3 � Simplify outline
    std::vector<Vec3> simplified;
    simplifyDP(edgePts,               // input
        0.005f,                // epsilon
        simplified);           // output

    // Step 4 � Arc-length resample
    // Convert to FormationPattern and resample using its method
    FormationPattern tmp("image", simplified, true);
    std::vector<Vec3> finalPts;
    finalPts.reserve(targetSamples);

    for (int i = 0; i < targetSamples; i++) {
        float u = float(i) / targetSamples;
        finalPts.push_back(tmp.sampleNormalized(u));
    }

    return FormationPattern("image", finalPts, true);
}
