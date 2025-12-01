#include "visualization/PolyscopeRenderer.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/curve_network.h"

static std::vector<PacketVis> activePackets;

void PolyscopeRenderer::drawPackets(double currentTime) {
    for (auto it = activePackets.begin(); it != activePackets.end();) {
        double alpha = (currentTime - it->createdAt) /
            (it->deliverAt - it->createdAt);

        if (alpha >= 1.0) {
            it = activePackets.erase(it); // remove delivered packet
            continue;
        }

        glm::vec3 pos = glm::mix(it->src, it->dst, alpha);
        glm::vec3 dir = glm::normalize(it->dst - it->src);

        // draw arrow from pos in direction of travel
        float scale = (it->kind == PacketKind::Broadcast) ? 0.08f : 0.03f;
        std::vector<glm::vec3> pts{ pos, pos + scale * dir };
        std::vector<std::array<size_t, 2>> edges{ {0,1} };

        auto net = polyscope::registerCurveNetwork("pkt_" + std::to_string(std::rand()), pts, edges);
        net->setRadius(scale * 0.2);
        net->setColor(it->kind == PacketKind::Broadcast ? glm::vec3(0.1, 1.0, 1.0)
            : glm::vec3(0.8, 0.2, 1.0));

        ++it;
    }
}
