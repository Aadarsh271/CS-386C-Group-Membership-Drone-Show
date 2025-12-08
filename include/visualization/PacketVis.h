#pragma once
#include "membership/Types.h"

enum class PacketKind {
    Heartbeat,
    Broadcast,
    Control
};

struct PacketVis {
    PacketKind kind;
    glm::vec3 src;
    glm::vec3 dst;
    double createdAt;
    double deliverAt;
    bool dropped = false;
};
