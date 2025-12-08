#pragma once
#include <vector>
#include <cstdint>
#include <glm/glm.hpp>

enum class MessageType {
    HEARTBEAT,
    GOSSIP,
    BROADCAST,
    CONTROL,

    RECONFIG_INIT,
    RECONFIG_ACK,
    RECONFIG_COMMIT
};

struct Message {
    int senderId;
    int receiverId;
    MessageType type;
    double timestamp;

    // Generic payload storage
    std::vector<uint8_t> payload;

    glm::vec3 pos;
    int32_t groupHint;

};
#pragma once
