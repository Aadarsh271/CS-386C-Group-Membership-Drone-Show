#pragma once
#include <vector>
#include <cstdint>

enum class MessageType {
    HEARTBEAT,
    GOSSIP,
    BROADCAST,
    CONTROL
};

struct Message {
    int senderId;
    int receiverId;
    MessageType type;
    double timestamp;

    // Generic payload storage
    std::vector<uint8_t> payload;
};
#pragma once
