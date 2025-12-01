#pragma once
#include <queue>
#include <vector>
#include "Message.h"
#include "visualization/PolyscopeRenderer.h"

struct QueuedMsg {
    double deliverAt;
    Message msg;
    bool operator<(const QueuedMsg& other) const {
        return deliverAt > other.deliverAt; // min-heap
    }
};

class NetworkSimulator {
public:
    NetworkSimulator(double baseLatency = 0.05, double lossRate = 0.0);

    void send(const Message& message, double currentTime);
    std::vector<Message> receive(double currentTime);

private:
    double baseLatency;
    double lossRate;
    std::priority_queue<QueuedMsg> queue;
	std::vector<QueuedMsg> broadcastQueue;
};
#pragma once
