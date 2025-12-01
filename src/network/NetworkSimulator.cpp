#include "network/NetworkSimulator.h"
#include <random>

static std::default_random_engine rng(std::random_device{}());
static std::uniform_real_distribution<double> lossDist(0.0, 1.0);

NetworkSimulator::NetworkSimulator(double baseLatency, double lossRate)
    : baseLatency(baseLatency), lossRate(lossRate) {
}

void NetworkSimulator::send(const Message& message, double currentTime) {
    if (lossDist(rng) < lossRate) return;

    double latency = baseLatency;

    if (message.receiverId == -1) {
        // Broadcast: deliver to all drones (network-layer doesn’t know drone count
        // so upper layer will map to recipients)
        broadcastQueue.push_back({ currentTime + latency, message });
    }
    else {
        queue.push(QueuedMsg{ currentTime + latency, message });
    }

    if (visualizationEnabled) {
        PacketVis p;
        p.kind = (message.type == MessageType::HEARTBEAT)
            ? PacketKind::Heartbeat
            : PacketKind::Broadcast;

        p.src = getDronePos(message.senderId);
        p.dst = getDronePos(message.receiverId);
        p.createdAt = currentTime;
        p.deliverAt = currentTime + latency;

        activePackets.push_back(p);
    }
}


std::vector<Message> NetworkSimulator::receive(double currentTime) {
    std::vector<Message> msgs;
    while (!queue.empty() && queue.top().deliverAt <= currentTime) {
        msgs.push_back(queue.top().msg);
        queue.pop();
    }
    return msgs;
}
