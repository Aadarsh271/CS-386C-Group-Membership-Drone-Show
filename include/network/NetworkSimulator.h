#pragma once
#include <queue>
#include <vector>
#include <functional>

#include "Message.h"
#include <config/SimulationConfig.h>

struct QueuedMsg {
    double deliverAt;
    Message msg;

    // For std::priority_queue: smallest deliverAt should come out first
    bool operator<(const QueuedMsg& other) const {
        return deliverAt > other.deliverAt; // min-heap behavior
    }
};

class NetworkSimulator {
public:
    explicit NetworkSimulator(const SimulationConfig& cfg);

    // Schedule a message for delivery (or drop it due to faults)
    void send(const Message& message, double currentTime);

    // Deliver all messages whose deliverAt <= currentTime
    std::vector<Message> receive(double currentTime);

    // Optional visualization callback: (msg, sendTime, deliverTime)
    void setVisCallback(std::function<void(const Message&, double, double)> callback) {
        visCallback = callback;
    }

    // Optional distance callback: returns a "distance" between nodes
    // Used for distance-based latency modeling.
    void setDistanceFunction(std::function<double(int, int)> callback) {
        distanceFunc = callback;
    }

private:
    // Config reference (single source of truth)
    const SimulationConfig& config;

    // Cached network fault parameters (copied from config at construction)
    double baseLatency;
    double alphaLatency;
    double baseLossProb;
    double betaLoss;
    double distanceLatencyFactor;

    bool enableBursts;
    double burstStartProb;
    double burstDropProb;
    double burstDuration;

    double duplicationProb;

    // Burst-loss state
    bool inBurst = false;
    double burstEndTime = 0.0;

    // Queues
    std::priority_queue<QueuedMsg> queue;   // unicast
    std::vector<QueuedMsg> broadcastQueue;  // broadcast packets

    // Callbacks
    std::function<void(const Message&, double, double)> visCallback;
    std::function<double(int, int)> distanceFunc; // (senderId, receiverId)
};
