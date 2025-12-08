#include "network/NetworkSimulator.h"
#include "core/RNG.h"

#include <random>

NetworkSimulator::NetworkSimulator(const SimulationConfig& cfg)
    : config(cfg),
    baseLatency(cfg.baseLatency),
    alphaLatency(cfg.alphaLatency),
    baseLossProb(cfg.baseLossProb),
    betaLoss(cfg.betaLoss),
    distanceLatencyFactor(cfg.distanceLatencyFactor),
    enableBursts(cfg.enableBursts),
    burstStartProb(cfg.burstStartProb),
    burstDropProb(cfg.burstDropProb),
    burstDuration(cfg.burstDuration),
    duplicationProb(cfg.duplicationProb)
{
}

// Helper to maybe update / apply burst-loss state
static bool shouldDropInBurst(bool& inBurst,
    double& burstEndTime,
    double currentTime,
    bool enableBursts,
    double burstStartProb,
    double burstDropProb,
    double burstDuration)
{
    if (!enableBursts) return false;

    std::uniform_real_distribution<double> uni(0.0, 1.0);

    // End burst if time is up
    if (inBurst && currentTime >= burstEndTime) {
        inBurst = false;
    }

    // Possibly start a new burst on this send
    if (!inBurst && uni(GLOBAL_RNG) < burstStartProb) {
        inBurst = true;
        burstEndTime = currentTime + burstDuration;
    }

    // If we are in a burst, drop with burstDropProb
    if (inBurst && uni(GLOBAL_RNG) < burstDropProb) {
        return true;
    }

    return false;
}

void NetworkSimulator::send(const Message& message, double currentTime) {

    std::uniform_real_distribution<double> uni(0.0, 1.0);

    // Burst-loss first (can kill entire runs of messages)
    if (shouldDropInBurst(inBurst, burstEndTime, currentTime,
        enableBursts, burstStartProb, burstDropProb, burstDuration)) {
        return;
    }

    // Current "load" in the network
    double load = static_cast<double>(queue.size() + broadcastQueue.size());

    // Base latency + overload growth
    double latency = baseLatency * (1.0 + alphaLatency * load);

    // Distance-based latency (if configured and distanceFunc provided)
    if (distanceLatencyFactor > 0.0 && distanceFunc) {
        double d = distanceFunc(message.senderId, message.receiverId);
        latency += distanceLatencyFactor * d;
    }

    // Base loss + overload loss
    double lossProb = baseLossProb + betaLoss * load;
    if (uni(GLOBAL_RNG) < lossProb) {
        return; // lost in the network
    }

    // Delivery time
    double deliverAt = currentTime + latency;

    // Visualization callback (packet-level)
    if (visCallback) {
        visCallback(message, currentTime, deliverAt);
    }

    // Schedule delivery
    if (message.receiverId == -1) {
        // Broadcast handled at simulation layer; we just carry the packet
        broadcastQueue.push_back(QueuedMsg{ deliverAt, message });
    }
    else {
        queue.push(QueuedMsg{ deliverAt, message });
    }

    // Possible duplication
    if (duplicationProb > 0.0 && uni(GLOBAL_RNG) < duplicationProb) {
        double dupDeliverAt = deliverAt; // could add tiny jitter if desired
        if (message.receiverId == -1) {
            broadcastQueue.push_back(QueuedMsg{ dupDeliverAt, message });
        }
        else {
            queue.push(QueuedMsg{ dupDeliverAt, message });
        }
    }
}

std::vector<Message> NetworkSimulator::receive(double currentTime) {
    std::vector<Message> msgs;

    // Unicast deliveries
    while (!queue.empty() && queue.top().deliverAt <= currentTime) {
        msgs.push_back(queue.top().msg);
        queue.pop();
    }

    // Broadcast deliveries: move ready ones, keep future ones
    std::vector<QueuedMsg> remaining;
    remaining.reserve(broadcastQueue.size());

    for (auto& qm : broadcastQueue) {
        if (qm.deliverAt <= currentTime) {
            msgs.push_back(qm.msg);
        }
        else {
            remaining.push_back(qm);
        }
    }

    broadcastQueue.swap(remaining);

    return msgs;
}
