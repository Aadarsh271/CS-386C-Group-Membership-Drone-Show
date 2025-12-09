#pragma once

#include <iostream>
#include <string>

// =============================================================================
// Simple Logging System for Drone Swarm Simulator
//
// Provides toggleable console output for debugging the membership protocol.
// Because sometimes you want to see every heartbeat, and sometimes you just
// want to watch the pretty drones dance.
// =============================================================================

namespace Log {

    // Global logging enable flag - toggled via UI
    inline bool enabled = false;

    // Log categories for fine-grained control
    inline bool showHeartbeats = true;
    inline bool showReconfig = true;
    inline bool showFailures = true;
    inline bool showMessages = true;

    // Core logging function
    template<typename... Args>
    inline void print(Args&&... args) {
        if (!enabled) return;
        (std::cout << ... << std::forward<Args>(args));
    }

    // Category-specific logging helpers
    template<typename... Args>
    inline void heartbeat(Args&&... args) {
        if (!enabled || !showHeartbeats) return;
        (std::cout << ... << std::forward<Args>(args));
    }

    template<typename... Args>
    inline void reconfig(Args&&... args) {
        if (!enabled || !showReconfig) return;
        (std::cout << ... << std::forward<Args>(args));
    }

    template<typename... Args>
    inline void failure(Args&&... args) {
        if (!enabled || !showFailures) return;
        (std::cout << ... << std::forward<Args>(args));
    }

    template<typename... Args>
    inline void message(Args&&... args) {
        if (!enabled || !showMessages) return;
        (std::cout << ... << std::forward<Args>(args));
    }

} // namespace Log
