# Drone Swarm Membership Protocol Simulator

A visualization and simulation tool for exploring distributed group membership protocols in drone swarms. Watch drones form mesmerizing patterns while their underlying protocol handles failures gracefully.

## Features

**Protocol Simulation**
- Neighborhood-surveillance failure detection (each drone monitors only 2 neighbors)
- 3-phase atomic broadcast reconfiguration (INIT → ACK → COMMIT)
- Deterministic closest-neighbor selection based on physical proximity
- Configurable timing parameters for protocol tuning

**Network Modeling**
- Configurable latency (base + load-dependent + distance-based)
- Packet loss with optional burst correlation
- Message duplication and jitter
- Real-time parameter adjustment via GUI

**Visualization Modes**
- **Drone Show Mode**: Black background, rainbow colors, automatic formation morphing
- **Membership Mode**: Full protocol visualization with neighbor links and packet traces

**Failure Injection**
- Kill individual drones or trigger cascade failures
- Random crash injection with configurable rates
- Message omission simulation for stress testing

---

## Build Instructions

### Requirements
- CMake ≥ 3.16
- C++20 compiler (MSVC, GCC, or Clang)
- Polyscope (fetched automatically via CMake FetchContent)
- GLM (fetched automatically)

### Build

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

### Run

```bash
./drone-show
```

On Windows:
```bash
.\Debug\drone-show.exe
```

---

## Using the Simulator

### Basic Navigation

The simulator uses Polyscope for 3D visualization. Here's how to navigate:

| Action | Control |
|--------|---------|
| **Rotate view** | Left-click + drag |
| **Pan (move around)** | Right-click + drag |
| **Zoom** | Scroll wheel |
| **Reset view** | Press `R` |

### Understanding the Two Modes

#### Drone Show Mode (Default)

When you first launch the simulator, you'll see the **Drone Show Mode**:
- **Black background** for dramatic effect
- **Rainbow-colored drones** that sparkle and pulse
- **Automatic formation morphing** - drones smoothly transition between shapes (circle → star → square → triangle)
- The protocol still runs underneath, but visualizations are hidden for aesthetic appeal

This mode is great for demonstrating what a coordinated drone swarm looks like in action.

#### Membership Mode

Click **"Membership Mode"** to see the protocol internals:
- **Gray background** for better visibility of protocol elements
- **Blue lines** connect each drone to its two monitored neighbors (the "neighborhood surveillance" topology)
- **Magenta/pink dots** are heartbeat packets traveling between neighbors
- **Cyan spikes** shooting upward are atomic broadcast messages (INIT, ACK, COMMIT)
- **Green drones** are UP (alive and participating)
- **Red drones** are DOWN (crashed/failed)

The status panel shows:
- **CONSISTENT** (green) - All drones agree on group membership
- **INCONSISTENT** (red) - Drones have different views (temporary during reconfiguration)
- **HEARTBEAT MISS** warning - Appears when a drone detects its neighbor has stopped responding

### Killing Drones and Watching Recovery

This is where it gets fun:

1. Switch to **Membership Mode** so you can see what's happening
2. Click **"Kill Random Drone"** in the Failure Injection section
3. Watch as:
   - The killed drone turns **red**
   - After a timeout, its neighbors detect the failure (you'll see a **HEARTBEAT MISS** warning)
   - One of the neighbors initiates reconfiguration (cyan broadcast spikes)
   - All surviving drones exchange messages
   - The **blue neighbor links** update to reflect the new topology
   - Status returns to **CONSISTENT**

Try the **"Cascade Failure"** button to kill a drone AND both its neighbors simultaneously - this is the worst-case scenario for the protocol and demonstrates its resilience.

### Scenario Buttons

| Button | What it does |
|--------|--------------|
| **Kill Random Drone** | Permanently kills one random UP drone |
| **Force Neighbor Reconfig** | Forces all drones to recalculate their closest neighbors |
| **Cascade Failure** | Kills a drone and both its neighbors (worst-case test) |
| **Kill Half** | Kills every other drone (stress test) |

### Configuring the Protocol

Expand the **Protocol Parameters** section to tune the membership protocol:

| Parameter | What it controls |
|-----------|------------------|
| **Heartbeat Interval** | How often drones send "I'm alive" messages (lower = faster detection, more traffic) |
| **Timeout Delta (small)** | Maximum expected message delay - used for failure detection |
| **Timeout Delta (large)** | Window for collecting ACKs during reconfiguration |
| **Reconfig Min Interval** | Minimum time between consecutive reconfigurations (prevents storms) |

### Network Simulation

Expand **Network Model** to simulate realistic network conditions:

| Parameter | Effect |
|-----------|--------|
| **Base Latency** | Minimum delay for all messages |
| **Alpha (Load Growth)** | Extra latency when network is busy |
| **Distance Factor** | Latency increases with physical distance between drones |
| **Base Loss Prob** | Probability any packet is dropped |
| **Enable Burst Loss** | Simulates correlated packet loss (like real WiFi interference) |

### Fault Injection

Expand **Fault Model** to enable random failures:

| Parameter | Effect |
|-----------|--------|
| **Enable Random Crashes** | Drones randomly fail during simulation |
| **Crash Rate** | Probability of crash per simulation tick |
| **Permanent Crash Prob** | Whether crashed drones stay dead or recover |
| **Omission Probability** | Drones randomly skip sending messages |
| **Send Jitter** | Random timing variation in message sending |

### Console Logging

Enable **Console Logging** to see protocol messages in your terminal:
- **Heartbeats** - Every heartbeat sent/received
- **Reconfig** - INIT, ACK, COMMIT messages and state changes
- **Failures** - When drones detect neighbor failures
- **Messages** - All message send/receive events

This is invaluable for understanding exactly what the protocol is doing.

### Formation Control

The **Formation Control** panel lets you control the drone show:

**Manual Mode:**
- Click pattern buttons to trigger smooth transitions
- Adjust morph duration for faster/slower transitions

**Auto Timeline Mode:**
- Drones automatically cycle through formations
- Click "Restart Show" to begin the sequence again

---

## Protocol Overview

### Neighborhood Surveillance

Traditional group membership protocols require every member to monitor every other member - O(n²) overhead. Our protocol uses **neighborhood surveillance**: each drone only monitors its **two closest neighbors** (left and right in terms of physical proximity).

When a neighbor stops sending heartbeats within the timeout window, the detecting drone initiates group reconfiguration.

### 3-Phase Atomic Broadcast

Reconfiguration uses a classic 3-phase protocol:

1. **INIT** - The detecting drone broadcasts "I want to reconfigure" to everyone
2. **ACK** - All participating drones respond with their current position
3. **COMMIT** - After collecting ACKs, the initiator broadcasts the final membership list

All drones that receive the COMMIT update their view simultaneously, ensuring consistency.

### Tie-Breaking

When multiple drones try to initiate reconfiguration simultaneously:
- Higher reconfigId (based on timestamp) wins
- If equal, lower drone ID wins

This ensures exactly one reconfiguration succeeds.

---

## Project Structure

```
src/
  main.cpp                 # Entry point
  core/
    Simulation.cpp         # Main simulation loop
    Drone.cpp              # Individual drone agent
  membership/
    MembershipState.cpp    # The protocol implementation
  network/
    NetworkSimulator.cpp   # Network fault modeling
  control/
    Controller.cpp         # Formation control
    FormationPattern.cpp   # Shape definitions (circle, star, square, triangle)
  visualization/
    PolyscopeRenderer.cpp  # UI and rendering

include/                   # Headers mirror src structure
tests/                     # Protocol test harnesses
```

---

## Test Harnesses

Run the test suite to verify protocol correctness:

```bash
cd build
./NeighborhoodSurveillanceTests    # Comprehensive failure scenarios
./MembershipReconfigHarness        # Basic reconfiguration tests
./FormationPermutationTest         # Formation stability tests
```

---

## Documentation

Full protocol details, correctness proofs, and evaluation in: **[report.pdf](report.pdf)**

---

## Tips for Demos

1. **Start in Drone Show Mode** - It's visually impressive
2. **Switch to Membership Mode** to explain the protocol
3. **Kill a drone** and narrate what's happening:
   - "The neighbors detect the missing heartbeat..."
   - "One initiates reconfiguration..."
   - "They all agree on the new membership..."
4. **Try Cascade Failure** to show worst-case resilience
5. **Enable Console Logging** if you want to show the actual messages

---

## License

This project was created for educational and research purposes.
