# Drone Show Distributed Membership & Visualization Simulator



This repository contains the implementation, tests, and visualization environment for a distributed **neighborhood-surveillance–based group membership protocol** and **drone-show formation controller**.

The simulator models:



* Heartbeat-based failure detection

* Deterministic closest-neighbor selection

* A 3-phase atomic broadcast protocol for reconfiguration

* Dynamic network conditions (latency, loss, jitter, burst failures)

* Formation interpolation & real-time drone-show morphing

* Full Polyscope visualization (neighbors, packets, formations)



## Full Report



The full write-up, including protocol details, proofs, design discussion, and evaluation, is in the report PDF:



**[report.pdf](report.pdf)**



## Build Instructions



### Requirements



* CMake ≥ 3.16

* C++20 compiler

* Polyscope (fetched automatically using `FetchContent`)

* glm



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



## Project Structure



```

src/                 # Simulator, membership, networking, visualization

include/             # Headers

tests/               # Membership harness, formation tests

report.pdf           # Full writeup and documentation

CMakeLists.txt

```



