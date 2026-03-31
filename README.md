# MiniNav

MiniNav is a modular C++ project for learning and building an indoor mobile robot localization and navigation system step by step.

## Current stage

V0: ideal 2D motion simulation

## Project goals

- Build a clean and extensible robotics project in modern C++
- Learn robotics through implementation
- Gradually implement motion modeling, odometry, EKF, planning, and control
- Prepare a strong project portfolio for robotics / autonomous systems job applications

## Planned milestones

- V0: ideal motion simulation
- V1: odometry and noise
- V2: EKF localization
- V3: engineering refactor
- V4: ROS2 integration
- V5: map + planning + control
- V6: Raspberry Pi robot deployment

## Build

```bash
cmake -S . -B build -G Ninja -DCMAKE_CXX_COMPILER=clang++
cmake --build build
```

## Run
```bash
./build/sim_v0
```