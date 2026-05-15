# Changelog

All notable changes to MiniNav will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.2.0] - 2026-05-16

V1 — Sensors, Noise & Odometry Drift.

Introduces two independent imperfect channels — actuator-side and sensor-side —
on top of the V0 deterministic baseline, plus an open-loop wheel-odometry
estimator. Establishes the drift problem that V2's EKF will solve.

### Added

- `mininav_sensors` static library
    - `ActuatorModel` implementing the Velocity Motion Model (Thrun,
      *Probabilistic Robotics* §5.3): variance scales as
      `α₁v² + α₂ω²` and `α₃v² + α₄ω²`, so a stationary command produces
      zero variance (no static drift)
    - `WheelEncoderModel` with physically causal pipeline:
      inverse kinematics → multiplicative slip noise → accumulated arc
      length → integer-tick quantization → differential output
    - Accumulate-then-difference semantics correctly handle low-speed
      undersampling where `v·dt < Δs_tick`
- `mininav_localization` static library
    - `WheelOdometry` estimator with a dependency-inverted interface:
      `update(EncoderTicks, dt)` has no knowledge of where the ticks
      originated, enabling drop-in replacement on real hardware in V6
- `core::EncoderTicks` plain struct as the typed boundary between
  `sensors` and `localization` — the two libraries do not link each
  other, only `core`
- `core::SimStateV1` versioned state struct (V0 state untouched)
- `core::kinematics`: `inverse_kinematics` and `forward_kinematics`
  free functions
- `core::RngFactory` with per-tag FNV-1a 64-bit seed derivation,
  guaranteeing independent RNG streams per noise source and stable
  sequences when new noise sources are added later
- `csv_header(SimStateV1)` / `csv_row(SimStateV1)` and
  `log_to_rerun(SimStateV1, ...)` ADL overloads
- `sim_v1` executable, coexisting with `sim_v0` (V0 preserved as
  regression baseline)
- CLI11 integration via `cmake/cli11.cmake` (FetchContent +
  `FIND_PACKAGE_ARGS`, header-only, system include, excluded from `ALL`)
- CLI flags on `sim_v1`: `--seed`, `--preset {low-noise|default|
  high-noise}`, `--rrd`, `--no-viz` (with mutex / membership validation)
- Self-documenting CSV output: every `traj_v1.csv` carries header
  comments with `seed`, `preset`, `dt`, `duration`, `generated_at`,
  making any saved trajectory exactly re-runnable
- Three-trajectory Rerun view: `cmd_traj` (green), `truth` (blue),
  `odom` (orange), plus diagnostic scalar time series for `cmd_v/w`,
  `true_velocity_v/w`, encoder `dticks_l/r`, and direct
  `error/position` + `error/yaw`
- 16 new unit tests across `core_tests`, `sensors_tests`,
  `localization_tests` executables, each pinning a specific
  engineering invariant (e.g. `ZeroCommandDoesNotConsumeRng`,
  `LowSpeedAccumulatesCorrectlyDespiteRoundingToZero`)
- Python drift-analysis script `scripts/analyze_v1_drift.py`
  producing `results/v1_trajectory.png` and
  `results/v1_drift_over_time.png`
- Documentation: `docs/v1_summary.md` (V1 retrospective) and
  `docs/experiments/v1_odom_drift.md` (drift experiment writeup)

### Fixed

- Rerun trail isolation in `viz/rerun_sink.cpp`: per-entity trail
  state is now keyed by entity path via `unordered_map`, so the three
  trajectories no longer share trail buffers

## [0.1.0] - 2026-05-04

### Added
- Initial simulation foundation with C++23 modules
- Eigen3 integration for robot state representation
- Basic differential drive kinematics
- CSV trajectory output
- CMake 3.28 + Clang 18 + Ninja build system
- `mininav.core.*` module namespace structure