# Changelog

All notable changes to MiniNav will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- Consolidated the simulation into a single `sim` binary (`src/apps/sim_main.cpp`)
  and a single `SimState` record type. `main` now reflects only the current design
  rather than carrying every past version side by side.
- Adopted a **tag-based versioning policy**: completed milestones are preserved as
  git tags + GitHub releases (`v0.1.0`–`v0.3.0`) plus `docs/` retrospectives, instead
  of as coexisting `sim_vN` binaries in the trunk. Regression protection now comes
  from tests and committed golden CSVs, not from keeping old binaries alive.
- Refactored `sim_main.cpp` into layered `NoisePreset` / `CliOptions` / `SimConfig` /
  `Simulator` components; the simulation output is byte-for-byte unchanged.
- Renamed version-tagged code identifiers now that a single simulation remains:
  `sim_v2` → `sim`, `SimStateV2` → `SimState`, `register_v2_statics` → `register_statics`,
  default output `data/traj_v2.csv` → `data/traj.csv`, Rerun application id
  `mininav_v2` → `mininav`. Analysis scripts updated to match.

### Removed

- `sim_v0` and `sim_v1` executables, the `SimStateV0` / `SimStateV1` state structs and
  their CSV/Rerun overloads, and the speculative `RobotModel` polymorphic wrapper.
  All remain reproducible from tags `v0.1.0` / `v0.2.0`.

## [0.3.0] - 2026-06-06

V2 — EKF Sensor Fusion.

Replaces V1's open-loop wheel odometry with a probabilistic Extended Kalman
Filter that fuses wheel encoders and a gyro, adds online gyro-bias estimation,
an RK4 process model, and NIS consistency diagnostics. V0/V1 are preserved as
regression baselines. Built incrementally across PRs #61–#64.

### Added

- `mininav.localization.ekf` — Extended Kalman Filter over the 6D state
  `[p_x, p_y, θ, v, ω, b_ω]`
    - Constant-velocity process model: position integrates the body twist
      `(v, ω)` while `(v, ω, b_ω)` evolve as random walks
    - Three-stage step `predict → update_encoder → update_imu`, with the
      encoder and gyro kept as **separate observations of the hidden state**
      (different sensor rates, better fault tolerance) rather than control
      inputs
    - **Joseph-form** covariance updates with forced symmetry every step for
      numerical stability
    - Process noise `Q` derived from the V1 actuator `α₁..₄` parameters;
      measurement noise `R` derived from the V1 physical sensor parameters
- `mininav.localization.ekf_state` — state-index constants,
  `make_initial_ekf_state` (μ₀ = 0, Σ₀ = diag(1e-6, 1e-6, 1e-6, 1e-2, 1e-2,
  1e-2)), and the `SimStateV2` versioned state struct (V0/V1 state untouched)
- `mininav.localization.encoder_observation` — `decode_encoder` and
  `encoder_noise_covariance`, deriving the 2D encoder measurement and its `R`
  from the physical encoder parameters at the filter's predicted velocity
- Selectable Euler/RK4 process integrator (`Integrator` enum): RK4 mean plus
  **analytic RK4 Jacobian**; the analytic Jacobian is validated column-by-column
  against central finite differences for **both** the Euler and RK4 paths
  (`ekf_jacobian_finite_diff_tests.cpp`)
- Online gyro-bias estimation: the gyro observes `ω + b_ω`, and the bias becomes
  **jointly observable** through the encoder's independent constraint on `ω`;
  gated by the process-noise term `q_bias_omega` (zeroed by `--no-bias`)
- NIS (Normalized Innovation Squared) consistency diagnostic: per-update encoder
  and gyro NIS returned from `update_*`, persisted to CSV, and covered by
  quadratic-form unit tests
- `mininav.sensors.imu_model` — gyro `ImuModel` with white noise `σ_omega`, a
  configurable true gyro bias, and an optional bias random-walk, each on an
  independent RNG stream
- `sim_v2` executable, coexisting with `sim_v0` / `sim_v1`
    - CLI flags: `--seed`, `--preset {low-noise|default|high-noise}`,
      `--integrator {euler|rk4}`, `--out`, `--q-scale`, `--r-scale`,
      `--no-bias`, `--rrd`, `--no-viz`. `--q-scale` / `--r-scale` tune only the
      filter's `Q` / `R` — the simulated truth and measurements are untouched
- `csv_header(SimStateV2)` / `csv_row(SimStateV2)` and `log_to_rerun(SimStateV2,
  ...)` ADL overloads, including the EKF mean/covariance, gyro-bias estimate
  and `Σ_bb`, and per-step NIS columns; CSV header comments additionally embed
  `mode`, `integrator`, `q_scale`, `r_scale`, and `bias`
- Rerun: EKF fused pose at `/world/robot/ekf` (with trail) and the gyro-bias
  learning curves `/plots/bias_omega/ekf` vs `/plots/bias_omega/truth`
- Version-organized Python analysis scripts under `scripts/v2/`:
  `analyze_ekf.py` (three-trajectory overlay, cumulative RMSE, NIS, 3σ
  state-error envelopes, bias learning), `analyze_covariance.py` (3σ
  position-covariance ellipse evolution + animated GIF), `analyze_integrator.py`
  (single-seed RK4-vs-Euler attribution), and `sweep_integrator.py` (multi-seed
  RK4-vs-Euler average); figures emitted to `results/v2/`
- Unit tests for EKF predict/update, the finite-difference Jacobian check over
  both integrators, gyro-bias observability, the NIS quadratic form, and
  bit-equality of the scalar `update_imu` against the general 5×5 Joseph flow
- Documentation: `docs/experiments/v2_ekf_fusion.md` (20-seed EKF-vs-odom RMSE
  study, the bias-estimation operating envelope, NIS consistency, and the
  covariance/observability analysis), plus the math derivations
  `docs/math/EKF_Foundations.md` and `docs/math/runge_kutta_integration.md`

### Changed

- Reorganized `scripts/` and `results/` by version: the V1 drift analysis moved
  to `scripts/v1/analyze_drift.py` and its figures to `results/v1/`, mirroring
  the new `scripts/v2/` and `results/v2/` layout

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
- Documentation: `docs/v1_summary.md` (V1 retrospective)
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