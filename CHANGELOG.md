# Changelog

All notable changes to MiniNav will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.5.0] - 2026-10-05

V4 — Closed-Loop Path Tracking.

Closes the estimate → plan → track loop for the first time, in the deterministic
C++ simulation: `sim nav` plans from the EKF's initial estimate, smooths the
path, and tracks it with a Regulated Pure Pursuit subset that sees the EKF
estimate, not ground truth, on a plant with actuator saturation and lag. Control
error is measured separately from localization drift. Built across PRs #81–#86,
after the roadmap re-scope (`c1379a5`).

### Added

- `control` static library (depends only on `core`; yaml-cpp private), with
  interfaces shaped like `nav2_core`
    - `Controller` / `GoalChecker` / `ProgressChecker`; `compute_velocity_commands`
      takes a `const GoalChecker*` as in Nav2, and the progress checker takes
      explicit simulation time so runs stay deterministic
    - `PurePursuitController` — curvature κ = 2·y_g / d², velocity-scaled
      look-ahead clamped to [L_min, L_max], curvature regulation from a
      fixed-distance look-ahead, approach slowdown, rotate in place toward the
      path or the goal heading, and an ω limit that lowers v while keeping κ;
      three `use_*` switches turn it into classic Pure Pursuit for ablations
    - `VelocitySmoother` — acceleration limits that scale both components
      together, so the commanded curvature is preserved
    - `SimpleGoalChecker` (position latch + optional yaw) and
      `SimpleProgressChecker` (stuck detection)
    - Per-section config parsers (`controller`, `goal_checker`,
      `progress_checker`): derived defaults, unknown keys rejected
- `simulation` static library — `Plant` (actuator dynamics → actuator noise →
  encoder / IMU → ground truth) and the noise-preset table, including a new
  all-zero `none` preset; `ActuatorDynamics` adds per-axis saturation and a
  first-order lag with exact discretization, and defaults to a pass-through
  that does no floating-point work
- `localization::EkfPipeline` — encoder decoding, R evaluation and the
  predict / update sequence, with an injectable initial pose
- `mininav.core.path` — `Path` moved from `planning` into `core`, plus polyline
  geometry: `project_onto` (windowed, monotone progress, so a U-turn's return
  leg cannot steal the projection) and `lookahead_point` (circle–segment
  intersection, no resampling)
- `mininav.core.robot_description` + `config/robot.yaml` — the single source of
  robot geometry, footprint (circumscribed radius), velocity limits and
  actuator time constant; every field required, unknown keys rejected
- `planning::path_smoothing` — true start / goal endpoints and greedy
  line-of-sight shortcutting, checked by a supercover `segment_is_free` that
  applies A\*'s no-corner-cutting rule; `sim plan --smooth`;
  `OccupancyGrid::is_traversable` shared by A\* and smoothing;
  `AStarPlanner::costmap()` / `config()`
- `sim nav` — closed-loop navigation: 20 Hz controller with zero-order hold on
  a 100 Hz plant, `--controller-input ekf|truth` (oracle), goal yaw, `--path`
  to follow a given path without planning (Nav2 FollowPath), and the
  terminal states `arrived` / `collision` (ground-truth footprint vs the
  uninflated map) / `stuck` / `timeout` / `plan_failed`
- `config/nav.yaml` — `planner`, `path_smoothing`, `controller`,
  `goal_checker` and `progress_checker` sections mirroring the Nav2 parameter
  files; an unknown section, or an inflation radius below the robot's
  circumscribed radius, fails before the run
- `nav.csv` — `SimState` columns plus actuator output, look-ahead, curvature,
  regime, the `e_ctrl` / `e_true` / `e_est` error split, arclength and
  clearance; no wall-clock value, so the same seed gives a byte-identical file
- Rerun closed-loop view (`mininav.viz.nav_log`): raw and smoothed paths,
  look-ahead point and pursuit arc, EKF 3σ ellipse, error / clearance / regime
  plots; `VizSink` gains per-frame `log_points` / `log_line_strip`
- `maps/apartment` — a 10 m × 7 m floor plan (placeholder layout) generated
  from `maps/src/apartment.toml` by `scripts/v4/gen_floorplan.py`, connected at
  0.20–0.30 m inflation
- Golden CSV regression guard (`tests/golden/`, CTest label `regression`):
  three EKF runs, three plans and one closed-loop run, compared by
  `csv_compare` (integers exact, floats within 1e-9 relative); deliberate
  updates only, through the `update_golden` target
- `.gitattributes` (`* text=auto eol=lf`, `*.pgm -text`, images binary)
- CTest labels `simulation`, `control` (including closed-loop checks against
  the analytic results: e^−π overshoot, 4.26 L settling, zero error on arcs,
  corner-cut scaling), `regression` and `nav` (end-to-end `sim nav` runs);
  184 → 349 tests
- Python scripts under `scripts/v4/`: `scenarios.py`, `step_response.py` (E1),
  `corner_cutting.py` (E2), `run_scenarios.py` and `tracking_error.py`
  (E3 / E4), `animate_nav.py` (README GIF); figures in `results/v4/`
- Documentation: `docs/v4_summary.md`, `docs/experiments/v4_control.md`
  (theory vs simulation, corner cutting and safety margins, the look-ahead
  trade-off, the error split and drift over distance), and
  `docs/math/pure_pursuit.md` (geometry, linearization, the lag stability
  bound τ < T_L, corner cutting including a small-angle closed form, the
  safety-margin budget, the 1-Lipschitz error split, the s^(3/2) drift law)

### Changed

- Roadmap re-scoped: V4 closes the plan–track loop in the deterministic C++
  simulation, and all ROS 2 work — nodes on standard messages, plus the A\*
  planner and the controller as Nav2 plugins — moves to V5
  (`docs/project_overview.md` §6)
- `sim` takes CLI11 subcommands — `sim ekf`, `sim plan`, `sim nav` — instead of
  mode flags; bare `sim` prints help and old flag-style calls are rejected.
  `src/apps/sim_main.cpp` became the module `mininav.apps.sim` (one
  implementation unit per mode plus shared helpers). EKF and planning output is
  byte-for-byte unchanged (checked by the golden tests)
- EKF in the closed loop: `ProcessNoiseParams` gains `q_dv` / `q_dw` for
  velocity changes the constant-velocity model cannot see (0 in `sim ekf`), and
  the filter's gyro σ is floored at the BNO055 quantization σ so `update_imu`
  always gets R > 0
- CI runs a Debug + Release matrix; the Release job executes the 50 ms A\*
  budget and checks the golden baselines under optimization

### Removed

- `src/apps/sim_main.cpp` (replaced by `src/apps/sim/`) and
  `src/planning/grid_types.cpp` (`Path` moved to `core`)

## [0.4.0] - 2026-09-27

V3 — Global Path Planning.

Adds the first model of the environment: an occupancy-grid map loaded from a
ROS-style PGM + `map.yaml`, Euclidean obstacle inflation, and an A\* global
planner behind a `nav2_core`-shaped `GlobalPlanner` facade, driven from
`sim --map` with a YAML planner config. Also swaps the logger backend to spdlog
and makes the visualization layer mockable with gmock. Built across PRs
#67–#73.

### Added

- `planning` static library (depends only on `core` and yaml-cpp)
    - `mininav.planning.grid_types` — `GridCoord`, world-frame `Path` with
      cumulative `length()`, `Heuristic` / `Connectivity` enums,
      `PlannerConfig`, and `constexpr is_admissible(Heuristic, Connectivity)`
    - `mininav.planning.occupancy_grid` — `OccupancyGrid` with ROS occupancy
      values (free 0 / occupied 100 / unknown −1), lower-left `origin`,
      floor-based `world_to_grid`, cell-center `grid_to_world`, and
      out-of-bounds cells reading as occupied
    - `mininav.planning.map_io` — `load_occupancy_grid(map.yaml)`: self-written
      PGM parser (ASCII P2 and binary P5, 8-bit), ROS threshold semantics
      (`occupied_thresh` / `free_thresh` / `negate`), y-axis flip, image path
      resolved relative to the YAML file, and descriptive errors for missing
      files, missing keys, bad magic, or pixel-count mismatches
    - `mininav.planning.inflation` — multi-source Euclidean distance transform
      (`obstacle_distance_cells`, `costmap_2d`-style nearest-source
      propagation) and boolean `inflate(grid, radius_m)`; unknown cells inside
      the radius are conservatively marked occupied
    - `mininav.planning.astar` — `GlobalPlanner` interface and `AStarPlanner`:
      flat `g` / `parent` / `closed` tables, binary-heap open set with lazy
      deletion, 4/8 connectivity, Manhattan / Euclidean / Octile heuristics,
      tie-breaking toward the goal, corner-cutting prevention, and an opt-in
      clearance cost gradient (`cost_weight`); `PlanResult` carries the path,
      success flag, expanded-node count, and planning time
    - `mininav.planning.planner_config` — `planner.yaml` parse / load /
      serialize with yaml-cpp; unknown (misspelled) keys are rejected at load time
- Admissibility validation: pairing the Manhattan heuristic with
  8-connectivity (which overestimates diagonal steps and silently breaks the
  optimality guarantee) is rejected by `parse_planner_config` and by the
  `AStarPlanner` constructor, so CLI overrides are caught too
- `sim --map` planning mode — a one-shot, RNG-free plan with `--map`,
  `--config`, `--start`, `--goal`, `--heuristic`, `--connectivity`, and
  `--inflation-radius`; writes a byte-deterministic `path.csv` whose header
  embeds the map, start, goal, heuristic, connectivity, inflation radius,
  success flag, expanded-node count, and path length
- `config/planner.yaml` — default planner configuration (Octile,
  8-connectivity, 0.05 m inflation)
- Maps: hand-drawn `corridor`, `room`, `maze`, and `office` (P2) plus a
  procedurally generated 500×500 `office500` floor plan (P5)
- Rerun planning view: `PlanScene` + `log_plan` (`mininav.viz.plan_log`) log the
  occupied cells, inflation margin, path, and start/goal poses as static
  entities; `VizSink` gains backend-agnostic `log_points_static` /
  `log_line_strip_static` primitives, so `viz` does not depend on `planning`
- `VizSink` abstract interface with `RerunSink` as its implementation, and a
  gmock-based `viz_tests` target (CTest label `viz`) that asserts the entity
  layout without spawning a viewer
- `planning_tests` (CTest label `planning`, 62 tests), including exact optimal
  path lengths, heuristic agreement under 4- and 8-connectivity, rejection of
  inadmissible pairings, and a 200×200 serpentine stress maze with
  build-independent expansion-count checks plus the Release-only 50 ms timing
  budget
- Python scripts under `scripts/v3/`: `plot_plan.py` (map + inflation + path
  figure), `benchmark_planner.py` (timing on random N×N maps),
  `optimality_check.py` (A\* vs Dijkstra ground truth), `animate_search.py`
  (animated A\* expansion), `gen_office500.py` (floor-plan generator); figures
  emitted to `results/v3/`, named after the map
- Documentation: `docs/v3_summary.md`, `docs/experiments/v3_planning.md` (timing,
  optimality, heuristic admissibility, determinism), and
  `docs/math/astar_planning.md` (grids, configuration-space inflation, the A\*
  optimality proof, heuristic admissibility and consistency)

### Changed

- Logger backend switched from the hand-written iostream logger to spdlog,
  linked privately into `core`; the `mininav.core.logger` interface is
  unchanged, messages are passed as `"{}"` arguments rather than format
  strings, and the `noexcept` contract is preserved
- `sim_state_log` free functions take `VizSink&` instead of `RerunSink&`
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