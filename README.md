<div align="center">

# MiniNav

[![CI](https://github.com/xinlei-robotics/MiniNav/actions/workflows/ci.yml/badge.svg)](https://github.com/xinlei-robotics/MiniNav/actions/workflows/ci.yml)
[![C++23](https://img.shields.io/badge/C%2B%2B-23-blue.svg)](https://en.cppreference.com/w/cpp/23)
[![CMake](https://img.shields.io/badge/CMake-3.28-064F8C.svg)](https://cmake.org/)
[![License](https://img.shields.io/badge/license-MIT-lightgrey.svg)](LICENSE)

**Indoor mobile robot localization & navigation system in modern C++.**

From kinematic simulation to a Raspberry Pi 5 + 4WD car indoor navigation
demo — built incrementally, version by version.

<img src="results/v4/nav_s3.gif" alt="MiniNav V4 — Regulated Pure Pursuit tracking a smoothed A* path on the EKF estimate" width="760"/>

*V4: closed loop in the deterministic C++ simulation — A\* plans from the EKF
estimate, the path is smoothed (gray → green), and a Regulated Pure Pursuit
controller tracks it through two doorways using the **estimate**, not the
ground truth (black: true trajectory, red: EKF). Right: the controller's view —
chassis, pursuit arc to the look-ahead point, and the EKF 3σ ellipse.
Experiments: [`docs/experiments/v4_control.md`](docs/experiments/v4_control.md).*

</div>

---

## What is MiniNav?

MiniNav is a personal robotics project that builds a complete — though
deliberately simplified — indoor navigation stack for a mobile robot.
Every layer of the classic mobile-robotics pipeline is implemented from
scratch in modern C++, validated in simulation first, and progressively
brought onto real hardware.

It answers the three core questions of mobile robot navigation:

| Question                | Topic        | Core technique                         |
|-------------------------|--------------|----------------------------------------|
| **Where am I?**         | Localization | Wheel odometry, IMU, EKF sensor fusion |
| **Where am I going?**   | Planning     | Occupancy grid map, A\* global planner |
| **How do I get there?** | Control      | Pure Pursuit path tracking             |

The project is organized as a multi-stage roadmap (V0 → V6), each
version solving one well-scoped problem and building on the previous one.

> **Current status: V4 complete.** The simulated robot now drives itself: `sim nav`
> plans from the EKF's estimate, smooths the A\* path, and tracks it with
> Regulated Pure Pursuit on a plant with actuator saturation and lag — steering
> by the *estimate*, not the ground truth. See
> [`docs/experiments/v4_control.md`](docs/experiments/v4_control.md) for the
> experiments, [`docs/math/pure_pursuit.md`](docs/math/pure_pursuit.md) for the
> derivations, and [`docs/v4_summary.md`](docs/v4_summary.md) for the design
> retrospective. Next: V5 moves the loop into ROS 2 + Nav2.

---

## Latest milestone — V4: Closed-loop path tracking

V4 closes the estimate → plan → track loop for the first time. The robot
follows a path it planned itself, steering by its own EKF estimate — and runs
into a wall when that estimate drifts too far. Everything runs in the
deterministic C++ simulation, so every result replays from a seed. It was built
across six PRs: a golden-CSV regression guard (#81), the plant / estimator split
and `robot.yaml` (#82), the `control` library (#83), path smoothing and a
real-scale floor plan (#84), the closed-loop `sim nav` (#85), and the
experiments (#86).

<p align="center">
  <img src="results/v4/e4_drift.png" alt="V4 — dead-reckoning drift vs distance and collision-free runs on a 34 m route" width="90%"/>
</p>

### The loop

- **Regulated Pure Pursuit** (a subset of Nav2's RPP) behind interfaces shaped
  like `nav2_core::Controller` / `GoalChecker` / `ProgressChecker`: velocity-scaled
  look-ahead, curvature regulation from a fixed-distance look-ahead, approach
  slowdown, rotate-in-place and an ω limit, plus a velocity smoother that
  scales both components together so the commanded curvature survives. The
  `control` library depends only on `core`, so V5 can wrap it as a Nav2 plugin.
- **A realistic plant.** Actuator saturation and a first-order lag (exact
  discretization), a 20 Hz controller with zero-order hold on a 100 Hz plant,
  and one source of truth for robot geometry and limits: `config/robot.yaml`.
- **Smoothed paths.** The A\* staircase gets its true start and goal and is
  straightened by line-of-sight shortcuts, checked with the same
  no-corner-cutting rule as A\* — every 45° step was a curvature impulse for the
  controller.
- **The EKF in the loop.** The V2 filter runs unchanged except for process
  noise covering the velocity changes the controller makes, which a
  constant-velocity model cannot see.
- **An honest error split.** `nav.csv` records the control error (estimate to
  path), the estimation error and the true error separately, and
  `--controller-input truth` runs an oracle with perfect localization.
- **A regression guard.** Golden CSVs for the EKF, planning and closed-loop
  modes are compared in CTest on a Debug + Release CI matrix, so the
  restructuring that made room for the loop provably changed no output.

### What the experiments found

Full report in [`docs/experiments/v4_control.md`](docs/experiments/v4_control.md);
derivations in [`docs/math/pure_pursuit.md`](docs/math/pure_pursuit.md).

- **The theory holds to about 2 %.** Linearized Pure Pursuit is a second-order
  system with damping ratio 1/√2 at every speed; its response depends only on
  distance travelled over look-ahead. Simulated overshoot and settling distance
  match the analytic values within 2 %, a third-order lag model predicts the
  effect of actuator lag within 0.3 percentage points, and velocity-scaled
  look-ahead keeps the response identical across speeds — the first-principles
  reason behind Nav2's `use_velocity_scaled_lookahead_dist`.
- **Defaults are derived, not tuned.** The stability bound (actuator lag below
  the look-ahead time) and a safety-margin budget give the default parameters;
  500 paired-seed runs over look-ahead time and speed found no setting that
  beats them on every metric, so they stayed unchanged.
- **The controller does its part.** Across 5 scenarios × 10 seeds with the EKF
  in the loop, the control error averages **0.58 cm** and peaks at **6.82 cm**
  (targets: 10 cm and 30 cm). With noise-free or ground-truth feedback, 60 of 60
  runs arrive within 5 cm with zero collisions.
- **Localization is the bottleneck.** With only encoders and a gyro, position
  error grows as s^1.5 with distance (a heading random walk, measured slope
  1.49–1.52), and after about 5 m one run in ten has drifted by more than a
  doorway's clearance. On a 34 m route, 19 of 20 EKF-driven runs hit a wall
  while 10 of 10 oracle runs arrive. That sets V5's scenario length and makes
  absolute localization a V6 requirement.
- **A full controller is less stable than its linear model.** At a look-ahead
  time of 0.5 s the linear model is still stable, but the real controller falls
  into a rotate-in-place ↔ track limit cycle that doubles the time to goal.

V4 is where MiniNav first moves under its own control, and where the cost of
localization drift became something you can measure in collisions.

---

## Previous milestone — V3: Global path planning

V3 gave MiniNav its first model of the environment: an occupancy grid loaded
from a ROS-style PGM + `map.yaml`, Euclidean obstacle inflation, and an A\*
planner behind a `nav2_core`-shaped `GlobalPlanner` facade.

<p align="center">
  <img src="results/v3/search_office500.gif" alt="V3 — A* search across a 25 m × 25 m floor plan" width="60%"/>
</p>

Findings from [`docs/experiments/v3_planning.md`](docs/experiments/v3_planning.md):

- **Fast** — 200×200 random maps plan in **3.9 ms at p95** (Release) against a
  50 ms budget; the 500×500 floor plan above plans in ~12 ms.
- **Optimal** — path lengths match a Dijkstra ground truth exactly.
- **Optimality is conditional** — Manhattan distance on an 8-connected grid
  overestimates diagonals and silently returned a path 1.13 cells too long;
  that pairing is now rejected at config load and in the planner constructor.

---

## Earlier milestones — V0 to V2

**V0** established the scaffolding every later version reuses: C++23
modules, differential-drive kinematics, the `Trajectory<T>` container with
ADL extension points, and deterministic CSV next to live Rerun output.
**V1** introduced two independent imperfect channels — a Velocity Motion
Model at the actuator and slip + quantization at the wheel encoder — and
quantified the open-loop odometry drift. **V2** replaced that odometry with a
6-state Extended Kalman Filter (`[pₓ, p_y, θ, v, ω, b_ω]`) that fuses encoders
and a gyro as observations, estimates the gyro bias online, and is checked with
NIS diagnostics; across 20 seeds the gain over odometry is preset-dependent
(−48.9 % position RMSE at `low-noise`, −8.3 % at `default`), and online bias
estimation destabilizes the filter at `high-noise`. See
[`docs/v0_summary.md`](docs/v0_summary.md), [`docs/v1_summary.md`](docs/v1_summary.md),
[`docs/v2_summary.md`](docs/v2_summary.md) and
[`docs/experiments/v2_ekf_fusion.md`](docs/experiments/v2_ekf_fusion.md).

---

## Roadmap

| Version | Theme                    | Key deliverables                                                                                   | Status |
|---------|--------------------------|----------------------------------------------------------------------------------------------------|--------|
| **V0**  | Simulation scaffolding   | Differential-drive kinematics, `Trajectory<T>`, CSV/Rerun dual output, GoogleTest, strict warnings | ✅      |
| **V1**  | Sensors, noise, odometry | Velocity Motion Model, encoder slip + quantization, `WheelOdometry`, drift experiments             | ✅      |
| **V2**  | EKF state estimation     | Gyro IMU model, 6-state EKF (predict + encoder/IMU updates), online gyro-bias estimation, RK4 process model, NIS diagnostics, 20-seed RMSE study vs odom baseline | ✅      |
| **V3**  | Path planning            | Occupancy grid (PGM + `map.yaml`), Euclidean obstacle inflation, A\* with admissibility-checked heuristics, YAML planner config, spdlog / gmock, `sim --map` planning mode | ✅      |
| **V4**  | Closed-loop path tracking | Regulated Pure Pursuit tracking the smoothed A\* path in the C++ simulation with the EKF estimate in the loop; actuator lag and saturation; control error measured separately from localization drift; golden CSV regression | ✅      |
| **V5**  | ROS 2 + Nav2 integration | Simulation and EKF nodes on standard messages; the A\* planner and the controller as Nav2 plugins; RViz2 goal-to-arrival demo |        |
| **V6**  | Real-world deploy        | Sim-to-real on Pi 5 + 4WD car, indoor navigation video                                             |        |

---

## Engineering foundations

Engineering capabilities shared by every version — most established in V0,
extended as the stack grew.

- **C++23 modules** via CMake 3.28 `FILE_SET CXX_MODULES`. Module
  interface files (`.ixx`) export only the API surface; heavy headers
  like Eigen stay in implementation files or global module fragments,
  keeping the module scan cost manageable.
- **ADL-based extension points**. `csv_row(T)` and `log_to_rerun(T, ...)`
  are free functions resolved by Argument-Dependent Lookup. Supporting a
  new state type means adding overloads rather than touching the
  `Trajectory` container or the Rerun sink — the serialization and viz
  layers stay open for extension.
- **Plain-data state struct**. `SimState` is plain data (no inheritance);
  the `Trajectory` container is a class template parameterized over it, so
  the same container code serves any record type.
- **Interface-isolated, mockable visualization**. Visualization code talks
  to an abstract `VizSink`; `RerunSink` is its only Rerun-aware
  implementation and hides the SDK behind `unique_ptr<Impl>`, so downstream
  targets never transitively `#include <rerun.hpp>`. gmock-based tests
  assert the entity layout without spawning a viewer.
- **Dual-track output**. CSV is deterministic and diff-able (regression
  baseline + Python post-processing); Rerun is interactive (live
  development); static PNGs are the publication artifact. Each format
  has a different reader and a different job.
- **Golden regression on a Debug + Release CI matrix**. Every mode writes
  deterministic CSV (the same seed gives the same file, with no wall-clock
  values). Committed baselines for the EKF, planning and closed-loop modes are
  compared in CTest by a small `csv_compare` tool — integers exact, floats
  within 1e-9 relative to absorb libm differences between CPUs — and change
  only through a deliberate `update_golden` target.
- **External configuration and structured logging**. yaml-cpp reads
  `robot.yaml` (the single source of robot geometry and limits),
  `planner.yaml` and `map.yaml`, rejecting unknown keys; spdlog backs the unchanged
  `mininav.core.logger` interface and is linked privately into `core`, so no
  spdlog type leaks downstream.
- **Strict warning policy**. `-Wall -Wextra -Wconversion -Werror` on
  Debug, with `SYSTEM` exemption for third-party headers — *strict on
  our code, permissive on theirs*.
- **Modern toolchain**. Clang 18, CMake 3.28 + Ninja; CLI11, GoogleTest /
  gmock, Rerun SDK, yaml-cpp and spdlog fetched via `FetchContent` (a system
  install is used when present); `gtest_discover_tests` for per-test CTest
  registration; `compile_commands.json` for clangd integration.

---

## Architecture

```
┌─────────────────────────────────────────────┐
│ Layer 5: Real Robot Deployment              │  Raspberry Pi 5 + 4WD car  (V6)
├─────────────────────────────────────────────┤
│ Layer 4: Motion Control                     │  Regulated Pure Pursuit    (V4 ✅)
├─────────────────────────────────────────────┤
│ Layer 3: Global Planning                    │  Occupancy grid + A*       (V3 ✅)
├─────────────────────────────────────────────┤
│ Layer 2: Localization & State Estimation    │  Odom + IMU + EKF          (V1 ✅, V2 ✅)
├─────────────────────────────────────────────┤
│ Layer 1: Kinematic Simulation               │  Differential-drive model  (V0 ✅)
└─────────────────────────────────────────────┘
```

### Module dependencies

```mermaid
graph TD
    sim[sim] --> core[core]
    sim --> simulation[simulation]
    sim --> localization[localization]
    sim --> planning[planning]
    sim --> control
    sim --> viz[viz]
    sim --> cli11[CLI11]

    simulation --> sensors[sensors]
    simulation --> core
    sensors --> core
    localization --> core
    planning --> core
    planning --> yamlcpp[yaml-cpp]
    control[control] --> core
    control -.->|private| yamlcpp
    viz --> core
    viz --> rerun[Rerun SDK]
    core --> eigen[Eigen3]
    core -.->|private| spdlog[spdlog]
    core -.->|private| yamlcpp

    style core fill:#0a4f3f,color:#fff
    style simulation fill:#1f4f1f,color:#fff
    style sensors fill:#1f4f1f,color:#fff
    style localization fill:#1f3f5c,color:#fff
    style planning fill:#5c3a1f,color:#fff
    style control fill:#5c1f3a,color:#fff
    style viz fill:#3a1f5c,color:#fff
    style sim fill:#5c4a1f,color:#fff
```

The simulated robot and the estimator remain **independent of each other**.
`simulation::Plant` (actuator noise, encoder and IMU models, ground truth)
and `localization::EkfPipeline` (decode, predict, update) communicate only
through plain structs — `EncoderTicks` plus a scalar gyro reading — passed
through the `sim` main loop. The `Ekf` lives in the *same* `localization`
library as the `WheelOdometry` baseline and consumes the *same*
`EncoderTicks`; `sim` runs both side by side so the EKF can be scored
against the odometry baseline on identical sensor streams. This dependency
inversion is what makes V6 work without modifying the estimator: real GPIO
ticks plug into the same struct the simulated encoder produces. It is also
the V5 node boundary — a simulation node wraps the `Plant`, an EKF node
wraps the `EkfPipeline`.

`planning` follows the same rule: it depends only on `core` and yaml-cpp —
an `OccupancyGrid` goes in, a `Path` (a `core` type, like `nav_msgs/Path` in
ROS) comes out — so a SLAM-built map or a
real robot's start pose can feed the same planner unchanged. `viz` in turn
does not depend on `planning`: a plan reaches the viewer as plain geometry
(points and poses), converted by the app.

`control` (V4) depends only on `core` as well: the Regulated
Pure Pursuit controller, velocity smoother, and goal / progress checkers
mirror the `nav2_core` plugin interfaces and see only `Path`, `Pose2D` and
`Twist2D` — no grid, no EKF, no sensors — so V5 can wrap them as Nav2 plugins
unchanged. The closed loop is assembled in the app (`sim nav`), and its view
reaches Rerun as plain geometry through `nav_log`, so `viz` still depends on
neither `planning` nor `control`.

**Versioning policy.** `main` reflects the current best design; superseded
code is refactored away rather than kept alongside. Each completed
milestone is preserved as a git tag and GitHub release (`v0.1.0`=V0,
`v0.2.0`=V1, `v0.3.0`=V2, `v0.4.0`=V3, `v0.5.0`=V4) plus a retrospective in `docs/`, so
every prior version stays reachable through history without weighing down
the trunk.

---

## Build & run

### Prerequisites

- Linux (or WSL 2) — tested on Ubuntu 24.04
- Clang 18+ with C++23 modules support
- CMake 3.28+, Ninja
- Eigen3 ≥ 3.4 (`sudo apt install libeigen3-dev`)
- Python venv with `rerun-sdk==0.31.4`:

  ```bash
  python3 -m venv .venv
  .venv/bin/pip install -r requirements.txt
  ```

CLI11, GoogleTest, the Rerun SDK, yaml-cpp, and spdlog are fetched
automatically via `FetchContent`.

### Build

```bash
# First-time configure (downloads Rerun SDK on first run)
cmake --preset clang18-debug

# Incremental builds
cmake --build --preset build-debug -j

# Run all tests (core / sensors / simulation / localization / planning / control / viz /
# regression / nav)
ctest --preset test-debug --output-on-failure

# Golden CSV regression only (end-to-end sim runs vs tests/golden/)
ctest --preset test-debug -L regression --output-on-failure
```

### Run the simulation

`sim` takes a subcommand: `sim ekf` runs the localization simulation,
`sim plan` a one-shot global plan, and `sim nav` the closed loop (V4).
`sim <mode> --help` lists each mode's options; `sim` alone prints the
overview.

```bash
# Default: random seed, default preset, RK4 integrator, online bias estimation
./build/clang18-debug/sim ekf

# Fully reproducible run (the seed prints to stdout when omitted)
./build/clang18-debug/sim ekf --seed 42 --preset default

# Disable online gyro-bias estimation (the 'ekf (no bias)' baseline).
# Write to its own file so it doesn't clobber the with-bias run above.
./build/clang18-debug/sim ekf --seed 42 --preset default --no-bias --out data/traj_nobias.csv

# Sensitivity knobs: scale the EKF's physics-derived Q / R (1.0 = physical
# value). These tune the *filter* only — the simulated truth/measurements
# are untouched.
./build/clang18-debug/sim ekf --q-scale 2.0      # trust the motion model less
./build/clang18-debug/sim ekf --r-scale 0.5      # trust the sensors more

# RK4-vs-Euler attribution: same seed/preset, integrator the only difference
./build/clang18-debug/sim ekf --integrator euler --out data/traj_euler.csv
./build/clang18-debug/sim ekf --integrator rk4   --out data/traj_rk4.csv

# Headless / CI mode — only writes data/traj.csv
./build/clang18-debug/sim ekf --no-viz
```

Robot geometry (wheel radius, wheel base, encoder resolution, footprint,
actuator limits) comes from `config/robot.yaml`; pass `--robot <file>` to
simulate a different robot.

### Plan a path (V3 planning mode)

`sim plan` runs a one-shot, RNG-free global plan: load the map, inflate
obstacles, run A\*, write `path.csv`, and show the result in Rerun.

```bash
# Plan across the two-room office map (opens the Rerun Viewer, writes data/path.csv)
./build/clang18-debug/sim plan --map maps/office.yaml --start 0.15,0.15 --goal 1.85,1.35 \
    --config config/planner.yaml

# Override individual planner.yaml fields from the command line
./build/clang18-debug/sim plan --map maps/maze.yaml --goal 0.95,0.95 --heuristic euclidean

# Manhattan is only admissible on a 4-connected grid; pairing it with 8 is rejected
./build/clang18-debug/sim plan --map maps/office.yaml --goal 1.85,1.35 \
    --heuristic manhattan --connectivity 4

# Headless: only writes the byte-deterministic path.csv
./build/clang18-debug/sim plan --map maps/office.yaml --goal 1.85,1.35 --no-viz --out data/path.csv

# Post-process the A* staircase (V4): true endpoints + line-of-sight shortcuts.
# On office500 this turns 524 waypoints / 34.37 m into 13 waypoints / 33.33 m.
./build/clang18-debug/sim plan --map maps/office500.yaml --start 1.175,1.175 \
    --goal 23.875,23.875 --config config/planner.yaml --smooth

# Real-scale floor plan for closed-loop experiments, inflated by the robot footprint
./build/clang18-debug/sim plan --map maps/apartment.yaml --start 2.9,1.6 --goal 7.2,5.6 \
    --inflation-radius 0.25 --smooth
```

`maps/apartment` is a 10 m × 7 m floor plan (rooms, 0.8–0.9 m doors, furniture)
generated from the rectangle list in `maps/src/apartment.toml` by
`python scripts/v4/gen_floorplan.py maps/src/apartment.toml`. Unlike the
hand-drawn demo maps, it stays connected once inflated by the robot footprint.
The current layout is a placeholder; editing the TOML with the real room's
measurements and regenerating swaps it out.

### Drive a path (V4 closed loop)

`sim nav` plans from the EKF's initial estimate, smooths the path, and tracks
it with Regulated Pure Pursuit at 20 Hz on a 100 Hz plant with actuator
saturation and lag, until the robot arrives, collides, stalls or times out.
Parameters come from `config/robot.yaml` and `config/nav.yaml`; the run writes
`nav.csv` (every `SimState` column plus look-ahead, curvature, regime, the
control / estimation / ground-truth error decomposition and clearance) and
shows the loop in Rerun.

```bash
# Hallway -> bedroom 1, turning to face west on arrival (EKF in the loop)
./build/clang18-debug/sim nav --map maps/apartment.yaml --start 0.6,3.6,0 \
    --goal 2.9,5.6,3.1416 --seed 42

# Oracle: ground truth into the controller, so only the control error remains
./build/clang18-debug/sim nav --map maps/apartment.yaml --start 2.9,1.6,1.5708 \
    --goal 7.2,5.6 --controller-input truth --no-viz

# Noise-free plant (encoder quantization only)
./build/clang18-debug/sim nav --map maps/apartment.yaml --start 2.9,1.6,1.5708 \
    --goal 7.2,5.6 --preset none --no-viz

# Follow a given path instead of planning one (Nav2 FollowPath); no map needed
./build/clang18-debug/sim nav --path tests/nav/l_path.csv --preset none \
    --controller-input truth --no-viz
```

When planning, `--map` and `--goal` are required and `--start` defaults to the
grid center (all in world meters). `--path` takes a `path.csv` as written by
`sim plan`, or any CSV with `x,y` columns: the goal is its last waypoint, the
start defaults to its first, and `--map` becomes optional (it only enables the
collision and clearance checks). Take timing numbers from the Release build
(`cmake --preset clang18-release && cmake --build --preset build-release -j`).

### Generate the V2 EKF figures

```bash
source .venv/bin/activate

# Per-run diagnostics from the two runs above: three-trajectory overlay,
# cumulative RMSE, NIS consistency, 3σ state-error envelopes, bias learning.
# The script is mode-aware (reads the CSV's `# mode` header).
python scripts/v2/analyze_ekf.py --input data/traj.csv --ekf-no-bias data/traj_nobias.csv
#   -> fusion_trajectory.png, fusion_rmse_over_time.png, nis_consistency.png,
#      state_errors.png, bias_learning.png

# 3σ position-covariance ellipse evolution (static + geometry + animated GIF)
python scripts/v2/analyze_covariance.py --input data/traj.csv
#   -> covariance_ellipses.png, covariance_geometry.png, covariance_evolution.gif

# RK4-vs-Euler attribution, single seed pair (consumes the two --out files above)
python scripts/v2/analyze_integrator.py --euler data/traj_euler.csv --rk4 data/traj_rk4.csv
#   -> integrator_rmse.png

# RK4-vs-Euler attribution, multi-seed average — drives sim itself per seed
python scripts/v2/sweep_integrator.py --n-seeds 30 --preset default
#   -> integrator_sweep.png
```

All figures land in `results/v2/` (override with `--output`). Every
`traj.csv` embeds `seed` / `preset` / `integrator` / `q_scale` /
`r_scale` / `bias` in its header comments, so any run replays exactly —
and `analyze_integrator.py` / `sweep_integrator.py` rely on the fact that
the EKF consumes no RNG, so an Euler and an RK4 run at the same seed share
a bit-identical truth and measurement stream.

### Generate the V3 planning figures

```bash
source .venv/bin/activate

# Map + inflation layer + A* path for one plan (reads the path.csv written above)
python scripts/v3/plot_plan.py --map maps/office.yaml --path data/path.csv
#   -> plan_office.png

# Timing benchmark on random N×N maps — drives the Release sim itself
python scripts/v3/benchmark_planner.py --sizes 50 100 200 --trials 12
#   -> planner_timing.png

# A* path length vs a Dijkstra ground truth
python scripts/v3/optimality_check.py --maps maps/office.yaml maps/maze.yaml maps/room.yaml \
    --goal-of office=1.85,1.35 maze=0.95,0.95 room=0.85,0.65 --start auto
#   -> optimality.png

# Animated A* expansion (the header GIF)
python scripts/v3/animate_search.py --map maps/office500.yaml --start 1.175,1.175 \
    --goal 23.875,23.875 --inflation-radius 0.05 --max-frames 160 --fps 24
#   -> search_office500.gif
```

All figures land in `results/v3/`, named after the map. Every `path.csv`
embeds the map, start, goal, heuristic, connectivity, inflation radius,
success flag, expanded-node count, and path length in its header comments.

### Generate the V4 control figures

```bash
cmake --build --preset build-release -j   # the scripts drive build/clang18-release/sim
source .venv/bin/activate

python scripts/v4/scenarios.py        # S1–S5 scenario list            -> scenarios.png
python scripts/v4/step_response.py    # E1: linearization and lag theory -> e1_*.png
python scripts/v4/corner_cutting.py   # E2: corner cutting and margins -> e2_*.png
python scripts/v4/run_scenarios.py    # E3/E4 batch runs (~1 min) into data/v4/
python scripts/v4/tracking_error.py   # E3/E4: trade-off, error split, drift -> e3_*.png, e4_*.png
python scripts/v4/animate_nav.py      # closed-loop animation (the header GIF) -> nav_s3.gif
```

Figures land in `results/v4/`; the raw runs stay in the gitignored `data/v4/`.
The scripts change parameters by writing variants of `config/robot.yaml` and
`config/nav.yaml`, so every run starts from the repository defaults.

### Reproducing earlier milestones (V0, V1)

V0 (kinematics scaffold) and V1 (noise + wheel-odometry drift) live at git
tags `v0.1.0` and `v0.2.0` — check one out to build and run its simulation,
or read the retrospectives in `docs/`. The trunk carries only the current
`sim`, with its `ekf`, `plan` and `nav` subcommands; V2, V3 and V4 are tagged
as `v0.3.0`, `v0.4.0` and `v0.5.0`.

---

## Visualization

The Rerun Viewer is the primary live-development surface. Below is a V1 run
replayed in its 3D view — command path (green), ground truth (blue), and
wheel-odometry estimate (orange) starting together and drifting apart over
20 s. Locally, you can scrub, pause, change the camera, toggle individual
entities on/off, and inspect the underlying time series.

<p align="center">
  <img src="results/v1/three_trajectories.gif" alt="MiniNav V1 — three-trajectory 3D replay in Rerun" width="720"/>
</p>

Entity paths logged by `sim`:

| Entity path                   | Meaning                                               |
|-------------------------------|-------------------------------------------------------|
| `/world/robot/cmd_traj/trail` | Where a perfect executor would go (noise-free reference) |
| `/world/robot/truth/trail`    | Actual ground truth (after actuator noise)            |
| `/world/robot/odom/trail`     | Wheel-odometry estimate (after the full sensor chain) |

Additional scalar time series are logged for diagnostics:
`cmd_v` / `cmd_w`, `true_velocity_v` / `true_velocity_w`, encoder
`dticks_l` / `dticks_r`, and direct `error/position` + `error/yaw`
channels.

On top of the command / truth / odom surface above, `sim` adds the EKF
channels:

| Entity path               | Meaning                                                       |
|---------------------------|---------------------------------------------------------------|
| `/world/robot/ekf`        | EKF fused pose estimate (with its trail at `/world/trails/ekf`) |
| `/plots/bias_omega/ekf`   | Online gyro-bias *estimate* `b_ω` over time                   |
| `/plots/bias_omega/truth` | The simulator's *true* gyro bias, for direct comparison       |

The `bias_omega` pair is the most legible demonstration in the sim: in the
Rerun time-series view you watch `b_ω` start at 0 and converge toward the
true bias within a few seconds — the payoff of the state augmentation,
and (at `high-noise`) the place where you can watch it fail to settle.

In planning mode (`sim plan`), the whole scene is logged once as static
data:

| Entity path                  | Meaning                                         |
|------------------------------|-------------------------------------------------|
| `/world/map`                 | Occupied cells of the original map              |
| `/world/map/inflated`        | Cells added by inflation (the safety margin)    |
| `/world/plan/path`           | The A\* path as a line strip, plus `/waypoints` |
| `/world/robot/start`         | Start pose                                      |
| `/world/robot/goal`          | Goal pose                                       |

In closed-loop mode (`sim nav`), the planning scene is logged once and every
step adds the V2 robot channels plus:

| Entity path                               | Meaning                                                   |
|-------------------------------------------|-----------------------------------------------------------|
| `/world/plan/raw`                         | The raw A\* staircase (the smoothed path is `/world/plan/path`) |
| `/world/control/lookahead`                | The controller's look-ahead point                         |
| `/world/control/arc`                      | The pursuit arc from the controller's input pose to it    |
| `/world/estimate/ekf_cov`                 | EKF 3σ position ellipse                                   |
| `/world/robot/actuator/{v,w}`             | Actuator output after saturation and lag                  |
| `/plots/error/{ctrl,est,true}`            | Control, estimation and true error                        |
| `/plots/clearance`, `/plots/regime`       | True clearance to obstacles, controller regime            |

World-frame geometry lives under `/world/control` and `/world/estimate`, never
under a pose entity that carries a `Transform3D`, so Rerun does not transform it
twice.

---

## Documentation

Per-version retrospectives and design notes live under `docs/`:

- [`docs/project_overview.md`](docs/project_overview.md) — full vision, V0 → V6 roadmap, technology choices
- [`docs/v0_summary.md`](docs/v0_summary.md) — V0 retrospective: scaffolding design, alternatives, lessons learned
- [`docs/v1_summary.md`](docs/v1_summary.md) — V1 retrospective: noise modelling, encoder physics, RNG design, drift analysis
- [`docs/v2_summary.md`](docs/v2_summary.md) — V2 retrospective: 6-state EKF design, sensors-as-observations, the bias-estimation operating envelope, RK4 process model, NIS diagnostics
- [`docs/v3_summary.md`](docs/v3_summary.md) — V3 retrospective: occupancy-grid and configuration-space design, the heuristic–connectivity admissibility rule, the timing-test lesson, technical debt toward V4
- [`docs/v4_summary.md`](docs/v4_summary.md) — V4 retrospective: plant / estimator split, `robot.yaml`, the RPP subset behind `nav2_core`-shaped interfaces, path smoothing, the golden regression guard, and what the drift data means for V5 / V6
- [`docs/experiments/v2_ekf_fusion.md`](docs/experiments/v2_ekf_fusion.md) — V2 experiment report: 20-seed EKF-vs-odom RMSE study, the bias-estimation operating envelope, NIS consistency, covariance/observability analysis
- [`docs/experiments/v3_planning.md`](docs/experiments/v3_planning.md) — V3 experiment report: timing benchmark (200×200 and a 500×500 floor plan), optimality vs Dijkstra, heuristic admissibility, determinism
- [`docs/experiments/v4_control.md`](docs/experiments/v4_control.md) — V4 experiment report: Pure Pursuit theory vs simulation, corner cutting and safety margins, the look-ahead trade-off, the control vs localization error split and drift over distance

Mathematical derivations live under `docs/math/`:

- [`docs/math/EKF_Foundations.md`](docs/math/EKF_Foundations.md) — EKF predict/update, Jacobians, Joseph-form covariance
- [`docs/math/runge_kutta_integration.md`](docs/math/runge_kutta_integration.md) — RK4 process integration and its analytic Jacobian
- [`docs/math/odom_noise.md`](docs/math/odom_noise.md) — velocity-motion-model noise, the basis for `Q` and `R`
- [`docs/math/astar_planning.md`](docs/math/astar_planning.md) — occupancy grids, configuration-space inflation, the A\* optimality proof, heuristic admissibility and consistency under 4/8-connectivity
- [`docs/math/pure_pursuit.md`](docs/math/pure_pursuit.md) — Pure Pursuit geometry, linearization, the lag stability bound, corner cutting, the safety-margin budget, and the error decomposition

---

## License

MIT — see [`LICENSE`](LICENSE).
