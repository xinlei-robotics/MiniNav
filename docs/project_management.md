# MiniNav Project Management

This document describes how the MiniNav repository tracks work:
how issues are filed, what labels mean, how the project board is
organized, and the conventions around milestones and releases.

It exists for two reasons:

1. **Operational**: a single source of truth for the project's
   own conventions, so future-me does not have to reinvent them.
2. **Transparent**: anyone reading the repo can see the work is
   tracked deliberately, not by accident.

---

## Workflow at a glance

```
Idea → Issue (one of 4 templates)
     → Project board (Backlog / Ready / In Progress / Blocked / In Review / Done)
     → PR (links back to issue via "Closes #N")
     → Merge → issue auto-closes
     → Milestone progress advances
     → On milestone completion: git tag + GitHub Release
```

No issue exists without a milestone. No PR merges without a linked issue
(except for very small infra fixes — typo fixes, badge updates).

---

## Issue templates

Four templates live under `.github/ISSUE_TEMPLATE/`. Blank issues are
disabled — every issue starts from a template.

| Template          | When to use                                                      |
|-------------------|------------------------------------------------------------------|
| **✨ Feature**     | New functionality, algorithm, or module                          |
| **🐛 Bug**        | Defect: incorrect behavior, crash, or regression                 |
| **🧪 Experiment** | Work that produces a measurable result with a numeric threshold  |
| **🧱 Tech debt**  | Known limitation deliberately deferred, with a trigger condition |

The distinction between **Bug** and **Tech debt** is intentional: a bug
is broken behavior we did not intend; tech debt is a limitation we
*chose* to live with, and the issue records the trigger that will make
us revisit the choice.

The distinction between **Feature** and **Experiment** is also
intentional: an experiment has a falsifiable hypothesis and a numeric
acceptance threshold. A feature that "should make the robot navigate
better" is not an experiment until it has a number attached.

---

## Labels

Labels follow three dimensions plus a few special cases. Total label
count is kept under 25 — more labels lower the signal of each.

### `type:*` — work type

| Label              | Meaning                                                 |
|--------------------|---------------------------------------------------------|
| `type: feature`    | New functionality (algorithm / module / API)            |
| `type: bug`        | Defect                                                  |
| `type: refactor`   | Structural change, behavior preserved                   |
| `type: docs`       | Documentation (incl. `docs/math/`, `docs/experiments/`) |
| `type: experiment` | Experiment with numeric acceptance criteria             |
| `type: infra`      | Build system, CI, toolchain, tooling                    |

### `area:*` — code area

Maps roughly 1:1 to CMake targets and the 5-layer architecture.

| Label                | Layer / Target                                      |
|----------------------|-----------------------------------------------------|
| `area: core`         | `mininav_core` — kinematics, types, Trajectory, RNG |
| `area: sensors`      | `mininav_sensors` — actuator, encoder, IMU models; `mininav_simulation` plant (V4+) |
| `area: localization` | `mininav_localization` — odometry, EKF              |
| `area: planning`     | `mininav_planning` (V3+) — A*, occupancy grid       |
| `area: control`      | `mininav_control` (V4+) — controller interfaces, Pure Pursuit |
| `area: viz`          | `mininav_viz` — Rerun integration                   |
| `area: ros2`         | `ros2_ws/*` (V5+) — nodes, Nav2 plugins, bringup    |
| `area: hardware`     | Pi 5 / IMU / motors (V6+)                           |
| `area: scripts`      | Python post-processing                              |
| `area: build`        | CMake, presets, FetchContent setup                  |
| `area: ci`           | GitHub Actions                                      |

### `priority:*`

Three levels: `priority: high`, `priority: medium`, `priority: low`.
For a single-contributor project, finer granularity (P0–P3) does not
add information.

### Special labels

| Label               | Meaning                                                           |
|---------------------|-------------------------------------------------------------------|
| `epic`              | Cross-cutting issue spanning multiple sub-issues; one per version |
| `tech-debt`         | Deferred work with a trigger condition                            |
| `blocked`           | Waiting on a prerequisite issue or external dependency            |
| `stretch-goal`      | Non-mandatory work (e.g. V7 SLAM)                                 |
| `good first commit` | A clean entry point for first-time contributors                   |

---

## Milestones

Milestones map 1:1 to the V0–V7 roadmap. Each milestone's description
embeds the version's quantitative target so the milestone page itself
answers "what does done look like for this version?"

| Milestone                            | Status         | Target metric (excerpt)                                                 |
|--------------------------------------|----------------|-------------------------------------------------------------------------|
| V0 — Simulation Scaffolding          | ✅ closed       | CSV byte-identical across launch modes                                  |
| V1 — Sensors, Noise & Odometry Drift | ✅ closed       | 0.2–0.6 m drift at 20s, default preset; byte-exact seed reproducibility |
| V2 — EKF Sensor Fusion               | ✅ closed       | RMSE reduction ≥ 50% vs odom; Jacobian finite-diff tolerance ≤ 1e-6     |
| V3 — Path Planning                   | ✅ closed       | A* on 200×200 map ≤ 50 ms; path length within 1 cell of shortest        |
| V4 — Closed-Loop Path Tracking       | open (next)    | Control error (estimate-to-path): mean ≤ 10 cm, peak ≤ 30 cm; zero collisions with ground-truth feedback |
| V5 — ROS 2 + Nav2 Integration        | open           | Goal-reach rate ≥ 80% on 5 scenarios (route lengths set from V4 drift data); e2e latency ≤ 100 ms |
| V6 — Real Robot Deployment           | open           | Sim-to-real gap table; Hausdorff distance quantified                    |
| V7 — SLAM Integration                | open (stretch) | Indoor mapping + navigation video                                       |

> **Re-scoped on 2026-09-28.** V4 was originally *Control + ROS 2* and V5
> *Full Simulation Loop*. Control and ROS 2 packaging are independent risks, and
> tuning a controller inside asynchronous, wall-clock ROS 2 makes experiments
> irreproducible. V4 therefore closes the plan–track loop in the deterministic
> C++ simulation, and V5 takes over all ROS 2 / Nav2 work. The GitHub milestones
> and their issues follow the same split.

See [`docs/project_overview.md`](project_overview.md) for the full
version roadmap and rationale.

---

## Project board

A single project — **MiniNav Roadmap** — with four views:

| View                  | Purpose                                                                                              |
|-----------------------|------------------------------------------------------------------------------------------------------|
| 🗺 **Roadmap**        | Board grouped by milestone. The portfolio view: anyone landing here sees V0–V7 progress at a glance. |
| 🎯 **Current Sprint** | Filtered to the active milestone, grouped by status. Daily working view.                             |
| 🧱 **By Layer**       | Table grouped by Layer. Architectural view across all work.                                          |
| 🪣 **Backlog**        | Table filtered to `Backlog` status, sorted by priority. The waiting pool.                            |

Custom fields beyond the GitHub defaults:

- **Layer** — single-select, mirrors the `area:*` labels but at slightly
  coarser grain (Sim / Sensors / Localization / Planning / Control / Viz / Build / Docs / Hardware)
- **Effort** — single-select XS / S / M / L / XL. Used for backlog
  prioritization, not commitment.

---

## Releases & tags

Each completed milestone produces a git tag and a GitHub Release. The
versioning scheme is **0.MAJOR.0** until V6 ships, when the project
reaches 1.0.

| Tag      | Marks completion of                              |
|----------|--------------------------------------------------|
| `v0.1.0` | V0 — Simulation Scaffolding                      |
| `v0.2.0` | V1 — Sensors, Noise & Odometry Drift             |
| `v0.3.0` | V2 — EKF Sensor Fusion                           |
| `v0.4.0` | V3 — Path Planning                               |
| `v0.5.0` | V4 — Closed-Loop Path Tracking                   |
| `v0.6.0` | V5 — ROS 2 + Nav2 Integration                    |
| `v1.0.0` | V6 — Real Robot Deployment (the project's "1.0") |
| `v1.1.0` | V7 — SLAM Integration (if completed)             |

Release notes are auto-generated by GitHub from merged PR titles, then
manually curated to highlight the milestone's key results
(quantitative metrics, demo links).

---

## PR conventions

Every PR template requires:

- A linked issue (`Closes #N` to auto-close on merge)
- A **Quantitative impact** field — even if "N/A", the absence of
  measurable change is itself information
- A checklist that ensures `ctest` passes, CSV regression is empty,
  new APIs have tests, and docs are updated

For trivial changes (typos, badge updates, README fixes), the checklist
items not applicable are simply checked off — the template is a
prompt, not a tax.

---

## Why this structure?

A few decisions are worth stating explicitly:

- **Issues describe actions, docs describe knowledge.** Issues are
  short-lived ("implement X"); the deep "why X is this way" lives in
  `docs/`. They reference each other but do not duplicate.

- **Retrospective issues for V0/V1 are honestly labeled.** Issues
  completed before this management system was set up carry a note in
  their body: "Retrospective issue created after the fact." This is
  more honorable than backdating creation timestamps, and the
  information value (what was actually built) is preserved either way.

- **The label set is small on purpose.** A label that is never used is
  worse than absent — it lowers the signal of every other label.
  Twenty labels are about the right number for a project this size.

- **The board is public.** Closing the loop on the second reason
  this document exists: the management surface is part of the
  portfolio. If you've made it this far, you've already verified
  it.
