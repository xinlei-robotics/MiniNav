#!/usr/bin/env python3
"""
scenarios.py — V4 闭环实验的场景清单(E3 / E4,docs/experiments/v4_control.md §2)。

S1–S4 在 maps/apartment(10 × 7 m 占位平面图,0.25 m 膨胀下连通)上,S5 是 V3 的
office500(25 × 25 m)长距离场景,用来看定位漂移随路程的增长。起点给出朝向;目标
不给朝向(到达只看 xy 容差)。

直接运行时打印清单,并画出五个场景的平滑路径(results/v4/scenarios.png)。

Run:
    python scripts/v4/scenarios.py
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Scenario:
    key: str
    title: str
    map_yaml: str
    start: str  # "x,y,yaw"
    goal: str   # "x,y"

    def nav_args(self) -> dict:
        return {"map_yaml": self.map_yaml, "start": self.start, "goal": self.goal}


SCENARIOS: tuple[Scenario, ...] = (
    Scenario("S1", "straight hallway", "maps/apartment.yaml", "0.6,3.6,0", "9.4,3.6"),
    Scenario("S2", "hallway -> door -> bedroom 1", "maps/apartment.yaml", "0.6,3.6,0", "2.9,5.6"),
    Scenario("S3", "living room -> bedroom 2 (two doors)", "maps/apartment.yaml", "2.9,1.6,1.5708", "7.2,5.6"),
    Scenario("S4", "kitchen island, narrow aisle", "maps/apartment.yaml", "6.2,1.1,0", "9.4,1.1"),
    Scenario("S5", "office500 long run", "maps/office500.yaml", "1.175,1.175,0", "23.875,23.875"),
)

BY_KEY = {s.key: s for s in SCENARIOS}


def main() -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    from _navio import DATA_DIR, RESULTS_DIR, REPO, find_sim, load_grid, plan_path

    sim = find_sim()
    fig, (ax_a, ax_o) = plt.subplots(1, 2, figsize=(13, 5.2), gridspec_kw={"width_ratios": [10, 6.2]})
    colors = {"S1": "#1f77b4", "S2": "#ff7f0e", "S3": "#2ca02c", "S4": "#d62728", "S5": "#9467bd"}
    for ax, map_yaml in ((ax_a, "maps/apartment.yaml"), (ax_o, "maps/office500.yaml")):
        grid = load_grid(REPO / map_yaml)
        img = np.where(grid.occ == 100, 0.15, 1.0)
        extent = (grid.origin[0], grid.origin[0] + grid.width * grid.resolution,
                  grid.origin[1], grid.origin[1] + grid.height * grid.resolution)
        ax.imshow(img, cmap="gray", vmin=0, vmax=1, origin="lower", extent=extent, interpolation="nearest")
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
    for sc in SCENARIOS:
        path = plan_path(sim, sc.map_yaml, sc.start, sc.goal, DATA_DIR / "paths" / f"{sc.key.lower()}.csv")
        length = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))
        print(f"{sc.key}: {sc.title:40s} {sc.map_yaml:22s} start={sc.start:16s} goal={sc.goal:13s} "
              f"smoothed {len(path)} waypoints / {length:.2f} m")
        ax = ax_o if "office500" in sc.map_yaml else ax_a
        ax.plot(path[:, 0], path[:, 1], "-", color=colors[sc.key], lw=2.0, label=f"{sc.key} ({length:.1f} m)")
        ax.plot(path[0, 0], path[0, 1], "o", color=colors[sc.key], ms=6)
        ax.plot(path[-1, 0], path[-1, 1], "*", color=colors[sc.key], ms=11)
    ax_a.set_title("apartment (10 × 7 m): S1–S4")
    ax_o.set_title("office500 (25 × 25 m): S5")
    for ax in (ax_a, ax_o):
        ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
    fig.suptitle("MiniNav V4 — closed-loop scenarios (smoothed A* paths; ● start, ★ goal)")
    fig.tight_layout()
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    out = RESULTS_DIR / "scenarios.png"
    fig.savefig(out, dpi=130)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
