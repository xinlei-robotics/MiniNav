#!/usr/bin/env python3
"""
plot_plan.py — 把一次全局规划画成发表用静态图(占据栅格 + 膨胀层 + A* 路径)。

读 map.yaml(+ 同目录 PGM)与规划器写出的 path.csv,叠加渲染:
  - 占据 cell(深灰方块)
  - 膨胀层(浅灰,机器人半径 + 安全裕度;在 Python 端按 path.csv 头里的
    inflation_radius 复算,与 C++ 的欧氏膨胀近似一致,仅用于展示安全裕度)
  - A* 路径折线 + 起点(绿)/ 目标(红)

产出:results/v3/plan_<map>.png(文件名随地图 stem,与 search_<map>.gif 同一约定,
      多张地图的出图不会互相覆盖)

Run:
    ./build/clang18-debug/sim plan --map maps/office.yaml --start 0.15,0.15 \\
        --goal 1.85,1.35 --no-viz --out data/path.csv
    python scripts/v3/plot_plan.py --map maps/office.yaml --path data/path.csv
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from _mapio import Grid, K_OCCUPIED, load_grid, read_path_csv


def inflate_mask(grid: Grid, radius_m: float) -> np.ndarray:
    """复算膨胀:到最近障碍欧氏距离 ≤ radius 的 cell(含障碍本身)。仅用于可视化。"""
    obst = grid.occ == K_OCCUPIED
    if radius_m <= 0.0:
        return obst.copy()
    r_cells = radius_m / grid.resolution
    r = int(math.ceil(r_cells))
    offs = [
        (dx, dy)
        for dy in range(-r, r + 1)
        for dx in range(-r, r + 1)
        if dx * dx + dy * dy <= r_cells * r_cells
    ]
    inflated = obst.copy()
    ys, xs = np.where(obst)
    for cy, cx in zip(ys, xs):
        for dx, dy in offs:
            ny, nx = cy + dy, cx + dx
            if 0 <= ny < grid.height and 0 <= nx < grid.width:
                inflated[ny, nx] = True
    return inflated


def draw_cells(ax, grid: Grid, mask: np.ndarray, color: str, alpha: float) -> None:
    res = grid.resolution
    ys, xs = np.where(mask)
    for gy, gx in zip(ys, xs):
        wx, wy = grid.grid_to_world(int(gx), int(gy))
        ax.add_patch(
            Rectangle(
                (wx - res / 2, wy - res / 2), res, res,
                facecolor=color, edgecolor="none", alpha=alpha,
            )
        )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--map", required=True, type=Path, help="map.yaml")
    ap.add_argument("--path", required=True, type=Path, help="path.csv from sim")
    ap.add_argument("--output", type=Path, default=Path("results/v3"))
    args = ap.parse_args()

    grid = load_grid(args.map)
    meta, waypoints = read_path_csv(args.path)
    radius = float(meta.get("inflation_radius", 0.0))

    fig, ax = plt.subplots(figsize=(8, 6))
    obst = grid.occ == K_OCCUPIED
    inflated = inflate_mask(grid, radius)
    margin = inflated & ~obst  # 膨胀新增的安全裕度

    draw_cells(ax, grid, margin, color="#b0b0b8", alpha=0.7)
    draw_cells(ax, grid, obst, color="#3c3c46", alpha=1.0)

    if waypoints.shape[0] > 0:
        ax.plot(waypoints[:, 0], waypoints[:, 1], "-", color="#28c85a",
                lw=2.0, label="A* path", zorder=3)
        sx, sy = waypoints[0, 0], waypoints[0, 1]
        gx, gy = waypoints[-1, 0], waypoints[-1, 1]
        ax.plot([sx], [sy], "o", color="#28c85a", ms=10, label="start", zorder=4)
        ax.plot([gx], [gy], "*", color="#e64646", ms=16, label="goal", zorder=4)

    ox, oy = grid.origin
    ax.set_xlim(ox, ox + grid.width * grid.resolution)
    ax.set_ylim(oy, oy + grid.height * grid.resolution)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    title = (
        f"Global plan: {Path(meta.get('map', args.map.name)).name}  "
        f"({meta.get('heuristic', '?')}, {meta.get('connectivity', '?')}-conn, "
        f"r={radius} m)\n"
        f"length = {meta.get('path_length_m', '?')} m, "
        f"expanded = {meta.get('expanded_nodes', '?')} nodes"
    )
    ax.set_title(title, fontsize=10)
    ax.legend(loc="upper left", fontsize=9)
    ax.grid(True, ls=":", alpha=0.3)

    args.output.mkdir(parents=True, exist_ok=True)
    out = args.output / f"plan_{args.map.stem}.png"
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    print(f"wrote {out}")
    print(
        f"  map={meta.get('map')} success={meta.get('success')} "
        f"length={meta.get('path_length_m')} expanded={meta.get('expanded_nodes')}"
    )


if __name__ == "__main__":
    main()
