#!/usr/bin/env python3
"""
optimality_check.py — A* 路径长度 vs Dijkstra ground-truth 偏差。

对每张测试地图,在 Python 端复刻 C++ A* 的栅格、步代价(直走 1 / 对角 √2)、
8 连通防穿角规则,跑一遍 **Dijkstra**(无启发,保证最优)拿到 ground-truth
最短路长度;再驱动 `sim` 拿 A* 的实际路径长度,比较二者偏差。

里程碑指标(v3_plan §0.3):偏差 ≤ 1 个 cell(= 1·resolution)。

产出:
  - results/v3/optimality.png   各地图 A* 与 Dijkstra 长度对比 + 偏差(cell)
  - stdout 表格

Run:
    python scripts/v3/optimality_check.py \\
        --maps maps/office.yaml maps/maze.yaml maps/room.yaml \\
        --goal-of office=1.85,1.35 --goal-of maze=0.95,0.95 --goal-of room=0.85,0.65
"""

from __future__ import annotations

import argparse
import heapq
import math
import subprocess
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from _mapio import Grid, K_OCCUPIED, K_UNKNOWN, load_grid, read_path_csv

SQRT2 = math.sqrt(2.0)


def traversable(grid: Grid, gx: int, gy: int, allow_unknown: bool) -> bool:
    if not grid.in_bounds(gx, gy):
        return False  # 越界视为占据
    v = grid.occ[gy, gx]
    if v == K_OCCUPIED:
        return False
    if v == K_UNKNOWN:
        return allow_unknown
    return True


def dijkstra_length(grid: Grid, start, goal, connectivity: int,
                    allow_unknown: bool) -> float | None:
    """ground-truth 最短长度(world 米)。无路径返回 None。"""
    sx, sy = grid.world_to_grid(*start)
    gx, gy = grid.world_to_grid(*goal)
    if not traversable(grid, sx, sy, allow_unknown):
        return None
    if not traversable(grid, gx, gy, allow_unknown):
        return None

    nbrs = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    if connectivity == 8:
        nbrs += [(1, 1), (1, -1), (-1, 1), (-1, -1)]

    W, H = grid.width, grid.height
    dist = np.full((H, W), math.inf)
    dist[sy, sx] = 0.0
    pq: list[tuple[float, int, int]] = [(0.0, sx, sy)]
    while pq:
        d, cx, cy = heapq.heappop(pq)
        if d > dist[cy, cx]:
            continue
        if (cx, cy) == (gx, gy):
            return d * grid.resolution  # cell 数 → 米
        for dx, dy in nbrs:
            nx, ny = cx + dx, cy + dy
            if not traversable(grid, nx, ny, allow_unknown):
                continue
            if dx != 0 and dy != 0:  # 防穿角
                if not traversable(grid, cx + dx, cy, allow_unknown):
                    continue
                if not traversable(grid, cx, cy + dy, allow_unknown):
                    continue
            step = SQRT2 if (dx != 0 and dy != 0) else 1.0
            nd = d + step
            if nd < dist[ny, nx]:
                dist[ny, nx] = nd
                heapq.heappush(pq, (nd, nx, ny))
    return None


def run_astar(sim_bin: Path, map_yaml: Path, start: str, goal: str,
              connectivity: int, out_csv: Path) -> dict:
    cmd = [
        str(sim_bin), "--map", str(map_yaml), "--start", start, "--goal", goal,
        "--connectivity", str(connectivity), "--inflation-radius", "0.0",
        "--no-viz", "--out", str(out_csv),
    ]
    subprocess.run(cmd, check=True, capture_output=True, text=True)
    meta, _ = read_path_csv(out_csv)
    return meta


def parse_goal_of(items: list[str]) -> dict[str, str]:
    out: dict[str, str] = {}
    for item in items:
        name, coord = item.split("=", 1)
        out[name] = coord
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--maps", nargs="+", type=Path, required=True)
    ap.add_argument("--goal-of", nargs="*", default=[], dest="goal_of",
                    help="per-map goal as name=x,y (name = map stem)")
    ap.add_argument("--start", default="auto",
                    help="world start \"x,y\" or 'auto' (grid center)")
    ap.add_argument("--connectivity", type=int, default=8, choices=(4, 8))
    ap.add_argument("--sim-bin", type=Path,
                    default=Path("build/clang18-release/sim"))
    ap.add_argument("--output", type=Path, default=Path("results/v3"))
    args = ap.parse_args()

    sim_bin = args.sim_bin
    if not sim_bin.exists():
        sim_bin = Path("build/clang18-debug/sim")
    goal_of = parse_goal_of(args.goal_of)
    args.output.mkdir(parents=True, exist_ok=True)
    tmp = args.output / "_opt_tmp.csv"

    names, astar_len, dijkstra_len, dev_cells = [], [], [], []
    print(f"{'map':<12}{'A* [m]':>10}{'Dijkstra [m]':>14}{'dev [cell]':>12}  ok")
    print("-" * 60)
    for map_yaml in args.maps:
        grid = load_grid(map_yaml)
        stem = map_yaml.stem
        if args.start == "auto":
            cx = grid.origin[0] + grid.width * grid.resolution * 0.5
            cy = grid.origin[1] + grid.height * grid.resolution * 0.5
            start = f"{cx},{cy}"
        else:
            start = args.start
        goal = goal_of.get(stem)
        if goal is None:
            print(f"{stem:<12}  (no --goal-of {stem}=x,y; skipped)")
            continue

        meta = run_astar(sim_bin, map_yaml, start, goal, args.connectivity, tmp)
        a_len = float(meta["path_length_m"]) if meta.get("success") == "1" else math.nan

        s = tuple(float(v) for v in start.split(","))
        g = tuple(float(v) for v in goal.split(","))
        d_len = dijkstra_length(grid, s, g, args.connectivity, allow_unknown=False)
        d_len = math.nan if d_len is None else d_len

        dev = abs(a_len - d_len) / grid.resolution if not (
            math.isnan(a_len) or math.isnan(d_len)) else math.nan
        ok = "Y" if (not math.isnan(dev) and dev <= 1.0) else "N"

        names.append(stem)
        astar_len.append(a_len)
        dijkstra_len.append(d_len)
        dev_cells.append(dev)
        print(f"{stem:<12}{a_len:>10.4f}{d_len:>14.4f}{dev:>12.4f}   {ok}")

    if tmp.exists():
        tmp.unlink()

    # 柱状对比图
    if names:
        x = np.arange(len(names))
        fig, ax = plt.subplots(figsize=(1.6 * len(names) + 3, 5))
        ax.bar(x - 0.2, astar_len, 0.4, label="A* (sim)", color="#28c85a")
        ax.bar(x + 0.2, dijkstra_len, 0.4, label="Dijkstra (truth)", color="#4682dc")
        for i, dev in enumerate(dev_cells):
            ax.annotate(f"Δ={dev:.2f} cell", (x[i], max(astar_len[i], dijkstra_len[i])),
                        textcoords="offset points", xytext=(0, 4),
                        ha="center", fontsize=8)
        ax.set_xticks(x)
        ax.set_xticklabels(names)
        ax.set_ylabel("path length [m]")
        ax.set_title(f"A* optimality vs Dijkstra ground-truth "
                     f"({args.connectivity}-connected)\n"
                     f"milestone: deviation ≤ 1 cell")
        ax.legend()
        ax.grid(True, axis="y", ls=":", alpha=0.4)
        out = args.output / "optimality.png"
        fig.tight_layout()
        fig.savefig(out, dpi=140)
        print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
