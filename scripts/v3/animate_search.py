#!/usr/bin/env python3
"""
animate_search.py — 把 A* 的寻路过程渲染成动图(扩展波纹 + 最终路径)。

A* 在地图上像波纹一样从起点铺开、被障碍绕过、最终收束到目标 —— 这是 V3 最
直观的"算法在干什么"演示。为拿到逐步的扩展顺序(C++ `PlanResult` 只给计数),
本脚本在 Python 端按**与 C++ `mininav::planning::AStarPlanner` 一致的规则**
(步代价 直走 1 / 对角 √2、Octile/Euclidean/Manhattan 启发、8 连通防穿角、
tie-break 偏向小 h)重跑一遍 A*,记录扩展序后用 matplotlib 动画导出 GIF。

帧渲染用单张 imshow 缓冲(增量上色,O(扩展数)),因此 500×500 的大图也能在
有限帧数内出图(自动按 --max-frames 算跨步)。

产出:results/v3/search_<map>.gif

Run:
    python scripts/v3/animate_search.py --map maps/maze.yaml --start auto --goal 0.95,0.95
    python scripts/v3/animate_search.py --map maps/office500.yaml \\
        --start 1.175,1.175 --goal 23.875,23.875 --inflation-radius 0.05 --max-frames 150
"""

from __future__ import annotations

import argparse
import heapq
import math
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import animation

from _mapio import Grid, K_OCCUPIED, K_UNKNOWN, inflate, load_grid

SQRT2 = math.sqrt(2.0)


def _heuristic(name: str, ax: int, ay: int, bx: int, by: int) -> float:
    dx, dy = abs(ax - bx), abs(ay - by)
    if name == "manhattan":
        return dx + dy
    if name == "euclidean":
        return math.hypot(dx, dy)
    return (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)  # octile


def astar_trace(occ: np.ndarray, start, goal, connectivity: int, heuristic: str,
                allow_unknown: bool):
    """返回 (expanded[(gx,gy)...] 按扩展序, path[(gx,gy)...] 或 None)。

    occ 为(已膨胀的)三值占据数组,gy-major(gy=0 在底)。start/goal 为 grid 坐标。"""
    H, W = occ.shape

    def passable(gx: int, gy: int) -> bool:
        if not (0 <= gx < W and 0 <= gy < H):
            return False
        v = occ[gy, gx]
        if v == K_OCCUPIED:
            return False
        if v == K_UNKNOWN:
            return allow_unknown
        return True

    sx, sy = start
    gx, gy = goal
    if not passable(sx, sy) or not passable(gx, gy):
        return [], None

    nbrs = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    if connectivity == 8:
        nbrs += [(1, 1), (1, -1), (-1, 1), (-1, -1)]

    g_cost = np.full((H, W), math.inf)
    parent = np.full((H, W, 2), -1, dtype=int)
    closed = np.zeros((H, W), dtype=bool)
    g_cost[sy, sx] = 0.0
    h0 = _heuristic(heuristic, sx, sy, gx, gy)
    pq: list[tuple[float, float, int, int]] = [(h0, h0, sx, sy)]  # (f, h, x, y)

    expanded: list[tuple[int, int]] = []
    found = False
    while pq:
        _, _, cx, cy = heapq.heappop(pq)
        if closed[cy, cx]:
            continue
        closed[cy, cx] = True
        expanded.append((cx, cy))
        if (cx, cy) == (gx, gy):
            found = True
            break
        for dx, dy in nbrs:
            nx, ny = cx + dx, cy + dy
            if not passable(nx, ny) or closed[ny, nx]:
                continue
            if dx != 0 and dy != 0:  # 防穿角
                if not passable(cx + dx, cy) or not passable(cx, cy + dy):
                    continue
            step = SQRT2 if (dx != 0 and dy != 0) else 1.0
            tentative = g_cost[cy, cx] + step
            if tentative < g_cost[ny, nx]:
                g_cost[ny, nx] = tentative
                parent[ny, nx] = (cx, cy)
                hn = _heuristic(heuristic, nx, ny, gx, gy)
                heapq.heappush(pq, (tentative + hn, hn, nx, ny))

    if not found:
        return expanded, None

    path: list[tuple[int, int]] = []
    cur = (gx, gy)
    while cur != (-1, -1):
        path.append(cur)
        px, py = parent[cur[1], cur[0]]
        cur = (int(px), int(py))
    path.reverse()
    return expanded, path


def base_image(grid: Grid, infl: np.ndarray) -> np.ndarray:
    """RGB 底图(gy-major):free 白 / 膨胀 浅灰 / 障碍 深灰 / unknown 米色。"""
    H, W = grid.occ.shape
    img = np.full((H, W, 3), 245, dtype=np.uint8)
    margin = (infl == K_OCCUPIED) & (grid.occ != K_OCCUPIED)
    img[margin] = (200, 200, 212)
    img[grid.occ == K_UNKNOWN] = (196, 196, 150)
    img[grid.occ == K_OCCUPIED] = (60, 60, 70)
    return img


def parse_xy(s: str, grid: Grid) -> tuple[int, int]:
    if s == "auto":
        wx = grid.origin[0] + grid.width * grid.resolution * 0.5
        wy = grid.origin[1] + grid.height * grid.resolution * 0.5
    else:
        wx, wy = (float(v) for v in s.split(","))
    return grid.world_to_grid(wx, wy)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--map", required=True, type=Path)
    ap.add_argument("--start", default="auto", help='"x,y" world meters, or auto (center)')
    ap.add_argument("--goal", required=True, help='"x,y" world meters')
    ap.add_argument("--heuristic", default="octile",
                    choices=("manhattan", "euclidean", "octile"))
    ap.add_argument("--connectivity", type=int, default=8, choices=(4, 8))
    ap.add_argument("--inflation-radius", type=float, default=0.0)
    ap.add_argument("--allow-unknown", action="store_true")
    ap.add_argument("--max-frames", type=int, default=140,
                    help="扩展阶段的最大帧数(大图按此算跨步)")
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--hold-seconds", type=float, default=2.0,
                    help="末尾停留(展示完整路径)秒数")
    ap.add_argument("--output", type=Path, default=None)
    args = ap.parse_args()
    # 与 C++ AStarPlanner 同一条规则:manhattan 在 8 连通下不 admissible,直接拒绝。
    if args.heuristic == "manhattan" and args.connectivity == 8:
        raise SystemExit("--heuristic manhattan is inadmissible with --connectivity 8; "
                         "use octile/euclidean or --connectivity 4")

    grid = load_grid(args.map)
    infl = inflate(grid, args.inflation_radius)
    start = parse_xy(args.start, grid)
    goal = parse_xy(args.goal, grid)

    expanded, path = astar_trace(infl, start, goal, args.connectivity,
                                 args.heuristic, args.allow_unknown)
    n_exp = len(expanded)
    print(f"map={args.map.name} expanded={n_exp} "
          f"path={'none' if path is None else f'{len(path)} cells'}")
    if n_exp == 0:
        raise SystemExit("start/goal blocked — nothing to animate")

    base = base_image(grid, infl)
    buf = base.copy()
    # 扩展序 → 颜色:plasma 渐变,直观看出"先扩展(紫)→ 后扩展(黄)"的波前。
    ramp = (matplotlib.colormaps["plasma"](np.linspace(0.0, 1.0, n_exp))[:, :3] * 255).astype(np.uint8)

    stride = max(1, math.ceil(n_exp / args.max_frames))
    exp_frames = math.ceil(n_exp / stride)
    hold_frames = max(1, int(args.fps * args.hold_seconds))
    total_frames = exp_frames + hold_frames

    H, W = grid.occ.shape
    ox, oy = grid.origin
    extent = [ox, ox + W * grid.resolution, oy, oy + H * grid.resolution]

    fig, ax = plt.subplots(figsize=(8, 8 * H / W + 0.6))
    im = ax.imshow(buf, origin="lower", extent=extent, interpolation="nearest")
    (path_line,) = ax.plot([], [], "-", color="#19e65a", lw=2.2, zorder=5)
    sx_w, sy_w = grid.grid_to_world(*start)
    gx_w, gy_w = grid.grid_to_world(*goal)
    ax.plot([sx_w], [sy_w], "o", color="#19e65a", ms=11, mec="k", zorder=6)
    ax.plot([gx_w], [gy_w], "*", color="#e63c3c", ms=18, mec="k", zorder=6)
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    title = ax.set_title("")

    painted = {"k": 0}

    def update(frame: int):
        if frame < exp_frames:
            target = min(n_exp, (frame + 1) * stride)
            for k in range(painted["k"], target):
                gx, gy = expanded[k]
                if buf[gy, gx, 0] > 120 or (buf[gy, gx] == base[gy, gx]).all():
                    buf[gy, gx] = ramp[k]  # 只覆盖 free/膨胀,不盖障碍
            painted["k"] = target
            im.set_data(buf)
            title.set_text(f"A* search — {args.map.stem}  "
                           f"({args.heuristic}, {args.connectivity}-conn)   "
                           f"expanded {target}/{n_exp}")
        else:
            # 扩展全部上色后,画出最终路径(末尾 hold)
            if painted["k"] < n_exp:
                for k in range(painted["k"], n_exp):
                    gx, gy = expanded[k]
                    buf[gy, gx] = ramp[k]
                painted["k"] = n_exp
                im.set_data(buf)
            if path is not None:
                wx = [grid.grid_to_world(c[0], c[1])[0] for c in path]
                wy = [grid.grid_to_world(c[0], c[1])[1] for c in path]
                path_line.set_data(wx, wy)
                title.set_text(f"A* path — {args.map.stem}   "
                               f"expanded {n_exp}, path {len(path)} cells")
            else:
                title.set_text(f"A* — {args.map.stem}: no path (unreachable)")
        return [im, path_line, title]

    anim = animation.FuncAnimation(fig, update, frames=total_frames, interval=1000 / args.fps,
                                   blit=False, repeat=False)

    out = args.output or Path("results/v3") / f"search_{args.map.stem}.gif"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    anim.save(out, writer=animation.PillowWriter(fps=args.fps))
    print(f"wrote {out}  ({total_frames} frames @ {args.fps} fps, stride={stride})")


if __name__ == "__main__":
    main()
