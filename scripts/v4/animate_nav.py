#!/usr/bin/env python3
"""
animate_nav.py — 把一次 `sim nav` 闭环运行渲染成 GIF(README 顶部的 V4 演示)。

左:整张地图 —— 占据 / 膨胀层、A* 原始台阶路径(灰)、平滑后路径(绿)、真值轨迹(黑)
与 EKF 估计轨迹(红虚线)。右:跟随机器人的 1.6 m 放大窗 —— 真值车体(矩形底盘 +
外接圆)、控制器看到的估计位姿、look-ahead 点与追踪圆弧(按 nav.csv 的曲率在估计位姿上
画)、EKF 位置 3σ 椭圆。底部文字:时间、速度、工况、e_ctrl / e_est / e_true。

默认场景 S3(客厅 → 两道门 → 卧室 2),seed 1,default 噪声,EKF 进控制器。

产出:results/v4/nav_<scenario>.gif(有 ffmpeg 时用全局调色板重编码,减小体积)

Run:
    python scripts/v4/animate_nav.py                      # S3, seed 1
    python scripts/v4/animate_nav.py --scenario S2 --seed 4 --fps 15 --stride 0.2
"""

from __future__ import annotations

import argparse
import math
import shutil
import subprocess

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import Circle, Ellipse, Polygon

from _navio import (DATA_DIR, REPO, RESULTS_DIR, find_sim, inflate, load_grid, plan_path, read_yaml_subset,
                    run_and_load)
from scenarios import BY_KEY

ZOOM_HALF = 0.8  # m,放大窗半宽


def footprint_polygon(x: float, y: float, yaw: float, length: float, width: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    corners = np.array([[length / 2, width / 2], [-length / 2, width / 2],
                        [-length / 2, -width / 2], [length / 2, -width / 2]])
    rot = np.array([[c, -s], [s, c]])
    return corners @ rot.T + np.array([x, y])


def pursuit_arc(x: float, y: float, yaw: float, kappa: float, length: float, n: int = 30) -> np.ndarray:
    """从 (x, y, yaw) 出发、曲率 κ、弧长 length 的圆弧(world)。"""
    s = np.linspace(0.0, length, n)
    if abs(kappa) < 1e-9:
        local = np.column_stack([s, np.zeros_like(s)])
    else:
        local = np.column_stack([np.sin(kappa * s) / kappa, (1.0 - np.cos(kappa * s)) / kappa])
    c, sn = math.cos(yaw), math.sin(yaw)
    return local @ np.array([[c, sn], [-sn, c]]) + np.array([x, y])


def covariance_ellipse(sxx: float, syy: float, sxy: float, n_sigma: float = 3.0) -> tuple[float, float, float]:
    vals, vecs = np.linalg.eigh(np.array([[sxx, sxy], [sxy, syy]]))
    vals = np.maximum(vals, 0.0)
    angle = math.degrees(math.atan2(vecs[1, 1], vecs[0, 1]))
    return 2 * n_sigma * math.sqrt(vals[1]), 2 * n_sigma * math.sqrt(vals[0]), angle


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="S3", choices=sorted(BY_KEY))
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--preset", default="default")
    ap.add_argument("--stride", type=float, default=0.2, help="simulated seconds per frame")
    ap.add_argument("--fps", type=int, default=12)
    ap.add_argument("--hold-seconds", type=float, default=1.5, help="freeze the last frame this long")
    args = ap.parse_args()

    sc = BY_KEY[args.scenario]
    sim = find_sim()
    meta, df = run_and_load(sim, DATA_DIR / "anim" / f"{sc.key}_seed{args.seed}.csv", **sc.nav_args(),
                            seed=args.seed, preset=args.preset, controller_input="ekf")
    raw = plan_path(sim, sc.map_yaml, sc.start, sc.goal, DATA_DIR / "anim" / f"{sc.key}_raw.csv", smooth=False)
    smooth = plan_path(sim, sc.map_yaml, sc.start, sc.goal, DATA_DIR / "anim" / f"{sc.key}_smooth.csv")
    robot = read_yaml_subset(REPO / "config" / "robot.yaml")
    length, width = robot["footprint"]["length"], robot["footprint"]["width"]
    r_robot = math.hypot(length / 2, width / 2)

    grid = load_grid(REPO / sc.map_yaml)
    inflated = inflate(grid, float(meta["inflation_radius"]))
    img = np.full(grid.occ.shape, 1.0)
    img[inflated == 100] = 0.82
    img[grid.occ == 100] = 0.2
    extent = (grid.origin[0], grid.origin[0] + grid.width * grid.resolution,
              grid.origin[1], grid.origin[1] + grid.height * grid.resolution)

    fig = plt.figure(figsize=(11.0, 5.4))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.55, 1])
    ax_map = fig.add_subplot(gs[0])
    ax_zoom = fig.add_subplot(gs[1])
    for ax in (ax_map, ax_zoom):
        ax.imshow(img, cmap="gray", vmin=0, vmax=1, origin="lower", extent=extent, interpolation="nearest")
        ax.plot(raw[:, 0], raw[:, 1], "-", color="0.55", lw=1.0)
        ax.plot(smooth[:, 0], smooth[:, 1], "-", color="tab:green", lw=1.8)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
    ax_map.set_xlim(extent[0], extent[1])
    ax_map.set_ylim(extent[2], extent[3])
    ax_map.plot(*smooth[-1], "*", color="tab:green", ms=14, mec="black", mew=0.6)
    ax_map.set_title(f"{sc.key}: {sc.title} — {meta['path_length_m']:.1f} m", fontsize=10)
    ax_zoom.set_title("controller view (1.6 m window)", fontsize=10)

    truth_trail, = ax_map.plot([], [], "-", color="black", lw=1.4, label="truth")
    ekf_trail, = ax_map.plot([], [], "--", color="tab:red", lw=1.2, label="EKF estimate")
    ax_map.plot([], [], "-", color="0.55", lw=1.0, label="A* raw path")
    ax_map.plot([], [], "-", color="tab:green", lw=1.8, label="smoothed path")
    robot_dot = Circle((0, 0), r_robot, fill=False, ec="black", lw=1.0)
    ax_map.add_patch(robot_dot)
    ax_map.legend(fontsize=8, loc="lower right", framealpha=0.9)

    z_truth, = ax_zoom.plot([], [], "-", color="black", lw=1.4)
    z_ekf, = ax_zoom.plot([], [], "--", color="tab:red", lw=1.2)
    body = Polygon(np.zeros((4, 2)), closed=True, fc="0.35", ec="black", alpha=0.75)
    ax_zoom.add_patch(body)
    circ = Circle((0, 0), r_robot, fill=False, ec="black", lw=0.8, ls=":")
    ax_zoom.add_patch(circ)
    ell = Ellipse((0, 0), 0, 0, fill=False, ec="tab:red", lw=1.4)
    ax_zoom.add_patch(ell)
    est_dot, = ax_zoom.plot([], [], "o", color="tab:red", ms=4)
    arc, = ax_zoom.plot([], [], "-", color="tab:blue", lw=1.6)
    carrot, = ax_zoom.plot([], [], "o", color="tab:blue", ms=7, mec="white")
    ax_zoom.plot([], [], "-", color="tab:blue", lw=1.6, label="pursuit arc → look-ahead point")
    ax_zoom.plot([], [], "-", color="tab:red", lw=1.4, label="EKF estimate + 3σ ellipse")
    ax_zoom.plot([], [], "s", color="0.35", label="true chassis (22 × 16 cm)")
    ax_zoom.legend(fontsize=7, loc="upper left", framealpha=0.9)
    info = fig.text(0.5, 0.025, "", ha="center", fontsize=9, family="monospace")

    t = df["t"].to_numpy()
    step = max(1, int(round(args.stride / (t[1] - t[0]))))
    frames = list(range(0, len(df), step)) + [len(df) - 1]
    hold = int(args.fps * args.hold_seconds)
    frames += [len(df) - 1] * hold

    tx, ty = df["truth_x"].to_numpy(), df["truth_y"].to_numpy()
    ex, ey = df["ekf_x"].to_numpy(), df["ekf_y"].to_numpy()

    def update(i: int):
        row = df.iloc[i]
        truth_trail.set_data(tx[: i + 1], ty[: i + 1])
        ekf_trail.set_data(ex[: i + 1], ey[: i + 1])
        robot_dot.center = (row.truth_x, row.truth_y)
        z_truth.set_data(tx[: i + 1], ty[: i + 1])
        z_ekf.set_data(ex[: i + 1], ey[: i + 1])
        body.set_xy(footprint_polygon(row.truth_x, row.truth_y, row.truth_yaw, length, width))
        circ.center = (row.truth_x, row.truth_y)
        w, h, ang = covariance_ellipse(row.ekf_sigma_xx, row.ekf_sigma_yy, row.ekf_sigma_xy)
        ell.center = (row.ekf_x, row.ekf_y)
        ell.width, ell.height, ell.angle = w, h, ang
        est_dot.set_data([row.ekf_x], [row.ekf_y])
        d = math.hypot(row.lookahead_x - row.ekf_x, row.lookahead_y - row.ekf_y)
        pts = pursuit_arc(row.ekf_x, row.ekf_y, row.ekf_yaw, row.curvature, d)
        arc.set_data(pts[:, 0], pts[:, 1])
        carrot.set_data([row.lookahead_x], [row.lookahead_y])
        ax_zoom.set_xlim(row.truth_x - ZOOM_HALF, row.truth_x + ZOOM_HALF)
        ax_zoom.set_ylim(row.truth_y - ZOOM_HALF, row.truth_y + ZOOM_HALF)
        info.set_text(f"t = {row.t:5.1f} s   v = {row.true_v:4.2f} m/s   {row.regime:<8s}   "
                      f"e_ctrl = {100 * row.e_ctrl:4.1f} cm   e_est = {100 * row.e_est:4.1f} cm   "
                      f"e_true = {100 * row.e_true:4.1f} cm")
        return truth_trail, ekf_trail, z_truth, z_ekf, body, ell, arc, carrot, info

    fig.suptitle("MiniNav V4 — Regulated Pure Pursuit on the EKF estimate (C++ closed-loop sim, default noise)",
                 fontsize=11)
    fig.subplots_adjust(left=0.01, right=0.99, top=0.9, bottom=0.07, wspace=0.03)
    anim = animation.FuncAnimation(fig, update, frames=frames, interval=1000 / args.fps, blit=False)
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    out = RESULTS_DIR / f"nav_{sc.key.lower()}.gif"
    raw_gif = DATA_DIR / "anim" / out.name
    anim.save(raw_gif, writer=animation.PillowWriter(fps=args.fps), dpi=80)
    # 每帧独立调色板的 GIF 很大;有 ffmpeg 时用全局 64 色调色板重新编码(体积约减半)。
    if shutil.which("ffmpeg"):
        subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", str(raw_gif), "-vf",
                        "split[a][b];[a]palettegen=max_colors=64:stats_mode=diff[p];"
                        "[b][p]paletteuse=dither=none:diff_mode=rectangle", str(out)], check=True)
    else:
        shutil.copyfile(raw_gif, out)
    print(f"wrote {out}  ({len(frames)} frames @ {args.fps} fps, status = {meta['status']}, "
          f"{meta['end_time_s']} s simulated)")


if __name__ == "__main__":
    main()
