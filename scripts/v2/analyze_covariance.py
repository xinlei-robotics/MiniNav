#!/usr/bin/env python3
"""
V2 位置协方差椭圆随时间的演化分析。

EKF 的位置 (x,y) 块 [[σxx, σxy],[σxy, σyy]] 是一个 2D 高斯, 可画成置信椭圆。
本脚本从 data/traj_v2.csv 读出该块, 展示它在 20s 运行里【怎么变】—— 这是
observability 的直观证据: 位置从未被直接观测, 椭圆只能随 dead-reckoning 单调
膨胀, 且长轴随运动朝向旋转。

(θ/v/ω/b_ω 是标量方差, 不构成 2D 椭圆, 已由 analyze_ekf 的 state_errors / bias_learning
覆盖, 不在此脚本范围。)

产出(默认 3σ, 99.7%):
  - results/v2/covariance_ellipses.png   静态: 轨迹 + 若干时刻的 Nσ 椭圆(按时间上色)
  - results/v2/covariance_geometry.png   定量: 长/短半轴、面积、长轴倾角随时间
  - results/v2/covariance_evolution.gif  动画: 椭圆随均值移动/膨胀(--format mp4 可换)

Run:
    python scripts/v2/analyze_covariance.py --input data/traj_v2.csv --output results/v2/
    python scripts/v2/analyze_covariance.py --no-anim          # 跳过较慢的动画
    python scripts/v2/analyze_covariance.py --format mp4        # 用 ffmpeg 出 MP4
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # 无显示环境 / 动画写盘

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter

# 复用 analyze_ekf 的 CSV 读取与椭圆构造(同目录)。
sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_ekf import load_trajectory, position_cov, ellipse_from_cov  # noqa: E402


def ellipse_geometry(df, n_std: float):
    """逐步算位置块的椭圆几何量(向量化)。

    返回 dict: a(长半轴), b(短半轴), area, angle_deg(长轴倾角, [-90,90])。
    2×2 对称阵特征值: (xx+yy)/2 ± sqrt(((xx-yy)/2)^2 + xy^2);
    长轴方向 θ = 0.5·atan2(2·xy, xx-yy)。
    """
    xx = df["ekf_sigma_xx"].to_numpy()
    yy = df["ekf_sigma_yy"].to_numpy()
    xy = df["ekf_sigma_xy"].to_numpy()

    mean = 0.5 * (xx + yy)
    rad = np.sqrt(np.maximum(((xx - yy) * 0.5) ** 2 + xy ** 2, 0.0))
    lam_major = np.maximum(mean + rad, 0.0)
    lam_minor = np.maximum(mean - rad, 0.0)

    a = n_std * np.sqrt(lam_major)
    b = n_std * np.sqrt(lam_minor)
    area = np.pi * a * b
    angle_deg = np.degrees(0.5 * np.arctan2(2.0 * xy, xx - yy))
    return {"a": a, "b": b, "area": area, "angle_deg": angle_deg}


# --- A. 静态快照叠加 -------------------------------------------------------
def plot_ellipse_snapshots(df, metadata, out_path: Path,
                           n_std: float, n_samples: int = 8) -> None:
    fig, ax = plt.subplots(figsize=(9, 8))

    ax.plot(df["truth_x"], df["truth_y"], color="#1f77b4", linewidth=2.0, label="truth")
    ax.plot(df["odom_x"], df["odom_y"], color="#ff7f0e", linewidth=1.3,
            linestyle="--", label="odom")
    ax.plot(df["ekf_x"], df["ekf_y"], color="#2ca02c", linewidth=1.5, label="ekf mean")

    idx = np.linspace(0, len(df) - 1, n_samples).astype(int)
    cmap = plt.cm.viridis(np.linspace(0.12, 0.95, len(idx)))
    for color, k in zip(cmap, idx):
        row = df.iloc[k]
        ell = ellipse_from_cov(position_cov(row), center=(row["ekf_x"], row["ekf_y"]),
                               n_std=n_std)
        ell.set_edgecolor(color)
        ax.add_patch(ell)
        ax.annotate(f"t={row['t']:.0f}s", xy=(row["ekf_x"], row["ekf_y"]),
                    xytext=(4, 4), textcoords="offset points", fontsize=8, color=color)

    ax.scatter([df["truth_x"].iloc[0]], [df["truth_y"].iloc[0]],
               marker="o", s=70, color="green", zorder=5, label="start")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3); ax.legend(loc="best")

    preset = metadata.get("preset", "?"); seed = metadata.get("seed", "?")
    ax.set_title(
        f"MiniNav V2 — position {n_std:.0f}σ covariance ellipses along the run\n"
        f"preset = {preset}, seed = {seed}  "
        f"(position unobserved → ellipse grows; major axis tracks heading)")
    fig.tight_layout(); fig.savefig(out_path, dpi=140); plt.close(fig)


# --- B. 几何量时间序列 -----------------------------------------------------
def plot_ellipse_geometry(df, metadata, out_path: Path, n_std: float) -> None:
    g = ellipse_geometry(df, n_std)
    t = df["t"].to_numpy()

    fig, (ax_ax, ax_area, ax_ang) = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    ax_ax.plot(t, g["a"], color="#9467bd", linewidth=1.8, label=f"major semi-axis ({n_std:.0f}σ)")
    ax_ax.plot(t, g["b"], color="#8c564b", linewidth=1.8, label=f"minor semi-axis ({n_std:.0f}σ)")
    ax_ax.set_ylabel("semi-axis [m]"); ax_ax.grid(True, alpha=0.3); ax_ax.legend(loc="best")
    ax_ax.set_title("Ellipse semi-axes — grow as position drifts (dead reckoning)")

    ax_area.plot(t, g["area"], color="#d62728", linewidth=1.8)
    ax_area.set_ylabel(f"{n_std:.0f}σ area [m²]"); ax_area.grid(True, alpha=0.3)
    ax_area.set_title("Ellipse area (∝ √det Σ_pos) — total positional uncertainty")

    ax_ang.plot(t, g["angle_deg"], color="#2ca02c", linewidth=1.4, label="major-axis angle")
    ax_ang.plot(t, np.degrees(df["ekf_yaw"].to_numpy()), color="#1f77b4", linewidth=1.0,
                alpha=0.6, label="ekf heading (yaw)")
    ax_ang.set_ylabel("angle [deg]"); ax_ang.set_xlabel("time [s]")
    ax_ang.grid(True, alpha=0.3); ax_ang.legend(loc="best")
    ax_ang.set_title("Major-axis orientation vs heading — uncertainty elongates along motion")

    preset = metadata.get("preset", "?"); seed = metadata.get("seed", "?")
    fig.suptitle(f"MiniNav V2 — covariance-ellipse geometry over time "
                 f"(preset = {preset}, seed = {seed})")
    fig.tight_layout(); fig.savefig(out_path, dpi=140); plt.close(fig)


# --- C. 动画 ---------------------------------------------------------------
def animate_ellipse(df, metadata, out_path: Path, n_std: float,
                    frames: int, fps: int, fmt: str) -> None:
    g = ellipse_geometry(df, n_std)
    step = max(1, len(df) // frames)
    sel = list(range(0, len(df), step))

    # 固定坐标轴: 数据范围 + 最大半轴留边, 让椭圆"移动+膨胀"看得清。
    pad = float(np.max(g["a"])) + 0.1
    xs = np.concatenate([df["truth_x"].to_numpy(), df["ekf_x"].to_numpy()])
    ys = np.concatenate([df["truth_y"].to_numpy(), df["ekf_y"].to_numpy()])
    xlim = (xs.min() - pad, xs.max() + pad)
    ylim = (ys.min() - pad, ys.max() + pad)

    fig, ax = plt.subplots(figsize=(8, 8))
    preset = metadata.get("preset", "?"); seed = metadata.get("seed", "?")

    def draw(frame_i: int):
        ax.clear()
        k = sel[frame_i]
        ax.plot(df["truth_x"].iloc[:k + 1], df["truth_y"].iloc[:k + 1],
                color="#1f77b4", linewidth=1.6, label="truth")
        ax.plot(df["ekf_x"].iloc[:k + 1], df["ekf_y"].iloc[:k + 1],
                color="#2ca02c", linewidth=1.6, label="ekf mean")
        row = df.iloc[k]
        ell = ellipse_from_cov(position_cov(row), center=(row["ekf_x"], row["ekf_y"]),
                               n_std=n_std)
        ell.set_edgecolor("#d62728")
        ax.add_patch(ell)
        ax.scatter([row["ekf_x"]], [row["ekf_y"]], s=25, color="#2ca02c", zorder=5)
        ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.set_aspect("equal")
        ax.grid(True, alpha=0.3); ax.legend(loc="upper left")
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        ax.set_title(f"MiniNav V2 — position {n_std:.0f}σ ellipse, t = {row['t']:.2f}s\n"
                     f"preset = {preset}, seed = {seed}")

    anim = FuncAnimation(fig, draw, frames=len(sel), interval=1000 / fps)
    if fmt == "mp4":
        anim.save(str(out_path), writer=FFMpegWriter(fps=fps))
    else:
        anim.save(str(out_path), writer=PillowWriter(fps=fps))
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input", default="data/traj_v2.csv", type=Path)
    parser.add_argument("--output", default="results/v2/", type=Path)
    parser.add_argument("--n-std", type=float, default=3.0, help="椭圆置信等级(默认 3σ)。")
    parser.add_argument("--no-anim", action="store_true", help="跳过较慢的动画输出。")
    parser.add_argument("--format", choices=["gif", "mp4"], default="gif",
                        help="动画格式(gif=Pillow, mp4=ffmpeg)。")
    parser.add_argument("--frames", type=int, default=150, help="动画抽稀后的目标帧数。")
    parser.add_argument("--fps", type=int, default=20)
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    df, metadata = load_trajectory(args.input)

    plot_ellipse_snapshots(df, metadata, args.output / "covariance_ellipses.png", args.n_std)
    plot_ellipse_geometry(df, metadata, args.output / "covariance_geometry.png", args.n_std)
    if not args.no_anim:
        out = args.output / f"covariance_evolution.{args.format}"
        animate_ellipse(df, metadata, out, args.n_std, args.frames, args.fps, args.format)

    g = ellipse_geometry(df, args.n_std)
    print("=" * 60)
    print(f"MiniNav V2 — covariance ellipse  [preset = {metadata.get('preset','?')}, "
          f"seed = {metadata.get('seed','?')}, {args.n_std:.0f}σ]")
    print("-" * 60)
    print(f"  {args.n_std:.0f}σ area:        {g['area'][0]:.3e} -> {g['area'][-1]:.3e} m²  "
          f"({g['area'][-1] / max(g['area'][0], 1e-12):.0f}x)")
    print(f"  major semi-axis:  {g['a'][0]:.4f} -> {g['a'][-1]:.4f} m")
    print(f"  minor semi-axis:  {g['b'][0]:.4f} -> {g['b'][-1]:.4f} m")
    print(f"  anisotropy a/b:   {g['a'][-1] / max(g['b'][-1], 1e-12):.2f} (final)")
    print("=" * 60)


if __name__ == "__main__":
    main()
