"""
plot_trajectory.py
==================
将 MiniNav 仿真器输出的 CSV 轨迹渲染为 PNG 图。

输入 CSV 列(由 src/core/csv_format.cpp 写出):
    t, x, y, yaw, v, w

输出布局:
    上半部:x-y 平面轨迹(主视觉,正方形 aspect),含起点 / 终点 marker
            与每隔 N 步的朝向箭头。
    下半部:v(t) 与 w(t) 双栏时间序列,展示 staged command 的分段常值结构。

用法:
    python scripts/plot_trajectory.py \
        --input data/traj.csv \
        --output results/traj_v0.png

依赖:numpy, pandas, matplotlib
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.gridspec import GridSpec


# ---------------------------------------------------------------------------
# 视觉参数
# ---------------------------------------------------------------------------
TRAJ_COLOR = "#1f77b4"        # 主轨迹 - 蓝
START_COLOR = "#2ca02c"       # 起点 - 绿
END_COLOR = "#d62728"         # 终点 - 红
ARROW_COLOR = "#1f77b4"       # 朝向箭头 - 与轨迹同色稍弱
V_COLOR = "#1f77b4"           # 线速度 - 蓝
W_COLOR = "#ff7f0e"           # 角速度 - 橙

ARROW_EVERY_N_STEPS = 150     # 每隔多少步画一个朝向箭头
ARROW_LENGTH_FRAC = 0.05      # 箭头长度 = 数据范围 * 该比例

FIG_DPI = 200                 # 200 DPI


def _load_trajectory(csv_path: Path) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    expected_cols = {"t", "x", "y", "yaw", "v", "w"}
    missing = expected_cols - set(df.columns)
    if missing:
        raise ValueError(
            f"CSV 缺少必要列: {missing}. 实际列: {list(df.columns)}"
        )
    if df.empty:
        raise ValueError(f"CSV 文件为空: {csv_path}")
    return df


def _plot_xy(ax: plt.Axes, df: pd.DataFrame) -> None:
    x = df["x"].to_numpy()
    y = df["y"].to_numpy()
    yaw = df["yaw"].to_numpy()

    # 主轨迹
    ax.plot(x, y, color=TRAJ_COLOR, linewidth=1.6, label="Trajectory", zorder=2)

    # 起点 / 终点 marker
    ax.scatter(
        x[0], y[0],
        s=110, color=START_COLOR, marker="o",
        edgecolors="white", linewidths=1.5,
        label="Start", zorder=4,
    )
    ax.scatter(
        x[-1], y[-1],
        s=140, color=END_COLOR, marker="X",
        edgecolors="white", linewidths=1.5,
        label="End", zorder=4,
    )

    # 朝向箭头
    data_span = max(x.max() - x.min(), y.max() - y.min())
    arrow_len = ARROW_LENGTH_FRAC * data_span
    indices = np.arange(0, len(df), ARROW_EVERY_N_STEPS)
    ax.quiver(
        x[indices], y[indices],
        arrow_len * np.cos(yaw[indices]),
        arrow_len * np.sin(yaw[indices]),
        angles="xy", scale_units="xy", scale=1.0,
        color=ARROW_COLOR, alpha=0.65, width=0.005,
        zorder=3,
        )

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Robot trajectory (ground truth)", fontsize=12, pad=10)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3, linestyle="--")
    ax.legend(loc="best", framealpha=0.9, fontsize=9)


def _plot_command(ax: plt.Axes, df: pd.DataFrame, column: str,
                  ylabel: str, title: str, color: str) -> None:
    ax.step(df["t"], df[column], where="post", color=color, linewidth=1.4)
    ax.fill_between(df["t"], df[column], step="post", alpha=0.15, color=color)
    ax.set_xlabel("t [s]")
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=11, pad=8)
    ax.grid(True, alpha=0.3, linestyle="--")
    ax.axhline(0.0, color="black", linewidth=0.6, alpha=0.4)


def render(csv_path: Path, output_path: Path) -> None:
    df = _load_trajectory(csv_path)

    fig = plt.figure(figsize=(9, 11))
    gs = GridSpec(
        nrows=2, ncols=2,
        height_ratios=[3.5, 1.2],
        hspace=0.30, wspace=0.25,
        left=0.10, right=0.96, top=0.94, bottom=0.06,
    )
    ax_xy = fig.add_subplot(gs[0, :])
    ax_v = fig.add_subplot(gs[1, 0])
    ax_w = fig.add_subplot(gs[1, 1])

    _plot_xy(ax_xy, df)
    _plot_command(ax_v, df, "v", "v [m/s]",
                  "Linear velocity command", V_COLOR)
    _plot_command(ax_w, df, "w", r"$\omega$ [rad/s]",
                  "Angular velocity command", W_COLOR)

    fig.suptitle(
        f"MiniNav V0 — Ideal differential-drive simulation"
        f"  ({df['t'].iloc[-1]:.1f} s, dt = "
        f"{df['t'].iloc[1] - df['t'].iloc[0]:.3f} s)",
        fontsize=13, y=0.985,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=FIG_DPI, bbox_inches="tight",
                facecolor="white")
    plt.close(fig)
    print(f"[plot_trajectory] wrote {output_path} ({df.shape[0]} rows)")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render MiniNav trajectory CSV to a PNG figure."
    )
    parser.add_argument(
        "--input", type=Path, default=Path("data/traj.csv"),
        help="Path to trajectory CSV (default: data/traj.csv)",
    )
    parser.add_argument(
        "--output", type=Path, default=Path("results/traj_v0.png"),
        help="Output PNG path (default: results/traj_v0.png)",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    render(args.input, args.output)


if __name__ == "__main__":
    main()