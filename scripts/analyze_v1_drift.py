#!/usr/bin/env python3
"""
Analyze MiniNav V1 wheel-odometry drift.

Reads data/traj_v1.csv produced by sim_v1 and generates:
  - results/v1_trajectory.png       —— truth vs odom 2D trajectory
  - results/v1_drift_over_time.png  —— position / yaw error vs time
  - stdout summary: final position error, final yaw error, peak error

Run:
    python scripts/analyze_v1_drift.py \
        --input  data/traj_v1.csv \
        --output results/
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def parse_metadata(path: Path) -> dict[str, str]:
    """CSV 文件前置的 # ... 注释行,提取 key=value 元信息。"""
    metadata: dict[str, str] = {}
    with path.open() as f:
        for line in f:
            if not line.startswith("#"):
                break
            payload = line.lstrip("#").strip()
            if "=" in payload:
                key, _, val = payload.partition("=")
                metadata[key.strip()] = val.strip()
    return metadata


def load_trajectory(path: Path) -> tuple[pd.DataFrame, dict[str, str]]:
    metadata = parse_metadata(path)
    df = pd.read_csv(path, comment="#")
    return df, metadata


def wrap_to_pi(angle: np.ndarray) -> np.ndarray:
    """规范化到 (-pi, pi],与 C++ 端的 atan2 trick 一致。"""
    return np.arctan2(np.sin(angle), np.cos(angle))


def compute_drift(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    df["err_x"]   = df["odom_x"]   - df["truth_x"]
    df["err_y"]   = df["odom_y"]   - df["truth_y"]
    df["err_pos"] = np.hypot(df["err_x"], df["err_y"])
    df["err_yaw"] = wrap_to_pi(df["odom_yaw"] - df["truth_yaw"])
    return df


def plot_trajectory(df: pd.DataFrame, metadata: dict[str, str], out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(df["truth_x"], df["truth_y"],
            label="truth",  linewidth=2, color="#1f77b4")
    ax.plot(df["odom_x"],  df["odom_y"],
            label="odom estimate", linewidth=2, color="#ff7f0e", linestyle="--")
    ax.scatter([df["truth_x"].iloc[0]], [df["truth_y"].iloc[0]],
               marker="o", s=80, color="green", label="start", zorder=5)
    ax.scatter([df["truth_x"].iloc[-1]], [df["truth_y"].iloc[-1]],
               marker="s", s=80, color="#1f77b4", label="truth end", zorder=5)
    ax.scatter([df["odom_x"].iloc[-1]], [df["odom_y"].iloc[-1]],
               marker="s", s=80, color="#ff7f0e", label="odom end", zorder=5)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    preset = metadata.get("preset", "?")
    seed   = metadata.get("seed",   "?")
    ax.set_title(
        f"MiniNav V1 — truth vs odom trajectory\n"
        f"preset = {preset},  seed = {seed}")

    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_drift_over_time(df: pd.DataFrame, metadata: dict[str, str],
                         out_path: Path) -> None:
    fig, (ax_pos, ax_yaw) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    ax_pos.plot(df["t"], df["err_pos"], color="#d62728", linewidth=1.5)
    ax_pos.set_ylabel("position error [m]")
    ax_pos.grid(True, alpha=0.3)

    ax_yaw.plot(df["t"], np.degrees(df["err_yaw"]),
                color="#9467bd", linewidth=1.5)
    ax_yaw.set_ylabel("yaw error [deg]")
    ax_yaw.set_xlabel("time [s]")
    ax_yaw.grid(True, alpha=0.3)

    preset = metadata.get("preset", "?")
    seed   = metadata.get("seed",   "?")
    fig.suptitle(
        f"MiniNav V1 — odometry drift over time\n"
        f"preset = {preset},  seed = {seed}")

    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def print_summary(df: pd.DataFrame, metadata: dict[str, str]) -> None:
    print("=" * 60)
    print("MiniNav V1 drift summary")
    print("=" * 60)
    for k, v in metadata.items():
        print(f"  {k:14s} = {v}")
    print("-" * 60)
    print(f"  final position error  = {df['err_pos'].iloc[-1]:.4f} m")
    print(f"  peak  position error  = {df['err_pos'].max():.4f} m")
    print(f"  final yaw error       = {np.degrees(df['err_yaw'].iloc[-1]):.3f} deg")
    print(f"  peak  yaw error (abs) = {np.degrees(df['err_yaw'].abs().max()):.3f} deg")
    print(f"  total truth distance  = "
          f"{np.sum(np.hypot(np.diff(df['truth_x']), np.diff(df['truth_y']))):.4f} m")
    print("=" * 60)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input",  default="data/traj_v1.csv", type=Path)
    parser.add_argument("--output", default="results/",         type=Path)
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)

    df, metadata = load_trajectory(args.input)
    df = compute_drift(df)

    plot_trajectory(df, metadata, args.output / "v1_trajectory.png")
    plot_drift_over_time(df, metadata, args.output / "v1_drift_over_time.png")
    print_summary(df, metadata)


if __name__ == "__main__":
    main()