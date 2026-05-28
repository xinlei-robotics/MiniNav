#!/usr/bin/env python3
"""
Analyze MiniNav V2 predict-only EKF behaviour.

Reads data/traj_v2.csv produced by sim_v2 (PR1, predict-only) and generates:
  - results/v2_predict_only_ellipses.png
  - results/v2_predict_only_growth.png
  - stdout summary: 初/末 trace(Σ)、位置标准差膨胀倍数等

predict-only 阶段没有观测, EKF 均值冻结在原点(v₀=ω₀=0), 而 Σ₀ 给 (v, ω) 的
不确定性被过程 Jacobian G 每步映射进 (px, py), 导致位置协方差无界增长。这张图
直观回答"为什么必须有 observation"。

Run:
    python scripts/analyze_v2_ekf.py \
        --input  data/traj_v2.csv \
        --output results/
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


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


def position_cov(row: pd.Series) -> np.ndarray:
    """从一行里取出位置 2×2 协方差子块 [[σxx, σxy], [σxy, σyy]]。"""
    return np.array(
        [[row["ekf_sigma_xx"], row["ekf_sigma_xy"]],
         [row["ekf_sigma_xy"], row["ekf_sigma_yy"]]])


def ellipse_from_cov(cov: np.ndarray, center: tuple[float, float],
                     n_std: float) -> Ellipse:
    """由 2×2 协方差构造 n_std σ 置信椭圆(特征分解 → 长短轴 + 倾角)。"""
    eigvals, eigvecs = np.linalg.eigh(cov)  # 升序, 对称矩阵实特征值
    # 主轴方向 = 最大特征值对应的特征向量
    major = eigvecs[:, 1]
    angle_deg = np.degrees(np.arctan2(major[1], major[0]))
    # 椭圆全宽/全高 = 2 · n_std · sqrt(eigenvalue)
    width, height = 2.0 * n_std * np.sqrt(np.maximum(eigvals[::-1], 0.0))
    return Ellipse(xy=center, width=width, height=height, angle=angle_deg,
                   fill=False, linewidth=1.4)


def plot_ellipses(df: pd.DataFrame, metadata: dict[str, str],
                  out_path: Path, n_samples: int = 6, n_std: float = 2.0) -> None:
    fig, ax = plt.subplots(figsize=(9, 8))

    # 背景: 三条轨迹
    ax.plot(df["truth_x"], df["truth_y"],
            label="truth", linewidth=2, color="#1f77b4")
    ax.plot(df["odom_x"], df["odom_y"],
            label="odom baseline", linewidth=1.5, color="#ff7f0e", linestyle="--")
    ax.plot(df["ekf_x"], df["ekf_y"],
            label="ekf mean (predict-only)", linewidth=1.5, color="#2ca02c")

    # 在若干等间隔采样时刻叠画位置 2σ 椭圆
    idx = np.linspace(0, len(df) - 1, n_samples).astype(int)
    cmap = plt.cm.viridis(np.linspace(0.15, 0.95, len(idx)))
    for color, k in zip(cmap, idx):
        row = df.iloc[k]
        ell = ellipse_from_cov(position_cov(row),
                               center=(row["ekf_x"], row["ekf_y"]), n_std=n_std)
        ell.set_edgecolor(color)
        ax.add_patch(ell)
        ax.annotate(f"t={row['t']:.0f}s",
                    xy=(row["ekf_x"], row["ekf_y"]),
                    xytext=(4, 4), textcoords="offset points",
                    fontsize=8, color=color)

    ax.scatter([df["truth_x"].iloc[0]], [df["truth_y"].iloc[0]],
               marker="o", s=70, color="green", label="start", zorder=5)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    preset = metadata.get("preset", "?")
    seed = metadata.get("seed", "?")
    ax.set_title(
        f"MiniNav V2 — predict-only position {n_std:.0f}σ covariance ellipses\n"
        f"preset = {preset}, seed = {seed} "
        f"(ellipses balloon with no observation)")

    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_growth(df: pd.DataFrame, metadata: dict[str, str],
                out_path: Path) -> None:
    fig, (ax_tr, ax_eig) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    # trace(Σ) = 全部 5 个对角项之和(总不确定性的标量摘要)
    trace_sigma = (df["ekf_sigma_xx"] + df["ekf_sigma_yy"] + df["ekf_sigma_thth"]
                   + df["ekf_sigma_vv"] + df["ekf_sigma_ww"])
    ax_tr.plot(df["t"], trace_sigma, color="#d62728", linewidth=1.6)
    ax_tr.set_ylabel("trace(Σ)")
    ax_tr.set_title("Total covariance trace — monotone, unbounded")
    ax_tr.grid(True, alpha=0.3)

    # 位置块 2×2 的特征值时间序列(比 trace 更能反映各向异性的膨胀)
    lam_major = np.empty(len(df))
    lam_minor = np.empty(len(df))
    for i in range(len(df)):
        eigvals = np.linalg.eigvalsh(position_cov(df.iloc[i]))
        lam_minor[i], lam_major[i] = eigvals[0], eigvals[1]
    ax_eig.plot(df["t"], lam_major, color="#9467bd", linewidth=1.6,
                label="major eigenvalue")
    ax_eig.plot(df["t"], lam_minor, color="#8c564b", linewidth=1.6,
                label="minor eigenvalue")
    ax_eig.set_ylabel("position-block eigenvalues [m²]")
    ax_eig.set_xlabel("time [s]")
    ax_eig.grid(True, alpha=0.3)
    ax_eig.legend(loc="best")

    preset = metadata.get("preset", "?")
    seed = metadata.get("seed", "?")
    fig.suptitle(
        f"MiniNav V2 — predict-only covariance growth\n"
        f"preset = {preset}, seed = {seed}")

    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def print_summary(df: pd.DataFrame, metadata: dict[str, str]) -> None:
    trace0 = (df[["ekf_sigma_xx", "ekf_sigma_yy", "ekf_sigma_thth",
                  "ekf_sigma_vv", "ekf_sigma_ww"]].iloc[0].sum())
    trace_f = (df[["ekf_sigma_xx", "ekf_sigma_yy", "ekf_sigma_thth",
                   "ekf_sigma_vv", "ekf_sigma_ww"]].iloc[-1].sum())
    std_x0 = np.sqrt(df["ekf_sigma_xx"].iloc[0])
    std_xf = np.sqrt(df["ekf_sigma_xx"].iloc[-1])
    mean_drift = np.hypot(df["ekf_x"].iloc[-1] - df["ekf_x"].iloc[0],
                          df["ekf_y"].iloc[-1] - df["ekf_y"].iloc[0])

    print("=" * 60)
    print("MiniNav V2 predict-only EKF summary")
    print("=" * 60)
    for k, v in metadata.items():
        print(f"  {k:14s} = {v}")
    print("-" * 60)
    print(f"  initial trace(Σ)      = {trace0:.4e}")
    print(f"  final   trace(Σ)      = {trace_f:.4e}")
    print(f"  σ_x growth            = {std_x0:.4e} m -> {std_xf:.4e} m "
          f"({std_xf / std_x0:.1f}×)")
    print(f"  EKF mean displacement = {mean_drift:.4e} m "
          f"(predict-only: 期望 ≈ 0, 均值冻结)")
    print("=" * 60)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default="data/traj_v2.csv", type=Path)
    parser.add_argument("--output", default="results/", type=Path)
    parser.add_argument(
        "--with-ellipses", action="store_true",
        help="额外输出位置协方差椭圆图。predict-only(均值冻结、θ 恒定)下椭圆退化为"
             "沿运动轴的一维拉长形状, 故 PR1 默认不出; 该图的黄金场景是 PR2"
             "(均值跟随 truth, 椭圆沿路径铺开后随 update 收敛)。")
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)

    df, metadata = load_trajectory(args.input)

    # PR1 主图: 协方差增长曲线(trace + 位置块特征值)。
    plot_growth(df, metadata, args.output / "v2_predict_only_growth.png")
    if args.with_ellipses:
        plot_ellipses(df, metadata, args.output / "v2_predict_only_ellipses.png")
    print_summary(df, metadata)


if __name__ == "__main__":
    main()