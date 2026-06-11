#!/usr/bin/env python3
"""
Analyze MiniNav V2 EKF behaviour (mode-aware).

读 data/traj.csv, 根据 CSV 头部的 `# mode` 元信息选择产图:

mode = predict-only:
  - results/v2/predict_only_growth.png    trace(Σ) 与位置块特征值的无界增长
  - results/v2/predict_only_ellipses.png  (--with-ellipses) 位置 2σ 椭圆膨胀

mode = encoder-update:
  - results/v2/encoder_trajectory.png        truth / odom / ekf 三轨迹叠加。
      ekf 与 odom 几乎重合(encoder-only EKF = dead reckoning), 两者都随时间
      偏离 truth —— encoder 约束不了位置漂移。
  - results/v2/encoder_covariance_split.png  速度块 vs 位置块协方差分离。

mode = encoder+imu:
  - results/v2/fusion_trajectory.png    truth / odom / ekf-encoder-only /
      ekf-fused 四轨迹叠加。fused 轨迹明显贴近 truth, 而 odom/encoder-only
      因 dead reckoning 偏离, IMU 让位置不再无界发散。
      注: encoder-only 不在 CSV 里, 但 odom 提供"无 IMU" 基线;
      fused 与 odom 的差距就是 IMU 的贡献。
  - results/v2/fusion_rmse_over_time.png  RMSE(truth, odom) 与 RMSE(truth, ekf)
      时间序列。给定 --ekf-no-bias 时叠加第三条 'ekf (no bias)', 形成
      odom / ekf / ekf_with_bias 三方对比(三条 RMSE 曲线)。
  - results/v2/state_errors.png  6 个状态维 (px,py,θ,v,ω,b_ω) 的 estimation error
      + ±3σ 包络。一致的 filter 误差应大部分落在带内; 标题给出落入比例。
  - results/v2/bias_learning.png:
      上图 b_ω 估计 + ±2σ 带 + 真值参考线(由 preset 恢复), 几秒内收敛到真值;
      下图 Σ_bb 从无信息先验(1e-2)对数坐标下塌缩 —— encoder+IMU 让 bias 可观测
      的最直观证据(state augmentation 的"招牌图")。
  - results/v2/nis_consistency.png  encoder/IMU 的 NIS + χ² 95% 区间带(consistency)。

无论哪种模式都打印 stdout summary。

Run:
    # 单次运行(2 条 RMSE):
    python scripts/v2/analyze_ekf.py --input data/traj.csv --output results/
    # 三方 RMSE 对比(需另跑一次 sim --no-bias, 同 seed/preset):
    python scripts/v2/analyze_ekf.py --input data/traj.csv \\
        --ekf-no-bias data/traj_nobias.csv --output results/
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


def wrap_to_pi(angle: np.ndarray) -> np.ndarray:
    """规范化到 (-pi, pi], 与 C++ 端 atan2(sin, cos) trick 一致。"""
    return np.arctan2(np.sin(angle), np.cos(angle))


# 各 preset 注入的常数gyro bias 真值, 镜像 sim_main.cpp 的 NoisePreset.imu_bias_init。
# ⚠ 这是 Python 侧对 C++ 常量的手工镜像, 存在耦合: 改了 C++ preset 必须同步这里。
#   更稳健的做法是让 C++ 把真值作为一列(如 imu_bias_true)写进 CSV, 分析脚本就
#   完全不必知道 preset 表 —— 见文件末尾 main() 上方的说明。
# 仅在 bias 为常数(imu_bias_rw == 0, 即所有出厂 preset)时, 真值可由 preset 名恢复;
# 若用了漂移 bias, 此值只代表初值, recover 后果由调用方决定是否使用。
_PRESET_TRUE_BIAS: dict[str, float] = {
    "low-noise": 0.01,
    "default": 0.02,
    "high-noise": 0.03,
}


def recover_true_bias(metadata: dict[str, str]) -> float | None:
    """从 CSV 元信息里的 preset 名恢复常数 gyro bias 真值; 未知 preset 返回 None。"""
    return _PRESET_TRUE_BIAS.get(metadata.get("preset", ""))


# χ² 分布 95% 双侧单样本区间 [2.5%, 97.5%] 分位点(无 scipy 依赖, 硬编码)。
#   dof=2 有闭式: χ²₂ 的 CDF = 1−e^{−x/2} ⇒ 分位点 x = −2·ln(1−q),
#                故 [−2ln0.975, −2ln0.025] = [0.05064, 7.37776]。
#   dof=1 无初等闭式, 用标准数值常量。
_CHI2_95: dict[int, tuple[float, float]] = {
    1: (0.0009820691, 5.0238862),
    2: (0.0506356, 7.3777589),
}


def nis_consistency(nis: np.ndarray, dof: int) -> tuple[float, float, float, float]:
    """返回 (mean, 区间下界, 区间上界, 落入 95% χ² 区间的比例%)。

    一致(consistent)的 filter: mean(NIS) ≈ dof, 且约 95% 的样本落在区间内。
    显著偏高 → Q/R 设得过小(过度自信); 偏低 → 设得过大(过度保守)。
    """
    lo, hi = _CHI2_95[dof]
    inside = float(np.mean((nis >= lo) & (nis <= hi))) * 100.0
    return float(np.mean(nis)), lo, hi, inside


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


def check_same_world(a: pd.DataFrame, b: pd.DataFrame) -> bool:
    """两份运行的 truth / 测量流应逐位相同(同 seed ⇒ 同世界); 否则跨估计器对比不成立。"""
    if len(a) != len(b):
        return False
    cols = ["truth_x", "truth_y", "truth_yaw", "imu_omega", "enc_dl", "enc_dr"]
    return all(np.allclose(a[c], b[c], rtol=0, atol=0) for c in cols)


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

    # trace(Σ) = 全部 6 个对角项之和(总不确定性的标量摘要; PR4 起含 bias 维 σ_bb)
    trace_sigma = (df["ekf_sigma_xx"] + df["ekf_sigma_yy"] + df["ekf_sigma_thth"]
                   + df["ekf_sigma_vv"] + df["ekf_sigma_ww"] + df["ekf_sigma_bb"])
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


def plot_trajectory_overlay(df: pd.DataFrame, metadata: dict[str, str],
                            out_path: Path) -> None:
    """truth / odom / ekf 三轨迹叠加。encoder-update 模式下 ekf 应≈odom, 两者都漂离 truth。"""
    fig, ax = plt.subplots(figsize=(9, 8))

    ax.plot(df["truth_x"], df["truth_y"],
            label="truth", linewidth=2.2, color="#1f77b4")
    ax.plot(df["odom_x"], df["odom_y"],
            label="odom (V1 baseline)", linewidth=1.6, color="#ff7f0e", linestyle="--")
    ax.plot(df["ekf_x"], df["ekf_y"],
            label="ekf (encoder-only)", linewidth=1.6, color="#2ca02c", linestyle=":")

    ax.scatter([df["truth_x"].iloc[0]], [df["truth_y"].iloc[0]],
               marker="o", s=70, color="green", label="start", zorder=5)
    ax.scatter([df["truth_x"].iloc[-1]], [df["truth_y"].iloc[-1]],
               marker="s", s=70, color="#1f77b4", zorder=5)
    ax.scatter([df["ekf_x"].iloc[-1]], [df["ekf_y"].iloc[-1]],
               marker="X", s=80, color="#2ca02c", zorder=5)

    # 量化 ekf 与 odom 的贴合度。
    max_ekf_odom = np.hypot(df["ekf_x"] - df["odom_x"], df["ekf_y"] - df["odom_y"]).max()

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    preset = metadata.get("preset", "?")
    seed = metadata.get("seed", "?")
    ax.set_title(
        f"MiniNav V2 — encoder-only EKF tracks wheel odometry\n"
        f"preset = {preset}, seed = {seed}   "
        f"(max |ekf − odom| = {max_ekf_odom * 100:.1f} cm; both drift from truth)")

    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_covariance_split(df: pd.DataFrame, metadata: dict[str, str],
                          out_path: Path) -> None:
    """速度块 vs 位置块协方差 trace。核心 observability 图: 速度有界、位置发散。"""
    pos_trace = df["ekf_sigma_xx"] + df["ekf_sigma_yy"]
    vel_trace = df["ekf_sigma_vv"] + df["ekf_sigma_ww"]

    fig, (ax_pos, ax_vel) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    ax_pos.plot(df["t"], pos_trace, color="#d62728", linewidth=1.8)
    ax_pos.set_yscale("log")
    ax_pos.set_ylabel("position block\ntrace(Σ_xx + Σ_yy) [m²]")
    ax_pos.set_title("Position covariance — UNBOUNDED (position unobservable → dead reckoning)")
    ax_pos.grid(True, alpha=0.3, which="both")

    ax_vel.plot(df["t"], vel_trace, color="#2ca02c", linewidth=1.8)
    ax_vel.set_ylabel("velocity block\ntrace(Σ_vv + Σ_ωω)")
    ax_vel.set_xlabel("time [s]")
    ax_vel.set_title("Velocity covariance — BOUNDED (directly observed by encoder)")
    ax_vel.grid(True, alpha=0.3)
    # 速度块从近乎贴零的稳态值起步, 设一个不从 0 开始的 y 轴更能看出 plateau。
    vel_min, vel_max = vel_trace.min(), vel_trace.max()
    margin = 0.15 * (vel_max - vel_min + 1e-12)
    ax_vel.set_ylim(max(0.0, vel_min - margin), vel_max + margin)

    preset = metadata.get("preset", "?")
    seed = metadata.get("seed", "?")
    fig.suptitle(
        f"MiniNav V2 — observability split: encoder bounds velocity, not position\n"
        f"preset = {preset}, seed = {seed}")

    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_fusion_trajectory(df: pd.DataFrame, metadata: dict[str, str],
                           out_path: Path) -> None:
    """truth / odom / ekf-fused 三轨迹叠加。fused 应明显贴近 truth。"""
    fig, ax = plt.subplots(figsize=(9, 8))

    ax.plot(df["truth_x"], df["truth_y"],
            label="truth", linewidth=2.4, color="#1f77b4")
    ax.plot(df["odom_x"], df["odom_y"],
            label="odom (encoder only, V1 baseline)",
            linewidth=1.6, color="#ff7f0e", linestyle="--")
    ax.plot(df["ekf_x"], df["ekf_y"],
            label="ekf fused (encoder + IMU)",
            linewidth=2.0, color="#2ca02c")

    ax.scatter([df["truth_x"].iloc[0]], [df["truth_y"].iloc[0]],
               marker="o", s=70, color="green", label="start", zorder=5)
    ax.scatter([df["truth_x"].iloc[-1]], [df["truth_y"].iloc[-1]],
               marker="s", s=70, color="#1f77b4", zorder=5)
    ax.scatter([df["odom_x"].iloc[-1]], [df["odom_y"].iloc[-1]],
               marker="X", s=80, color="#ff7f0e", zorder=5)
    ax.scatter([df["ekf_x"].iloc[-1]], [df["ekf_y"].iloc[-1]],
               marker="X", s=80, color="#2ca02c", zorder=5)

    odom_final_err = np.hypot(df["odom_x"].iloc[-1] - df["truth_x"].iloc[-1],
                              df["odom_y"].iloc[-1] - df["truth_y"].iloc[-1])
    ekf_final_err = np.hypot(df["ekf_x"].iloc[-1] - df["truth_x"].iloc[-1],
                             df["ekf_y"].iloc[-1] - df["truth_y"].iloc[-1])

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    preset = metadata.get("preset", "?")
    seed = metadata.get("seed", "?")
    ax.set_title(
        f"MiniNav V2 — encoder + IMU fusion vs single-sensor odom\n"
        f"preset = {preset}, seed = {seed}   "
        f"(final |odom-truth| = {odom_final_err * 100:.1f} cm; "
        f"|ekf-truth| = {ekf_final_err * 100:.1f} cm)")

    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def _prefix_rmse(df: pd.DataFrame, x: str, y: str, yaw: str) -> tuple[np.ndarray, np.ndarray]:
    """某估计器相对 truth 的累积(prefix)position / yaw RMSE 时间序列。"""
    dx = df[x] - df["truth_x"]
    dy = df[y] - df["truth_y"]
    dyaw = wrap_to_pi((df[yaw] - df["truth_yaw"]).to_numpy())
    n = np.arange(1, len(df) + 1)
    return (np.sqrt((dx ** 2 + dy ** 2).cumsum() / n),
            np.sqrt((dyaw ** 2).cumsum() / n))


def plot_fusion_rmse_over_time(df: pd.DataFrame, metadata: dict[str, str],
                               out_path: Path,
                               df_nobias: pd.DataFrame | None = None) -> None:
    """累积 RMSE 时间序列: odom vs ekf-fused, 给定 df_nobias 时再叠加 ekf(no bias)。

    df_nobias 应是【同 seed/preset、--no-bias】的运行(truth/测量流逐位相同), 这样三条
    曲线只在"估计器"维度不同 —— 干净对比 odom / ekf / ekf_with_bias。
    """
    t = df["t"]
    rmse_odom_pos, rmse_odom_yaw = _prefix_rmse(df, "odom_x", "odom_y", "odom_yaw")
    rmse_ekf_pos, rmse_ekf_yaw = _prefix_rmse(df, "ekf_x", "ekf_y", "ekf_yaw")

    fig, (ax_pos, ax_yaw) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    ax_pos.plot(t, rmse_odom_pos, color="#ff7f0e", linewidth=1.8, label="odom (encoder only)")
    ax_yaw.plot(t, np.degrees(rmse_odom_yaw), color="#ff7f0e", linewidth=1.8, label="odom")

    if df_nobias is not None:
        nb_pos, nb_yaw = _prefix_rmse(df_nobias, "ekf_x", "ekf_y", "ekf_yaw")
        ax_pos.plot(t, nb_pos, color="#9467bd", linewidth=1.8, label="ekf (no bias)")
        ax_yaw.plot(t, np.degrees(nb_yaw), color="#9467bd", linewidth=1.8, label="ekf (no bias)")

    ekf_label = "ekf_with_bias (encoder + IMU)" if df_nobias is not None \
        else "ekf fused (encoder + IMU)"
    ax_pos.plot(t, rmse_ekf_pos, color="#2ca02c", linewidth=1.8, label=ekf_label)
    ax_yaw.plot(t, np.degrees(rmse_ekf_yaw), color="#2ca02c", linewidth=1.8, label=ekf_label)

    ax_pos.set_ylabel("cumulative position RMSE [m]")
    ax_pos.grid(True, alpha=0.3)
    ax_pos.legend(loc="best")
    ax_pos.set_title("Position RMSE — IMU fusion stops the drift at the turn")

    ax_yaw.set_ylabel("cumulative yaw RMSE [deg]")
    ax_yaw.set_xlabel("time [s]")
    ax_yaw.grid(True, alpha=0.3)
    ax_yaw.legend(loc="best")
    ax_yaw.set_title("Yaw RMSE — IMU keeps heading anchored")

    preset = metadata.get("preset", "?")
    seed = metadata.get("seed", "?")
    fig.suptitle(
        f"MiniNav V2 — RMSE comparison (preset = {preset}, seed = {seed})",
        y=1.00)

    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_bias_learning(df: pd.DataFrame, metadata: dict[str, str],
                       out_path: Path) -> None:
    """gyro bias 在线估计 —— 估计值收敛 + 协方差 Σ_bb 塌缩。

    这是 state augmentation 最直观的证据: b_ω 从无信息先验出发, 在 encoder+IMU
    双传感器把它变得【可观测】之后几秒内收敛到真值, Σ_bb 随之从 1e-2 量级塌缩。
    镜像了 sim_main 在 Rerun Time Series 里画的 /plots/bias_omega/{ekf,truth}。
    """
    t = df["t"]
    bias_est = df["ekf_bias_omega"]
    sigma_bb = df["ekf_sigma_bb"]
    std_bb = np.sqrt(sigma_bb.clip(lower=0.0))  # ±σ 带宽; clip 防极小负浮点

    fig, (ax_est, ax_cov) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    # --- 上: 估计值 + ±2σ 带 + 真值参考线 ---
    ax_est.plot(t, bias_est, color="#2ca02c", linewidth=1.8, label="ekf estimate")
    ax_est.fill_between(t, bias_est - 2.0 * std_bb, bias_est + 2.0 * std_bb,
                        color="#2ca02c", alpha=0.18, label="±2σ band")
    true_bias = recover_true_bias(metadata)
    if true_bias is not None:
        ax_est.axhline(true_bias, color="#d62728", linewidth=1.6, linestyle="--",
                       label=f"true bias = {true_bias:.3f} rad/s")
    ax_est.axhline(0.0, color="gray", linewidth=0.8, alpha=0.5)  # 先验起点 b₀ = 0
    ax_est.set_ylabel("gyro bias  b_ω  [rad/s]")
    ax_est.grid(True, alpha=0.3)
    ax_est.legend(loc="best")
    ax_est.set_title("Gyro bias estimate — converges once encoder + IMU make it observable")

    # --- 下: Σ_bb 塌缩(对数坐标) ---
    ax_cov.plot(t, sigma_bb, color="#9467bd", linewidth=1.8)
    ax_cov.set_yscale("log")
    ax_cov.set_ylabel("bias covariance\nΣ_bb  [(rad/s)²]")
    ax_cov.set_xlabel("time [s]")
    ax_cov.grid(True, alpha=0.3, which="both")
    ax_cov.set_title("Bias covariance — collapses from the uninformative prior (1e-2)")

    preset = metadata.get("preset", "?")
    seed = metadata.get("seed", "?")
    fig.suptitle(
        f"MiniNav V2 — gyro bias state augmentation (PR4)\n"
        f"preset = {preset}, seed = {seed}")

    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_state_errors(df: pd.DataFrame, metadata: dict[str, str], out_path: Path) -> None:
    """6 个状态维度的 estimation error + ±3σ 包络。

    一致的 filter: 误差曲线应大部分时间落在 ±3σ 带内(约 99.7%)。误差长期冲出包络
    说明 filter 过度自信(Σ 偏小)。这是"估计落在自身不确定性内"最直接的可视化。

    真值来源(均在 CSV 内):
      px,py,θ → truth_x/y/yaw;  v,ω → true_v/true_w(actuator 瞬时真值);
      b_ω → 由 preset 恢复的常数 bias(未知则跳过该维)。
    σ_ii 取协方差对角(σ_xy 不参与对角包络)。
    """
    t = df["t"].to_numpy()
    true_bias = recover_true_bias(metadata)

    # (标题, 估计列, 真值序列, 方差列, 单位, 是否角度)
    err_theta = wrap_to_pi((df["ekf_yaw"] - df["truth_yaw"]).to_numpy())
    panels = [
        ("p_x", df["ekf_x"].to_numpy() - df["truth_x"].to_numpy(), "ekf_sigma_xx", "m"),
        ("p_y", df["ekf_y"].to_numpy() - df["truth_y"].to_numpy(), "ekf_sigma_yy", "m"),
        ("θ", err_theta, "ekf_sigma_thth", "rad"),
        ("v", df["ekf_v"].to_numpy() - df["true_v"].to_numpy(), "ekf_sigma_vv", "m/s"),
        ("ω", df["ekf_omega"].to_numpy() - df["true_w"].to_numpy(), "ekf_sigma_ww", "rad/s"),
    ]
    if true_bias is not None:
        panels.append(
            ("b_ω", df["ekf_bias_omega"].to_numpy() - true_bias, "ekf_sigma_bb", "rad/s"))

    fig, axes = plt.subplots(2, 3, figsize=(14, 7), sharex=True)
    axes = axes.ravel()

    for ax, (name, err, sigma_col, unit) in zip(axes, panels):
        three_sigma = 3.0 * np.sqrt(np.clip(df[sigma_col].to_numpy(), 0.0, None))
        inside = float(np.mean(np.abs(err) <= three_sigma)) * 100.0

        ax.plot(t, err, color="#2ca02c", linewidth=1.0, label="error (est − truth)")
        ax.fill_between(t, -three_sigma, three_sigma, color="#1f77b4", alpha=0.18,
                        label="±3σ envelope")
        ax.axhline(0.0, color="gray", linewidth=0.7)
        ax.set_title(f"{name}: {inside:.1f}% within ±3σ")
        ax.set_ylabel(f"error [{unit}]")
        ax.grid(True, alpha=0.3)

    for ax in axes[len(panels):]:  # 无 bias 真值时隐藏多出的空子图
        ax.set_visible(False)
    axes[0].legend(loc="upper left", fontsize=8)
    for ax in axes[max(0, len(panels) - 3):len(panels)]:
        ax.set_xlabel("time [s]")

    preset = metadata.get("preset", "?")
    seed = metadata.get("seed", "?")
    fig.suptitle(
        f"MiniNav V2 — state estimation error vs ±3σ envelope\n"
        f"preset = {preset}, seed = {seed}  (error should stay within its own uncertainty)")
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_nis(df: pd.DataFrame, metadata: dict[str, str], out_path: Path) -> None:
    """encoder / IMU 的 NIS 时间序列 + χ² 95% 区间带。

    这是 filter consistency 的"招牌诊断图": NIS 主体应落在红色虚线区间内、
    围绕 dof 参考线波动。大量越界(尤其持续偏高)说明 Q/R 与真实噪声不匹配,
    是 --q-scale / --r-scale 调参的依据。
    """
    t = df["t"].to_numpy()
    fig, (ax_e, ax_i) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    for ax, col, dof, name, color in (
        (ax_e, "nis_encoder", 2, "encoder", "#2ca02c"),
        (ax_i, "nis_imu", 1, "IMU", "#1f77b4"),
    ):
        nis = df[col].to_numpy()
        mean, lo, hi, inside = nis_consistency(nis, dof)

        ax.plot(t, nis, color=color, linewidth=0.7, alpha=0.75, label=f"{name} NIS")
        ax.axhline(dof, color="gray", linewidth=1.0, label=f"dof = {dof}  (E[NIS])")
        ax.axhline(lo, color="#d62728", linewidth=1.0, linestyle="--")
        ax.axhline(hi, color="#d62728", linewidth=1.0, linestyle="--",
                   label="χ² 95% interval")
        ax.set_ylabel(f"{name} NIS")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)
        ax.set_title(
            f"{name}: mean = {mean:.2f} (expect {dof}); "
            f"{inside:.1f}% of samples within 95% χ² band")
        # NIS 偶有大尖峰(初始 transient / 转弯), 收紧 y 轴让主体可读。
        ax.set_ylim(0.0, max(hi * 1.8, float(np.percentile(nis, 99))))

    ax_i.set_xlabel("time [s]")
    preset = metadata.get("preset", "?")
    seed = metadata.get("seed", "?")
    q_scale = metadata.get("q_scale", "1")
    r_scale = metadata.get("r_scale", "1")
    fig.suptitle(
        f"MiniNav V2 — NIS consistency check (PR5c)\n"
        f"preset = {preset}, seed = {seed}, q_scale = {q_scale}, r_scale = {r_scale}")

    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def print_summary(df: pd.DataFrame, metadata: dict[str, str],
                  df_nobias: pd.DataFrame | None = None) -> None:
    mode = metadata.get("mode", "predict-only")
    print("=" * 64)
    print(f"MiniNav V2 EKF summary  [mode = {mode}]")
    print("=" * 64)
    for k, v in metadata.items():
        print(f"  {k:14s} = {v}")
    print("-" * 64)

    if mode == "encoder+imu":
        #  报告 odom / ekf-fused 各自对 truth 的全程 RMSE。
        dx_o = df["odom_x"] - df["truth_x"]; dy_o = df["odom_y"] - df["truth_y"]
        dy_o_yaw = wrap_to_pi((df["odom_yaw"] - df["truth_yaw"]).to_numpy())
        dx_e = df["ekf_x"] - df["truth_x"]; dy_e = df["ekf_y"] - df["truth_y"]
        dy_e_yaw = wrap_to_pi((df["ekf_yaw"] - df["truth_yaw"]).to_numpy())

        rmse_odom_pos = float(np.sqrt((dx_o ** 2 + dy_o ** 2).mean()))
        rmse_ekf_pos = float(np.sqrt((dx_e ** 2 + dy_e ** 2).mean()))
        rmse_odom_yaw = float(np.sqrt((dy_o_yaw ** 2).mean()))
        rmse_ekf_yaw = float(np.sqrt((dy_e_yaw ** 2).mean()))

        final_odom_err = float(np.hypot(dx_o.iloc[-1], dy_o.iloc[-1]))
        final_ekf_err = float(np.hypot(dx_e.iloc[-1], dy_e.iloc[-1]))

        print(f"  RMSE(truth, odom)        pos = {rmse_odom_pos:.4f} m,  "
              f"yaw = {np.degrees(rmse_odom_yaw):.3f} deg")
        print(f"  RMSE(truth, ekf fused)   pos = {rmse_ekf_pos:.4f} m,  "
              f"yaw = {np.degrees(rmse_ekf_yaw):.3f} deg")
        if df_nobias is not None:
            dx_nb = df_nobias["ekf_x"] - df_nobias["truth_x"]
            dy_nb = df_nobias["ekf_y"] - df_nobias["truth_y"]
            dy_nb_yaw = wrap_to_pi((df_nobias["ekf_yaw"] - df_nobias["truth_yaw"]).to_numpy())
            rmse_nb_pos = float(np.sqrt((dx_nb ** 2 + dy_nb ** 2).mean()))
            rmse_nb_yaw = float(np.sqrt((dy_nb_yaw ** 2).mean()))
            print(f"  RMSE(truth, ekf no-bias) pos = {rmse_nb_pos:.4f} m,  "
                  f"yaw = {np.degrees(rmse_nb_yaw):.3f} deg")
            if rmse_nb_pos > 0:
                print(f"  bias correction gain     pos: "
                      f"{100 * (1 - rmse_ekf_pos / rmse_nb_pos):.1f}%,  "
                      f"yaw: {100 * (1 - rmse_ekf_yaw / rmse_nb_yaw):.1f}%")
        if rmse_odom_pos > 0 and rmse_odom_yaw > 0:
            print(f"  fusion improvement       pos: "
                  f"{100 * (1 - rmse_ekf_pos / rmse_odom_pos):.1f}%,  "
                  f"yaw: {100 * (1 - rmse_ekf_yaw / rmse_odom_yaw):.1f}%")
        print(f"  final |odom - truth|     = {final_odom_err:.4f} m")
        print(f"  final |ekf  - truth|     = {final_ekf_err:.4f} m  "
              f"(IMU 把 yaw 锚定, 间接把位置漂移压下去)")

        # gyro bias 在线估计结果。
        bias_final = float(df["ekf_bias_omega"].iloc[-1])
        sbb0 = float(df["ekf_sigma_bb"].iloc[0])
        sbb_f = float(df["ekf_sigma_bb"].iloc[-1])
        print(f"  gyro bias estimate       final = {bias_final:.5f} rad/s")
        true_bias = recover_true_bias(metadata)
        if true_bias is not None:
            print(f"  gyro bias true (preset)        = {true_bias:.5f} rad/s  "
                  f"(|err| = {abs(bias_final - true_bias):.5f} rad/s)")
        shrink = f"{sbb0 / sbb_f:.0f}x 收缩" if sbb_f > 0 else "收缩"
        print(f"  bias covariance Σ_bb     = {sbb0:.4e} -> {sbb_f:.4e} (rad/s)²  "
              f"({shrink}; fusion 使 bias 可观测)")

        # NIS consistency(PR5c): mean ≈ dof 且约 95% 样本落入 χ² 区间 → filter 一致。
        if "nis_encoder" in df.columns and "nis_imu" in df.columns:
            enc_mean, _, _, enc_in = nis_consistency(df["nis_encoder"].to_numpy(), dof=2)
            imu_mean, _, _, imu_in = nis_consistency(df["nis_imu"].to_numpy(), dof=1)
            print(f"  NIS encoder (dof 2)      mean = {enc_mean:.3f},  "
                  f"{enc_in:.1f}% in 95% χ² band")
            print(f"  NIS imu     (dof 1)      mean = {imu_mean:.3f},  "
                  f"{imu_in:.1f}% in 95% χ² band")
    elif mode == "encoder-update":
        odom_err = np.hypot(df["odom_x"].iloc[-1] - df["truth_x"].iloc[-1],
                            df["odom_y"].iloc[-1] - df["truth_y"].iloc[-1])
        ekf_err = np.hypot(df["ekf_x"].iloc[-1] - df["truth_x"].iloc[-1],
                           df["ekf_y"].iloc[-1] - df["truth_y"].iloc[-1])
        max_ekf_odom = np.hypot(df["ekf_x"] - df["odom_x"], df["ekf_y"] - df["odom_y"]).max()

        pos0 = df["ekf_sigma_xx"].iloc[0] + df["ekf_sigma_yy"].iloc[0]
        pos_f = df["ekf_sigma_xx"].iloc[-1] + df["ekf_sigma_yy"].iloc[-1]
        vel = df["ekf_sigma_vv"] + df["ekf_sigma_ww"]
        vel_mid = vel.iloc[len(vel) // 2:].mean()

        print(f"  final |odom - truth|       = {odom_err:.4f} m")
        print(f"  final |ekf  - truth|       = {ekf_err:.4f} m  (≈ odom: 都在漂移)")
        print(f"  max   |ekf  - odom|        = {max_ekf_odom:.4e} m  "
              f"(退化为单传感器开环积分)")
        print(f"  position-block trace(Σ)    = {pos0:.4e} -> {pos_f:.4e} m²  "
              f"({pos_f / pos0:.0f}x 增长, 无界)")
        print(f"  velocity-block trace(Σ)    = {vel_mid:.4e}  (稳态, 有界)")

        sbb0 = df["ekf_sigma_bb"].iloc[0]
        sbb_f = df["ekf_sigma_bb"].iloc[-1]
        print(f"  bias-block Σ_bb            = {sbb0:.4e} -> {sbb_f:.4e}  "
              f"(几乎不收缩: 无 IMU 时 bias 不可观测)")
    else:
        trace0 = df[["ekf_sigma_xx", "ekf_sigma_yy", "ekf_sigma_thth",
                     "ekf_sigma_vv", "ekf_sigma_ww", "ekf_sigma_bb"]].iloc[0].sum()
        trace_f = df[["ekf_sigma_xx", "ekf_sigma_yy", "ekf_sigma_thth",
                      "ekf_sigma_vv", "ekf_sigma_ww", "ekf_sigma_bb"]].iloc[-1].sum()
        std_x0 = np.sqrt(df["ekf_sigma_xx"].iloc[0])
        std_xf = np.sqrt(df["ekf_sigma_xx"].iloc[-1])
        mean_drift = np.hypot(df["ekf_x"].iloc[-1] - df["ekf_x"].iloc[0],
                              df["ekf_y"].iloc[-1] - df["ekf_y"].iloc[0])
        print(f"  initial trace(Σ)      = {trace0:.4e}")
        print(f"  final   trace(Σ)      = {trace_f:.4e}")
        print(f"  σ_x growth            = {std_x0:.4e} m -> {std_xf:.4e} m "
              f"({std_xf / std_x0:.1f}×)")
        print(f"  EKF mean displacement = {mean_drift:.4e} m "
              f"(predict-only: 期望 ≈ 0, 均值冻结)")
    print("=" * 64)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default="data/traj.csv", type=Path)
    parser.add_argument("--output", default="results/v2/", type=Path)
    parser.add_argument(
        "--with-ellipses", action="store_true",
        help="(predict-only 模式) 额外输出位置协方差椭圆图。")
    parser.add_argument(
        "--ekf-no-bias", type=Path, default=None,
        help="(encoder+imu 模式) 同 seed/preset 的 --no-bias 运行 CSV; 给定时 RMSE 图叠加"
             " 第三条 'ekf (no bias)' 曲线, 形成 odom / ekf / ekf_with_bias 三方对比。")
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)

    df, metadata = load_trajectory(args.input)
    mode = metadata.get("mode", "predict-only")
    df_nobias = None

    if mode == "encoder+imu":
        # 可选: 读入 --no-bias 运行用于三方 RMSE 对比, 并校验同一个世界。
        if args.ekf_no_bias is not None:
            df_nobias, meta_nb = load_trajectory(args.ekf_no_bias)
            if meta_nb.get("seed") != metadata.get("seed") \
                    or meta_nb.get("preset") != metadata.get("preset"):
                raise SystemExit("--ekf-no-bias 的 seed/preset 与主输入不一致, 三方对比不成立。")
            if not check_same_world(df, df_nobias):
                raise SystemExit("--ekf-no-bias 的 truth/测量流与主输入不一致 (检查 seed)。")
            if meta_nb.get("bias") == "on":
                print("⚠ 警告: --ekf-no-bias 文件的元信息 bias=on, 它可能不是 --no-bias 运行。")

        # fusion 四轨迹叠加 + 累积 RMSE 时间序列(给定 df_nobias 则三方对比)。
        plot_fusion_trajectory(df, metadata, args.output / "fusion_trajectory.png")
        plot_fusion_rmse_over_time(df, metadata, args.output / "fusion_rmse_over_time.png",
                                   df_nobias=df_nobias)
        # 6 维状态 error + ±3σ 包络。
        plot_state_errors(df, metadata, args.output / "state_errors.png")
        # gyro bias 在线估计(收敛 + Σ_bb 塌缩)。
        plot_bias_learning(df, metadata, args.output / "bias_learning.png")
        # NIS consistency 检验(PR5c)。
        if "nis_encoder" in df.columns and "nis_imu" in df.columns:
            plot_nis(df, metadata, args.output / "nis_consistency.png")
    elif mode == "encoder-update":
        # 三轨迹叠加 + 协方差分离(observability)。
        plot_trajectory_overlay(df, metadata, args.output / "encoder_trajectory.png")
        plot_covariance_split(df, metadata, args.output / "encoder_covariance_split.png")
    else:
        # 协方差增长曲线(+ 可选椭圆)。
        plot_growth(df, metadata, args.output / "predict_only_growth.png")
        if args.with_ellipses:
            plot_ellipses(df, metadata, args.output / "predict_only_ellipses.png")

    print_summary(df, metadata, df_nobias)


if __name__ == "__main__":
    main()