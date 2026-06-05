#!/usr/bin/env python3
"""
Analyze MiniNav V2 EKF behaviour (mode-aware).

读 data/traj_v2.csv, 根据 CSV 头部的 `# mode` 元信息选择产图:

mode = predict-only:
  - results/v2_predict_only_growth.png    trace(Σ) 与位置块特征值的无界增长
  - results/v2_predict_only_ellipses.png  (--with-ellipses) 位置 2σ 椭圆膨胀

mode = encoder-update:
  - results/v2_encoder_trajectory.png        truth / odom / ekf 三轨迹叠加。
      ekf 与 odom 几乎重合(encoder-only EKF = dead reckoning), 两者都随时间
      偏离 truth —— encoder 约束不了位置漂移。
  - results/v2_encoder_covariance_split.png  速度块 vs 位置块协方差分离。

mode = encoder+imu:
  - results/v2_fusion_trajectory.png    truth / odom / ekf-encoder-only /
      ekf-fused 四轨迹叠加。fused 轨迹明显贴近 truth, 而 odom/encoder-only
      因 dead reckoning 偏离, IMU 让位置不再无界发散。
      注: encoder-only 不在 CSV 里, 但 odom 提供"无 IMU" 基线;
      fused 与 odom 的差距就是 IMU 的贡献。
  - results/v2_fusion_rmse_over_time.png  RMSE(truth, odom) 与 RMSE(truth, ekf)
      时间序列, 直观展示 fused 在转弯段开始后误差不再发散。
  - results/v2_bias_learning.png:
      上图 b_ω 估计 + ±2σ 带 + 真值参考线(由 preset 恢复), 几秒内收敛到真值;
      下图 Σ_bb 从无信息先验(1e-2)对数坐标下塌缩 —— encoder+IMU 让 bias 可观测
      的最直观证据(state augmentation 的"招牌图")。

无论哪种模式都打印 stdout summary。

Run:
    python scripts/analyze_v2_ekf.py --input data/traj_v2.csv --output results/
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


# 各 preset 注入的常数gyro bias 真值, 镜像 sim_v2_main.cpp 的 V2Preset.imu_bias_init。
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


def plot_fusion_rmse_over_time(df: pd.DataFrame, metadata: dict[str, str],
                               out_path: Path) -> None:
    """累积 RMSE(truth, odom) vs RMSE(truth, ekf-fused) 时间序列。"""
    # 累积 RMSE 用滑窗会更平滑, 但用 prefix RMSE 更直观地展示"误差长期累积"。
    dx_odom = df["odom_x"] - df["truth_x"]
    dy_odom = df["odom_y"] - df["truth_y"]
    dy_odom_yaw = wrap_to_pi((df["odom_yaw"] - df["truth_yaw"]).to_numpy())

    dx_ekf = df["ekf_x"] - df["truth_x"]
    dy_ekf = df["ekf_y"] - df["truth_y"]
    dy_ekf_yaw = wrap_to_pi((df["ekf_yaw"] - df["truth_yaw"]).to_numpy())

    n = np.arange(1, len(df) + 1)
    rmse_odom_pos = np.sqrt((dx_odom ** 2 + dy_odom ** 2).cumsum() / n)
    rmse_ekf_pos = np.sqrt((dx_ekf ** 2 + dy_ekf ** 2).cumsum() / n)
    rmse_odom_yaw = np.sqrt((dy_odom_yaw ** 2).cumsum() / n)
    rmse_ekf_yaw = np.sqrt((dy_ekf_yaw ** 2).cumsum() / n)

    fig, (ax_pos, ax_yaw) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    ax_pos.plot(df["t"], rmse_odom_pos, color="#ff7f0e", linewidth=1.8,
                label="odom (encoder only)")
    ax_pos.plot(df["t"], rmse_ekf_pos, color="#2ca02c", linewidth=1.8,
                label="ekf fused (encoder + IMU)")
    ax_pos.set_ylabel("cumulative position RMSE [m]")
    ax_pos.grid(True, alpha=0.3)
    ax_pos.legend(loc="best")
    ax_pos.set_title("Position RMSE — IMU fusion stops the drift at the turn")

    ax_yaw.plot(df["t"], np.degrees(rmse_odom_yaw), color="#ff7f0e", linewidth=1.8,
                label="odom")
    ax_yaw.plot(df["t"], np.degrees(rmse_ekf_yaw), color="#2ca02c", linewidth=1.8,
                label="ekf fused")
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
    镜像了 sim_v2_main 在 Rerun Time Series 里画的 /plots/bias_omega/{ekf,truth}。
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


def print_summary(df: pd.DataFrame, metadata: dict[str, str]) -> None:
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
    parser.add_argument("--input", default="data/traj_v2.csv", type=Path)
    parser.add_argument("--output", default="results/", type=Path)
    parser.add_argument(
        "--with-ellipses", action="store_true",
        help="(predict-only 模式) 额外输出位置协方差椭圆图。")
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)

    df, metadata = load_trajectory(args.input)
    mode = metadata.get("mode", "predict-only")

    if mode == "encoder+imu":
        # fusion 四轨迹叠加 + 累积 RMSE 时间序列。
        plot_fusion_trajectory(df, metadata, args.output / "v2_fusion_trajectory.png")
        plot_fusion_rmse_over_time(df, metadata, args.output / "v2_fusion_rmse_over_time.png")
        # gyro bias 在线估计(收敛 + Σ_bb 塌缩)。
        plot_bias_learning(df, metadata, args.output / "v2_bias_learning.png")
    elif mode == "encoder-update":
        # 三轨迹叠加 + 协方差分离(observability)。
        plot_trajectory_overlay(df, metadata, args.output / "v2_encoder_trajectory.png")
        plot_covariance_split(df, metadata, args.output / "v2_encoder_covariance_split.png")
    else:
        # 协方差增长曲线(+ 可选椭圆)。
        plot_growth(df, metadata, args.output / "v2_predict_only_growth.png")
        if args.with_ellipses:
            plot_ellipses(df, metadata, args.output / "v2_predict_only_ellipses.png")

    print_summary(df, metadata)


if __name__ == "__main__":
    main()