#!/usr/bin/env python3
"""
RK4 vs Euler 归因实验。

比较 EKF 内部过程模型用 RK4 与用一阶欧拉时, 估计精度的差异。两个输入 CSV 必须
由同一 seed、同一 preset、仅 --integrator 不同的两次 sim 运行产生:

    ./build/clang18-debug/sim --no-viz --seed 42 --preset default \\
        --integrator euler --out data/traj_euler.csv
    ./build/clang18-debug/sim --no-viz --seed 42 --preset default \\
        --integrator rk4   --out data/traj_rk4.csv
    python scripts/v2/analyze_integrator.py \\
        --euler data/traj_euler.csv --rk4 data/traj_rk4.csv --output results/v2/

因为 EKF 不消耗 RNG, 两次运行的 truth / encoder / IMU 流逐位相同, 估计差异只来自积分器。

产出:
  - results/v2/integrator_rmse.png   累积 position / yaw RMSE 时间序列 (euler vs rk4)
  - stdout 摘要: 全程 RMSE 与 RK4 相对 Euler 的增益
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def wrap_to_pi(angle: np.ndarray) -> np.ndarray:
    """规范化到 (-pi, pi], 与 C++ 端 atan2(sin, cos) trick 一致。"""
    return np.arctan2(np.sin(angle), np.cos(angle))


def parse_metadata(path: Path) -> dict[str, str]:
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


def load(path: Path) -> tuple[pd.DataFrame, dict[str, str]]:
    return pd.read_csv(path, comment="#"), parse_metadata(path)


def rmse_series(df: pd.DataFrame) -> dict[str, np.ndarray]:
    """ekf 相对 truth 的累积(prefix)position / yaw RMSE 时间序列。"""
    dx = df["ekf_x"] - df["truth_x"]
    dy = df["ekf_y"] - df["truth_y"]
    dyaw = wrap_to_pi((df["ekf_yaw"] - df["truth_yaw"]).to_numpy())
    n = np.arange(1, len(df) + 1)
    return {
        "pos": np.sqrt((dx ** 2 + dy ** 2).cumsum() / n),
        "yaw": np.sqrt((dyaw ** 2).cumsum() / n),
    }


def overall_rmse(df: pd.DataFrame) -> tuple[float, float]:
    dx = df["ekf_x"] - df["truth_x"]
    dy = df["ekf_y"] - df["truth_y"]
    dyaw = wrap_to_pi((df["ekf_yaw"] - df["truth_yaw"]).to_numpy())
    pos = float(np.sqrt((dx ** 2 + dy ** 2).mean()))
    yaw = float(np.sqrt((dyaw ** 2).mean()))
    return pos, yaw


def check_same_world(euler: pd.DataFrame, rk4: pd.DataFrame) -> bool:
    """两次运行的 truth / 测量流应逐位相同(同 seed ⇒ 同世界); 否则归因不成立。"""
    if len(euler) != len(rk4):
        return False
    cols = ["truth_x", "truth_y", "truth_yaw", "imu_omega", "enc_dl", "enc_dr"]
    return all(np.allclose(euler[c], rk4[c], rtol=0, atol=0) for c in cols)


def plot_rmse(euler: pd.DataFrame, rk4: pd.DataFrame, meta: dict[str, str],
              out_path: Path) -> None:
    re = rmse_series(euler)
    rr = rmse_series(rk4)
    t = euler["t"]

    fig, (ax_pos, ax_yaw) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    ax_pos.plot(t, re["pos"], color="#ff7f0e", linewidth=1.8, label="EKF / Euler")
    ax_pos.plot(t, rr["pos"], color="#2ca02c", linewidth=1.8, label="EKF / RK4")
    ax_pos.set_ylabel("cumulative position RMSE [m]")
    ax_pos.grid(True, alpha=0.3)
    ax_pos.legend(loc="best")
    ax_pos.set_title("Position RMSE vs truth — integrator attribution")

    ax_yaw.plot(t, np.degrees(re["yaw"]), color="#ff7f0e", linewidth=1.8, label="EKF / Euler")
    ax_yaw.plot(t, np.degrees(rr["yaw"]), color="#2ca02c", linewidth=1.8, label="EKF / RK4")
    ax_yaw.set_ylabel("cumulative yaw RMSE [deg]")
    ax_yaw.set_xlabel("time [s]")
    ax_yaw.grid(True, alpha=0.3)
    ax_yaw.legend(loc="best")
    ax_yaw.set_title("Yaw RMSE vs truth")

    preset = meta.get("preset", "?")
    seed = meta.get("seed", "?")
    fig.suptitle(
        f"MiniNav V2 — EKF integrator attribution (RK4 vs Euler)\n"
        f"preset = {preset}, seed = {seed}  "
        f"(same truth + measurements; only the EKF process model differs)")

    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--euler", default="data/traj_euler.csv", type=Path)
    parser.add_argument("--rk4", default="data/traj_rk4.csv", type=Path)
    parser.add_argument("--output", default="results/v2/", type=Path)
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)

    euler, meta_e = load(args.euler)
    rk4, meta_r = load(args.rk4)

    if meta_e.get("seed") != meta_r.get("seed") or meta_e.get("preset") != meta_r.get("preset"):
        raise SystemExit(
            f"两个 CSV 的 seed/preset 不一致, 归因不成立: "
            f"euler=({meta_e.get('seed')},{meta_e.get('preset')}) "
            f"rk4=({meta_r.get('seed')},{meta_r.get('preset')})")
    if meta_e.get("integrator") != "euler" or meta_r.get("integrator") != "rk4":
        print(f"⚠ 警告: integrator 元信息异常 "
              f"(euler 文件={meta_e.get('integrator')}, rk4 文件={meta_r.get('integrator')})")
    if not check_same_world(euler, rk4):
        raise SystemExit("两次运行的 truth/测量流不一致 — 无法归因到积分器。检查 seed。")

    plot_rmse(euler, rk4, meta_e, args.output / "integrator_rmse.png")

    pos_e, yaw_e = overall_rmse(euler)
    pos_r, yaw_r = overall_rmse(rk4)

    def gain(base: float, new: float) -> float:
        return 100.0 * (1.0 - new / base) if base > 0 else 0.0

    print("=" * 68)
    print(f"MiniNav V2 — RK4 vs Euler 积分器归因  "
          f"[preset = {meta_e.get('preset')}, seed = {meta_e.get('seed')}]")
    print("=" * 68)
    print(f"  {'':22s}{'position RMSE [m]':>20s}{'yaw RMSE [deg]':>18s}")
    print(f"  {'EKF / Euler':22s}{pos_e:>20.5f}{np.degrees(yaw_e):>18.4f}")
    print(f"  {'EKF / RK4':22s}{pos_r:>20.5f}{np.degrees(yaw_r):>18.4f}")
    print("-" * 68)
    print(f"  RK4 相对 Euler 增益     position: {gain(pos_e, pos_r):+.2f}%,  "
          f"yaw: {gain(yaw_e, yaw_r):+.2f}%")
    print("-" * 68)
    print("  注 (inverse crime): truth 由 RK4 生成, RK4-EKF 与之共用积分器; 且 dt=0.01")
    print("       每步都有观测, 积分误差被传感器噪声主导。故此增益为下界。")
    print("=" * 68)


if __name__ == "__main__":
    main()
