#!/usr/bin/env python3
"""
RK4 vs Euler 归因实验 —— 多 seed 扫描求平均 (V2 PR5a)。

对每个 seed 各跑一次 sim(--integrator euler / rk4, 其余相同), 计算 EKF 相对 truth
的 position / yaw RMSE, 然后在 seed 之间求平均(+ 标准差)。单 seed 的随机性会让
~1% 量级的积分器增益淹没在噪声里; 多 seed 平均才能稳定地分离出"积分器本身"的贡献。

为什么平均有效: 每个 seed 下 euler 与 rk4 共享同一条 truth + 测量流(EKF 不耗 RNG),
所以【每个 seed 内】的增益是干净归因; 跨 seed 平均只是降低对单条轨迹的偶然依赖。

Run:
    python scripts/v2/sweep_integrator.py --n-seeds 50 --preset default --output results/v2/
    # 或显式给定 seeds:
    python scripts/v2/sweep_integrator.py --seeds 1 7 42 100 --preset high-noise

⚠ inverse crime 等同 analyze_integrator.py 的交代: truth 由 RK4 生成, 故增益是下界。

产出:
  - results/v2/integrator_sweep.png  跨 seed 平均的累积 RMSE(±1σ 带, euler vs rk4)
  - stdout: 每 seed 表 + 平均 ± 标准差 + 平均增益
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

# 复用单对分析脚本里的 RMSE 工具(同目录)。
sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_integrator import load, overall_rmse, rmse_series, check_same_world  # noqa: E402


def run_sim(binary: Path, seed: int, preset: str, integrator: str, out: Path) -> None:
    """跑一次 sim(headless), 失败则抛错。"""
    cmd = [
        str(binary), "--no-viz", "--seed", str(seed), "--preset", preset,
        "--integrator", integrator, "--out", str(out),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(f"sim failed (seed={seed}, integrator={integrator}):\n{proc.stderr}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bin", type=Path,
                        default=Path("build/clang18-debug/sim"),
                        help="sim 可执行文件路径。")
    parser.add_argument("--preset", default="default",
                        choices=["low-noise", "default", "high-noise"])
    parser.add_argument("--seeds", type=int, nargs="+", default=None,
                        help="显式 seed 列表; 缺省用 0..n-seeds-1。")
    parser.add_argument("--n-seeds", type=int, default=50,
                        help="--seeds 未给时, 使用 seeds = 0..N-1。")
    parser.add_argument("--output", default="results/v2/", type=Path)
    args = parser.parse_args()

    if not args.bin.exists():
        raise SystemExit(f"找不到 sim: {args.bin} (先 cmake --build --preset build-debug)")

    seeds = args.seeds if args.seeds is not None else list(range(args.n_seeds))
    args.output.mkdir(parents=True, exist_ok=True)

    # 逐 seed 累积标量 RMSE 与累积曲线。
    rows: list[tuple[int, float, float, float, float]] = []
    pos_curves_e, pos_curves_r = [], []
    yaw_curves_e, yaw_curves_r = [], []
    t_axis = None

    with tempfile.TemporaryDirectory(prefix="v2_integ_sweep_") as tmp:
        tmp_dir = Path(tmp)
        for seed in seeds:
            csv_e = tmp_dir / f"euler_{seed}.csv"
            csv_r = tmp_dir / f"rk4_{seed}.csv"
            run_sim(args.bin, seed, args.preset, "euler", csv_e)
            run_sim(args.bin, seed, args.preset, "rk4", csv_r)

            df_e, _ = load(csv_e)
            df_r, _ = load(csv_r)
            if not check_same_world(df_e, df_r):
                raise SystemExit(f"seed={seed}: euler/rk4 的 truth/测量流不一致, 归因不成立。")

            pos_e, yaw_e = overall_rmse(df_e)
            pos_r, yaw_r = overall_rmse(df_r)
            rows.append((seed, pos_e, pos_r, yaw_e, yaw_r))

            se, sr = rmse_series(df_e), rmse_series(df_r)
            pos_curves_e.append(se["pos"]); pos_curves_r.append(sr["pos"])
            yaw_curves_e.append(se["yaw"]); yaw_curves_r.append(sr["yaw"])
            if t_axis is None:
                t_axis = df_e["t"].to_numpy()

    arr = np.array([(p_e, p_r, y_e, y_r) for _, p_e, p_r, y_e, y_r in rows])
    pos_e_all, pos_r_all, yaw_e_all, yaw_r_all = arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]

    # 每 seed 增益(先比再平均, 比"先平均再比"更能反映典型表现)。
    pos_gain = 100.0 * (1.0 - pos_r_all / pos_e_all)
    yaw_gain = 100.0 * (1.0 - yaw_r_all / np.where(yaw_e_all > 0, yaw_e_all, np.nan))

    # ---- stdout 表 ----
    print("=" * 78)
    print(f"MiniNav V2 — RK4 vs Euler 积分器归因 (多 seed 平均)  "
          f"[preset = {args.preset}, n = {len(seeds)}]")
    print("=" * 78)
    print(f"  {'seed':>6} {'pos_euler[m]':>13} {'pos_rk4[m]':>12} {'pos_gain%':>10} "
          f"{'yaw_euler°':>11} {'yaw_rk4°':>10} {'yaw_gain%':>10}")
    for (seed, p_e, p_r, y_e, y_r), pg, yg in zip(rows, pos_gain, yaw_gain):
        yg_str = f"{yg:+.3f}" if np.isfinite(yg) else "   n/a"
        print(f"  {seed:>6} {p_e:>13.5f} {p_r:>12.5f} {pg:>+10.3f} "
              f"{np.degrees(y_e):>11.4f} {np.degrees(y_r):>10.4f} {yg_str:>10}")
    print("-" * 78)

    def ms(x: np.ndarray) -> str:
        return f"{x.mean():.5f} ± {x.std(ddof=1):.5f}" if len(x) > 1 else f"{x.mean():.5f}"

    print(f"  position RMSE [m]   Euler: {ms(pos_e_all)}    RK4: {ms(pos_r_all)}")
    print(f"  yaw RMSE [deg]      Euler: {ms(np.degrees(yaw_e_all))}    "
          f"RK4: {ms(np.degrees(yaw_r_all))}")
    print("-" * 78)
    pg_std = f" ± {pos_gain.std(ddof=1):.3f}" if len(pos_gain) > 1 else ""
    print(f"  平均 RK4 增益(每 seed 先比再平均)  position: {pos_gain.mean():+.3f}%{pg_std}")
    if np.isfinite(yaw_gain).any():
        yg_valid = yaw_gain[np.isfinite(yaw_gain)]
        yg_std = f" ± {yg_valid.std(ddof=1):.3f}" if len(yg_valid) > 1 else ""
        print(f"  {'':40} yaw:      {yg_valid.mean():+.3f}%{yg_std}")
    print("-" * 78)
    print("  注 (inverse crime): truth 由 RK4 生成 → 增益为下界; dt=0.01 且每步观测,")
    print("       积分误差被传感器噪声主导。详见 analyze_integrator.py。")
    print("=" * 78)

    # ---- 平均累积 RMSE 曲线(±1σ 带) ----
    pos_e_stack = np.vstack(pos_curves_e); pos_r_stack = np.vstack(pos_curves_r)
    yaw_e_stack = np.degrees(np.vstack(yaw_curves_e)); yaw_r_stack = np.degrees(np.vstack(yaw_curves_r))

    fig, (ax_pos, ax_yaw) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    def band(ax, t, stack, color, label):
        m, s = stack.mean(axis=0), stack.std(axis=0)
        ax.plot(t, m, color=color, linewidth=1.8, label=label)
        ax.fill_between(t, m - s, m + s, color=color, alpha=0.15)

    band(ax_pos, t_axis, pos_e_stack, "#ff7f0e", "EKF / Euler")
    band(ax_pos, t_axis, pos_r_stack, "#2ca02c", "EKF / RK4")
    ax_pos.set_ylabel("cumulative position RMSE [m]")
    ax_pos.grid(True, alpha=0.3); ax_pos.legend(loc="best")
    ax_pos.set_title(f"Position RMSE — mean ±1σ over {len(seeds)} seeds")

    band(ax_yaw, t_axis, yaw_e_stack, "#ff7f0e", "EKF / Euler")
    band(ax_yaw, t_axis, yaw_r_stack, "#2ca02c", "EKF / RK4")
    ax_yaw.set_ylabel("cumulative yaw RMSE [deg]")
    ax_yaw.set_xlabel("time [s]")
    ax_yaw.grid(True, alpha=0.3); ax_yaw.legend(loc="best")
    ax_yaw.set_title("Yaw RMSE — mean ±1σ (integrator does not touch heading)")

    fig.suptitle(
        f"MiniNav V2 — EKF integrator attribution, averaged over {len(seeds)} seeds\n"
        f"preset = {args.preset}  (per-seed: same truth + measurements; only EKF model differs)")
    fig.tight_layout()
    out_png = args.output / "integrator_sweep.png"
    fig.savefig(out_png, dpi=140)
    plt.close(fig)
    print(f"  图已写入 {out_png}")


if __name__ == "__main__":
    main()
