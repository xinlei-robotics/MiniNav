#!/usr/bin/env python3
"""
run_scenarios.py — 批量跑 `sim nav`(场景 × seed × 参数),供 E3 / E4 分析(tracking_error.py)。

三组运行(默认全跑;都用 default 噪声档位,同一 seed 下不同参数面对同一组噪声实现 = CRN):

  e3     参数折中:S1–S5 × T_L × desired_linear_vel × seed 1–5,EKF 进控制器;另加经典 PP
         (固定 L_d、不限速、不原地转)对照。S5 不带地图、直接跟随它的平滑路径(--path):
         E3 只关心控制器负责的误差,office500 上 EKF 漂移导致的碰撞是 E4 的题目。
         每次运行读完就只留一行汇总 → data/v4/e3_summary.csv(原始 CSV 删除)。
  e4     误差分解:S1–S5 × seed 1–10 × {ekf, truth},默认参数、带地图(含碰撞判定);
         另加每个场景一次无噪声(--preset none)运行作对照。
         精简后的 nav.csv(.csv.gz)保留在 data/v4/e4/,汇总 → data/v4/e4_summary.csv。
  drift  漂移 vs 路程:S5 × seed 1–20,EKF 进控制器,(a) 带地图(第一次碰撞时走了多远),
         (b) 不带地图跟随同一条平滑路径(漂移曲线一直延伸到 34 m)。精简 CSV 在 data/v4/drift/。

Run:
    cmake --build --preset build-release -j
    python scripts/v4/run_scenarios.py            # 全部(约几分钟)
    python scripts/v4/run_scenarios.py e4 drift   # 只跑其中几组
"""

from __future__ import annotations

import argparse
import itertools
import os
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import pandas as pd

from _navio import DATA_DIR, SIM_DT, classic_pp, find_sim, plan_path, run_and_load, save_slim, traveled, \
    variant_configs
from scenarios import BY_KEY, SCENARIOS

E3_LOOKAHEAD_TIMES = (0.6, 0.7, 0.8, 1.0, 1.2, 1.4)
E3_VELOCITIES = (0.2, 0.3, 0.4)
E3_CLASSIC = (0.3, 0.5)  # 经典 PP 的固定 L_d(v = 0.3 m/s)
E3_SEEDS = range(1, 6)
E4_SEEDS = range(1, 11)
DRIFT_SEEDS = range(1, 21)
JITTER_WINDOW_S = 1.0


def omega_jitter(df: pd.DataFrame) -> float:
    """ω 指令的高频分量 RMS:减去 1 s 滑动平均后的均方根(rad/s)。转弯本身是低频的,
    留下来的是控制器对噪声 / 滞后的抖动。"""
    w = df["cmd_w"].to_numpy()
    n = max(1, int(round(JITTER_WINDOW_S / SIM_DT)))
    if len(w) <= n:
        return float("nan")
    smooth = np.convolve(w, np.ones(n) / n, mode="same")
    core = slice(n // 2, len(w) - n // 2)
    return float(np.sqrt(np.mean((w[core] - smooth[core]) ** 2)))


def summarize(meta: dict, df: pd.DataFrame) -> dict:
    s = traveled(df)
    moving = df["true_v"].abs().to_numpy() > 1e-3
    return {
        "status": meta["status"],
        "end_time_s": meta["end_time_s"],
        "path_length_m": meta["path_length_m"],
        "traveled_m": float(s[-1]) if len(s) else 0.0,
        "goal_error_true_m": meta["goal_error_true_m"],
        "goal_error_input_m": meta["goal_error_input_m"],
        "e_ctrl_mean": float(df["e_ctrl"].mean()),
        "e_ctrl_p95": float(df["e_ctrl"].quantile(0.95)),
        "e_ctrl_max": float(df["e_ctrl"].max()),
        "e_true_mean": float(df["e_true"].mean()),
        "e_true_max": float(df["e_true"].max()),
        "e_est_final": float(df["e_est"].iloc[-1]),
        "min_clearance_m": meta["min_clearance_m"],
        "omega_jitter": omega_jitter(df),
        "omega_rms": float(np.sqrt(np.mean(df["cmd_w"].to_numpy()[moving] ** 2))) if moving.any() else 0.0,
        "nis_encoder_mean": float(df["nis_encoder"].mean()),
        "nis_imu_mean": float(df["nis_imu"].mean()),
    }


# ---- 单次运行(进程池里执行)--------------------------------------------------------

def _job(job: dict) -> dict:
    sim = Path(job.pop("sim"))
    keep = job.pop("keep")
    tags = job.pop("tags")
    out = Path(job.pop("out"))
    meta, df = run_and_load(sim, out, keep=False, **job)
    if keep:
        save_slim(meta, df, out.with_suffix(".csv.gz"))
    return {**tags, **summarize(meta, df)}


def run_jobs(jobs: list[dict], label: str) -> pd.DataFrame:
    workers = max(1, (os.cpu_count() or 2) - 1)
    print(f"{label}: {len(jobs)} runs on {workers} workers ...", flush=True)
    with ProcessPoolExecutor(max_workers=workers) as pool:
        rows = list(pool.map(_job, jobs, chunksize=4))
    return pd.DataFrame(rows)


def scenario_args(key: str, follow_s5: bool) -> dict:
    sc = BY_KEY[key]
    if key == "S5" and follow_s5:
        return {"path_csv": DATA_DIR / "paths" / "s5.csv"}
    return sc.nav_args()


def ensure_paths(sim: Path) -> None:
    for sc in SCENARIOS:
        plan_path(sim, sc.map_yaml, sc.start, sc.goal, DATA_DIR / "paths" / f"{sc.key.lower()}.csv")


# ---- 三组运行 -------------------------------------------------------------------------

def run_e3(sim: Path) -> None:
    configs = []
    for t_l, v in itertools.product(E3_LOOKAHEAD_TIMES, E3_VELOCITIES):
        name = f"e3_TL{t_l}_v{v}"
        _, nav = variant_configs(name, nav={"controller": {"lookahead_time": t_l, "desired_linear_vel": v}})
        configs.append(({"controller": "rpp", "lookahead_time": t_l, "velocity": v, "lookahead_dist": np.nan},
                        nav))
    for L in E3_CLASSIC:
        name = f"e3_classic_L{L}"
        _, nav = variant_configs(name, nav={"controller": classic_pp(L, 0.3)})
        configs.append(({"controller": "classic", "lookahead_time": np.nan, "velocity": 0.3,
                         "lookahead_dist": L}, nav))
    jobs = []
    for (tags, nav), sc, seed in itertools.product(configs, SCENARIOS, E3_SEEDS):
        out = DATA_DIR / "e3" / f"{sc.key}_{nav.stem}_seed{seed}.csv"
        jobs.append({"sim": str(sim), "keep": False, "out": str(out),
                     "tags": {**tags, "scenario": sc.key, "seed": seed},
                     **scenario_args(sc.key, follow_s5=True), "seed": seed, "preset": "default",
                     "controller_input": "ekf", "nav": nav})
    df = run_jobs(jobs, "e3")
    df.to_csv(DATA_DIR / "e3_summary.csv", index=False)
    print(f"wrote {DATA_DIR / 'e3_summary.csv'}")


def run_e4(sim: Path) -> None:
    jobs = []
    for sc, seed, ci in itertools.product(SCENARIOS, E4_SEEDS, ("ekf", "truth")):
        out = DATA_DIR / "e4" / f"{sc.key}_{ci}_seed{seed}.csv"
        jobs.append({"sim": str(sim), "keep": True, "out": str(out),
                     "tags": {"preset": "default", "scenario": sc.key, "seed": seed, "controller_input": ci},
                     **sc.nav_args(), "seed": seed, "preset": "default", "controller_input": ci})
    # 无噪声对照(§0.4 的"无噪声零碰撞、100% 到达"):每个场景一次,只留汇总。
    for sc, ci in itertools.product(SCENARIOS, ("ekf", "truth")):
        out = DATA_DIR / "e4" / f"none_{sc.key}_{ci}.csv"
        jobs.append({"sim": str(sim), "keep": False, "out": str(out),
                     "tags": {"preset": "none", "scenario": sc.key, "seed": 1, "controller_input": ci},
                     **sc.nav_args(), "seed": 1, "preset": "none", "controller_input": ci})
    df = run_jobs(jobs, "e4")
    df.to_csv(DATA_DIR / "e4_summary.csv", index=False)
    print(f"wrote {DATA_DIR / 'e4_summary.csv'}")


def run_drift(sim: Path) -> None:
    jobs = []
    for seed, with_map in itertools.product(DRIFT_SEEDS, (True, False)):
        mode = "map" if with_map else "follow"
        out = DATA_DIR / "drift" / f"S5_{mode}_seed{seed}.csv"
        jobs.append({"sim": str(sim), "keep": True, "out": str(out),
                     "tags": {"scenario": "S5", "seed": seed, "mode": mode},
                     **scenario_args("S5", follow_s5=not with_map), "seed": seed, "preset": "default",
                     "controller_input": "ekf"})
    df = run_jobs(jobs, "drift")
    df.to_csv(DATA_DIR / "drift_summary.csv", index=False)
    print(f"wrote {DATA_DIR / 'drift_summary.csv'}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("groups", nargs="*", choices=("e3", "e4", "drift"), help="default: all")
    args = ap.parse_args()
    sim = find_sim()
    ensure_paths(sim)
    groups = args.groups or ["e3", "e4", "drift"]
    runners = {"e3": run_e3, "e4": run_e4, "drift": run_drift}
    for g in groups:
        runners[g](sim)


if __name__ == "__main__":
    main()
