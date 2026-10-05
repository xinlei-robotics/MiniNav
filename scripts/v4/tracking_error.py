#!/usr/bin/env python3
"""
tracking_error.py — E3 / E4 的统计与图(数据由 run_scenarios.py 产出,docs/experiments/v4_control.md §5–§6)。

  E3  参数折中:e_ctrl 均值 / 峰值、ω 抖动、完成时间、最小净空 vs T_L × v_des,经典 PP 对照;
      与默认参数(T_L = 1.0 s、v = 0.3 m/s)的同 seed 配对差(CRN)。
  E4  误差分解:§0.4 验收表(e_ctrl 均值 ≤ 10 cm、峰值 ≤ 30 cm;oracle / 无噪声零碰撞、
      100% 到达);逐样本检查 e_true ≤ e_ctrl + e_est;一次运行的三种误差时序与分布。
      漂移 vs 路程:S5(34 m)上 e_est 随路程的增长、真值到达误差、第一次碰撞的路程。
      闭环 NIS / NEES:按工况(加减速 / 巡航 / 原地转)分开看编码器、陀螺的 NIS,
      20 次运行平均的位置 NEES 对 χ² 区间。

产出:
  results/v4/e3_tradeoff.png
  results/v4/e4_error_decomposition.png
  results/v4/e4_drift.png
  results/v4/e4_consistency.png
  stdout                                各表

Run:
    python scripts/v4/run_scenarios.py
    python scripts/v4/tracking_error.py
"""

from __future__ import annotations

import argparse
import math

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from _navio import CHI2_95, DATA_DIR, RESULTS_DIR, SIM_DT, chi2_quantile, read_nav_csv, traveled
from scenarios import SCENARIOS

M_EFF = 0.25 - math.hypot(0.11, 0.08) - math.sqrt(2.0) * 0.05  # ≈ 0.043 m(§5,config 默认值)
DEFAULT = (1.0, 0.3)
V2_OPEN_LOOP_NIS = {"encoder": 4.66, "imu": 1.81}  # docs/experiments/v2_ekf_fusion.md §4.3(default,20 seed)
SCEN_COLORS = {"S1": "#1f77b4", "S2": "#ff7f0e", "S3": "#2ca02c", "S4": "#d62728", "S5": "#9467bd"}

pd.set_option("display.width", 200)
pd.set_option("display.max_columns", 30)


# ---- E3 -----------------------------------------------------------------------------

def e3() -> None:
    path = DATA_DIR / "e3_summary.csv"
    if not path.exists():
        print(f"skip E3: {path} missing (python scripts/v4/run_scenarios.py e3)")
        return
    d = pd.read_csv(path)
    d["time_per_m"] = d["end_time_s"] / d["path_length_m"]
    d["arrived"] = d["status"] == "arrived"
    d["collided"] = d["status"] == "collision"
    on_map = d["scenario"] != "S5"  # S5 在 E3 里不带地图(净空无定义)

    rpp = d[d["controller"] == "rpp"]
    agg = rpp.groupby(["lookahead_time", "velocity"]).agg(
        arrived=("arrived", "mean"), collisions=("collided", "sum"),
        e_ctrl_mean=("e_ctrl_mean", "mean"), e_ctrl_max=("e_ctrl_max", "max"),
        jitter=("omega_jitter", "mean"), time_per_m=("time_per_m", "mean"))
    clear = rpp[on_map[rpp.index]].groupby(["lookahead_time", "velocity"])["min_clearance_m"].min()
    agg["min_clearance"] = clear
    classic = d[d["controller"] == "classic"].groupby("lookahead_dist").agg(
        arrived=("arrived", "mean"), collisions=("collided", "sum"),
        e_ctrl_mean=("e_ctrl_mean", "mean"), e_ctrl_max=("e_ctrl_max", "max"),
        jitter=("omega_jitter", "mean"), time_per_m=("time_per_m", "mean"))
    classic["min_clearance"] = d[(d["controller"] == "classic") & on_map].groupby("lookahead_dist")[
        "min_clearance_m"].min()

    print("\nE3 — S1–S5 × 5 seeds, default noise, EKF into the controller "
          "(S5 follows its smoothed path without the map)")
    show = agg.copy()
    for col in ("e_ctrl_mean", "e_ctrl_max", "min_clearance"):
        show[col] = (100 * show[col]).round(2)
    show["arrived"] = (100 * show["arrived"]).round(0)
    print("RPP  (e_ctrl / clearance in cm, jitter in rad/s, arrived in %)")
    print(show.round(3).to_string())
    show = classic.copy()
    for col in ("e_ctrl_mean", "e_ctrl_max", "min_clearance"):
        show[col] = (100 * show[col]).round(2)
    show["arrived"] = (100 * show["arrived"]).round(0)
    print("classic PP (v = 0.3 m/s, fixed L_d)")
    print(show.round(3).to_string())

    # CRN 配对差:同一 (scenario, seed) 与默认参数相减。
    base = rpp[(rpp["lookahead_time"] == DEFAULT[0]) & (rpp["velocity"] == DEFAULT[1])].set_index(
        ["scenario", "seed"])
    print(f"\npaired differences vs default (T_L = {DEFAULT[0]} s, v = {DEFAULT[1]} m/s), mean ± std over "
          "25 (scenario, seed) pairs")
    print(f"{'T_L':>4} {'v':>4} | {'Δe_ctrl_mean [mm]':>18} | {'Δe_ctrl_max [mm]':>17} | {'Δjitter [rad/s]':>16} "
          f"| {'Δtime [s/m]':>12}")
    for (t_l, v), grp in rpp.groupby(["lookahead_time", "velocity"]):
        if v != DEFAULT[1]:
            continue
        g = grp.set_index(["scenario", "seed"]).loc[base.index]
        cells = []
        for col, scale in (("e_ctrl_mean", 1000), ("e_ctrl_max", 1000), ("omega_jitter", 1), ("time_per_m", 1)):
            delta = scale * (g[col] - base[col])
            cells.append(f"{delta.mean():+8.3f} ± {delta.std():6.3f}")
        print(f"{t_l:4.1f} {v:4.1f} | " + " | ".join(cells))

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    panels = (("e_ctrl_mean", 100, "mean e_ctrl [cm]", axes[0, 0]),
              ("e_ctrl_max", 100, "peak e_ctrl [cm]", axes[0, 1]),
              ("jitter", 1, "ω jitter: RMS of ω − 1 s moving mean [rad/s]", axes[1, 0]),
              ("time_per_m", 1, "time per metre of path [s/m]", axes[1, 1]))
    vel_colors = {0.2: "tab:blue", 0.3: "tab:green", 0.4: "tab:red"}
    for col, scale, label, ax in panels:
        for v, grp in agg.reset_index().groupby("velocity"):
            ax.plot(grp["lookahead_time"], scale * grp[col], "o-", color=vel_colors[v], label=f"RPP, v = {v} m/s")
        for k, (L, row) in enumerate(classic.iterrows()):
            ax.axhline(scale * row[col], color="black", lw=1.0, ls=("--", ":")[k],
                       label=f"classic PP, L_d = {L} m, v = 0.3")
        ax.axvline(DEFAULT[0], color="gray", lw=0.6, ls=":")
        ax.set_xlabel("look-ahead time T_L [s]")
        ax.set_ylabel(label)
        ax.legend(fontsize=7)
    axes[0, 1].axhline(30.0, color="tab:red", lw=0.8, ls="-.")
    axes[0, 1].text(0.62, 29.0, "target ≤ 30 cm", fontsize=8, color="tab:red", va="top")
    fig.suptitle("MiniNav V4 — E3: look-ahead time × speed trade-off (S1–S5 × 5 seeds, default noise, EKF input)")
    fig.tight_layout()
    fig.savefig(RESULTS_DIR / "e3_tradeoff.png", dpi=130)
    print(f"wrote {RESULTS_DIR / 'e3_tradeoff.png'}")


# ---- E4 -----------------------------------------------------------------------------

def load_runs(folder: str, pattern: str) -> dict[str, tuple[dict, pd.DataFrame]]:
    runs = {}
    for f in sorted((DATA_DIR / folder).glob(pattern)):
        runs[f.name.removesuffix(".csv.gz")] = read_nav_csv(f)
    return runs


def e4_tables(summary: pd.DataFrame) -> None:
    noise_free = summary[summary["preset"] == "none"]
    summary = summary[summary["preset"] == "default"].copy()
    summary["arrived"] = summary["status"] == "arrived"
    summary["collided"] = summary["status"] == "collision"
    # 到达误差只对到达的运行有意义(碰撞的运行停在半路)。
    summary.loc[~summary["arrived"], "goal_error_true_m"] = np.nan
    print("\nE4 — S1–S5 × 10 seeds × {ekf, truth}, default noise, default parameters, map on")
    t = summary.groupby(["scenario", "controller_input"]).agg(
        path_m=("path_length_m", "mean"), arrived=("arrived", "sum"), collisions=("collided", "sum"),
        e_ctrl_mean=("e_ctrl_mean", "mean"), e_ctrl_max=("e_ctrl_max", "max"),
        e_true_max=("e_true_max", "max"), goal_err_true=("goal_error_true_m", "mean"),
        goal_err_true_max=("goal_error_true_m", "max"), min_clearance=("min_clearance_m", "min"))
    for col in ("e_ctrl_mean", "e_ctrl_max", "e_true_max", "goal_err_true", "goal_err_true_max", "min_clearance"):
        t[col] = (100 * t[col]).round(2)
    print("(errors and clearance in cm; goal errors over arrived runs only; S5 + ekf: runs end at the first "
          "collision)")
    print(t.round(2).to_string())
    ekf = summary[summary["controller_input"] == "ekf"]
    print(f"\nacceptance (§0.4, ekf input, all 50 runs): mean e_ctrl = {100 * ekf['e_ctrl_mean'].mean():.2f} cm "
          f"(≤ 10), peak e_ctrl = {100 * ekf['e_ctrl_max'].max():.2f} cm (≤ 30)")
    print(f"noise-free (preset none, S1–S5 × {{ekf, truth}}): "
          f"{(noise_free['status'] == 'arrived').sum()}/{len(noise_free)} arrived, "
          f"{(noise_free['status'] == 'collision').sum()} collisions, max true goal error "
          f"{100 * noise_free['goal_error_true_m'].max():.2f} cm, peak e_ctrl "
          f"{100 * noise_free['e_ctrl_max'].max():.2f} cm, min clearance "
          f"{100 * noise_free['min_clearance_m'].min():.2f} cm")
    oracle = summary[summary["controller_input"] == "truth"]
    print(f"oracle: {oracle['arrived'].sum()}/{len(oracle)} arrived, {oracle['collided'].sum()} collisions, "
          f"max true goal error {100 * oracle['goal_error_true_m'].max():.2f} cm (tolerance 5 cm)")


def e4_decomposition(runs: dict) -> None:
    worst = 0.0
    rows = 0
    for name, (_, df) in runs.items():
        if "_ekf_" not in name:
            continue
        gap = df["e_true"] - df["e_ctrl"] - df["e_est"]
        worst = max(worst, float(gap.max()))
        rows += len(df)
    print(f"\n1-Lipschitz bound e_true ≤ e_ctrl + e_est over {rows} samples of the 50 EKF runs: "
          f"max(e_true − e_ctrl − e_est) = {worst:.2e} m (CSV rounding is 7 significant digits)")

    fig = plt.figure(figsize=(14, 8.5))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 1.05])
    ax_t = fig.add_subplot(gs[0, :])
    meta, df = runs["S3_ekf_seed1"]
    s = traveled(df)
    ax_t.fill_between(s, 0, 100 * (df["e_ctrl"] + df["e_est"]), color="gray", alpha=0.18,
                      label="bound e_ctrl + e_est")
    ax_t.plot(s, 100 * df["e_true"], color="black", lw=1.4, label="e_true = dist(p_true, path)")
    ax_t.plot(s, 100 * df["e_ctrl"], color="tab:blue", lw=1.2, label="e_ctrl = dist(p̂, path)  (controller)")
    ax_t.plot(s, 100 * df["e_est"], color="tab:red", lw=1.2, label="e_est = ‖p_true − p̂‖  (localization)")
    ax_t.set_xlabel("distance travelled [m]")
    ax_t.set_ylabel("error [cm]")
    ax_t.set_title("S3 (living room → bedroom 2), seed 1, EKF into the controller — the controller holds e_ctrl "
                   "at a few cm; e_true follows e_est")
    ax_t.legend(fontsize=8, loc="upper left")

    ax_b = fig.add_subplot(gs[1, 0])
    ax_c = fig.add_subplot(gs[1, 1])
    keys = [sc.key for sc in SCENARIOS]
    data_ctrl_ekf, data_ctrl_truth, data_est, data_true = [], [], [], []
    for key in keys:
        def cat(ci: str, col: str) -> np.ndarray:
            return np.concatenate([df[col].to_numpy() for n, (_, df) in runs.items()
                                   if n.startswith(f"{key}_{ci}_")])
        floor = 1e-3  # cm;对数轴下把 0 画在底部
        data_ctrl_ekf.append(np.maximum(100 * cat("ekf", "e_ctrl"), floor))
        data_ctrl_truth.append(np.maximum(100 * cat("truth", "e_ctrl"), floor))
        data_est.append(np.maximum(100 * cat("ekf", "e_est"), floor))
        data_true.append(np.maximum(100 * cat("ekf", "e_true"), floor))
    pos = np.arange(len(keys))
    width = 0.2
    for k, (data, color, label) in enumerate(((data_ctrl_truth, "tab:cyan", "e_ctrl, oracle (truth in)"),
                                              (data_ctrl_ekf, "tab:blue", "e_ctrl, EKF in"),
                                              (data_est, "tab:red", "e_est, EKF in"),
                                              (data_true, "black", "e_true, EKF in"))):
        bp = ax_b.boxplot(data, positions=pos + (k - 1.5) * width, widths=0.17, whis=(5, 95), showfliers=False,
                          patch_artist=True)
        for patch in bp["boxes"]:
            patch.set(facecolor=color, alpha=0.55)
        for med in bp["medians"]:
            med.set(color="black")
        ax_b.plot([], [], "s", color=color, alpha=0.55, label=label)
    ax_b.set_xticks(pos, [f"{k}" for k in keys])
    ax_b.set_yscale("log")
    ax_b.set_ylim(1e-3, 1e3)
    ax_b.set_ylabel("error [cm] (box 25–75%, whiskers 5–95%)")
    ax_b.set_title("all samples, 10 seeds per scenario")
    ax_b.legend(fontsize=7, loc="upper center", ncol=2)

    for key in keys:
        maxima = [(df["e_ctrl"].max(), df["e_true"].max()) for n, (_, df) in runs.items()
                  if n.startswith(f"{key}_ekf_")]
        arr = 100 * np.asarray(maxima)
        ax_c.plot(arr[:, 0], arr[:, 1], "o", color=SCEN_COLORS[key], label=key)
    lim = ax_c.get_xlim()
    ax_c.plot([0, 30], [0, 30], color="gray", lw=0.8, ls="--", label="e_true = e_ctrl")
    ax_c.set_xlim(0, max(6.0, lim[1]))
    ax_c.set_xlabel("peak e_ctrl per run [cm]")
    ax_c.set_ylabel("peak e_true per run [cm]")
    ax_c.set_title("per-run peaks: control error vs true error (EKF in)")
    ax_c.legend(fontsize=7)
    fig.suptitle("MiniNav V4 — E4: error decomposition (default noise, default parameters)")
    fig.tight_layout()
    fig.savefig(RESULTS_DIR / "e4_error_decomposition.png", dpi=130)
    print(f"wrote {RESULTS_DIR / 'e4_error_decomposition.png'}")


def e4_drift(e4: pd.DataFrame) -> None:
    follow = load_runs("drift", "S5_follow_seed*.csv.gz")
    drift_summary = pd.read_csv(DATA_DIR / "drift_summary.csv")
    on_map = drift_summary[drift_summary["mode"] == "map"]
    grid = np.linspace(0.0, 34.0, 341)
    est = np.vstack([np.interp(grid, traveled(df), df["e_est"].to_numpy()) for _, df in follow.values()])
    def major_3sigma(df: pd.DataFrame) -> np.ndarray:
        sxx, syy, sxy = (df[c].to_numpy() for c in ("ekf_sigma_xx", "ekf_sigma_yy", "ekf_sigma_xy"))
        lam = 0.5 * (sxx + syy) + np.sqrt(0.25 * (sxx - syy) ** 2 + sxy ** 2)
        return 3.0 * np.sqrt(lam)

    sigma = np.vstack([np.interp(grid, traveled(df), major_3sigma(df)) for _, df in follow.values()])
    med, p10, p90 = (np.percentile(est, q, axis=0) for q in (50, 10, 90))

    def first_crossing(curve: np.ndarray, level: float) -> float:
        idx = np.nonzero(curve > level)[0]
        return float(grid[idx[0]]) if len(idx) else float("nan")

    print(f"\ndrift vs distance (S5, 20 seeds, EKF in, no map): e_est median / p90 at "
          + ", ".join(f"{s:.0f} m = {100 * np.interp(s, grid, med):.1f} / {100 * np.interp(s, grid, p90):.1f} cm"
                      for s in (5, 10, 20, 34)))
    for level in (M_EFF, 0.10):
        print(f"  e_est p90 exceeds {100 * level:.1f} cm after {first_crossing(p90, level):.1f} m; "
              f"median after {first_crossing(med, level):.1f} m")
    d = on_map.sort_values("traveled_m")
    collided = d[d["status"] == "collision"]
    print(f"S5 with the map (20 seeds): {len(collided)} collisions, first collision after "
          f"{collided['traveled_m'].min():.1f} m, median {collided['traveled_m'].median():.1f} m; "
          f"{(d['status'] == 'arrived').sum()} arrived")

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=(14, 5.2), gridspec_kw={"width_ratios": [1.5, 1]})
    for row in est:
        ax_a.plot(grid, 100 * row, color="tab:red", lw=0.5, alpha=0.25)
    ax_a.fill_between(grid, 100 * p10, 100 * p90, color="tab:red", alpha=0.15, label="e_est 10–90% (20 seeds)")
    ax_a.plot(grid, 100 * med, color="tab:red", lw=2.0, label="e_est median")
    ax_a.plot(grid, 100 * np.median(sigma, axis=0), color="tab:purple", lw=1.4, ls="--",
              label="EKF-reported 3σ, major axis (median)")
    arrived = e4[(e4["controller_input"] == "ekf") & (e4["status"] == "arrived")]
    ax_a.plot(arrived["traveled_m"], 100 * arrived["goal_error_true_m"], "k.", ms=6,
              label="true goal error at arrival (S1–S4, EKF in)")
    ax_a.axhline(100 * M_EFF, color="black", lw=0.8, ls="--")
    ax_a.text(24.0, 100 * M_EFF * 0.8, f"worst-case margin m_eff = {100 * M_EFF:.1f} cm", fontsize=8, va="top")
    ax_a.axhline(10.0, color="gray", lw=0.8, ls=":")
    ax_a.text(24.0, 10.0 * 0.85, "typical door clearance ≈ 10 cm (E2c)", fontsize=8, color="gray", va="top")
    ax_a.set_yscale("log")
    ax_a.set_xlabel("distance travelled s [m]")
    ax_a.set_ylabel("position estimation error [cm]")
    ax_a.set_title("dead-reckoning drift (encoders + gyro, no absolute fix), S5 without the map")
    ax_a.set_xlim(0, 34.5)
    ax_a.set_ylim(0.3, 400)
    ax_a.legend(fontsize=8, loc="upper left")

    total = len(d)
    distances = np.concatenate([[0.0], collided["traveled_m"].to_numpy(), [34.4]])
    alive = total - np.concatenate([[0], np.arange(1, len(collided) + 1), [len(collided)]])
    ax_b.step(distances, 100 * alive / total, where="post", color="black", lw=1.8)
    ax_b.plot(collided["traveled_m"], 100 * (total - np.arange(1, len(collided) + 1)) / total, "x", color="tab:red")
    ax_b.set_xlabel("distance travelled s [m]")
    ax_b.set_ylabel("runs without collision [%]")
    ax_b.set_title(f"S5 office500 with the map, EKF in ({total} seeds)")
    ax_b.set_xlim(0, 34.5)
    ax_b.set_ylim(0, 105)
    fig.suptitle("MiniNav V4 — E4: localization drift sets the reachable distance (oracle runs: 10/10 arrive)")
    fig.tight_layout()
    fig.savefig(RESULTS_DIR / "e4_drift.png", dpi=130)
    print(f"wrote {RESULTS_DIR / 'e4_drift.png'}")


def phase_of(df: pd.DataFrame) -> np.ndarray:
    """按工况给每行打标签:rotate(原地转)/ accel(|dv/dt| > 0.05 m/s²)/ cruise / stopped。"""
    acc = np.abs(np.gradient(df["act_v"].to_numpy(), SIM_DT))
    phase = np.where(df["act_v"].abs() > 0.01, "cruise", "stopped").astype(object)
    phase[acc > 0.05] = "accel"
    phase[df["regime"].isin(["rotate_to_path", "rotate_to_goal"]).to_numpy()] = "rotate"
    return phase


def e4_consistency(runs: dict) -> None:
    frames = []
    for name, (_, df) in runs.items():
        if "_ekf_" not in name:
            continue
        frames.append(pd.DataFrame({"phase": phase_of(df), "nis_encoder": df["nis_encoder"],
                                    "nis_imu": df["nis_imu"]}))
    nis = pd.concat(frames)
    lo2, hi2 = CHI2_95[2]
    lo1, hi1 = CHI2_95[1]
    table = nis.groupby("phase").agg(samples=("nis_encoder", "size"), nis_encoder=("nis_encoder", "mean"),
                                     nis_imu=("nis_imu", "mean"))
    table["enc_in_95"] = nis.groupby("phase")["nis_encoder"].apply(lambda x: ((x >= lo2) & (x <= hi2)).mean())
    table["imu_in_95"] = nis.groupby("phase")["nis_imu"].apply(lambda x: ((x >= lo1) & (x <= hi1)).mean())
    overall = pd.DataFrame({"samples": [len(nis)], "nis_encoder": [nis["nis_encoder"].mean()],
                            "nis_imu": [nis["nis_imu"].mean()],
                            "enc_in_95": [((nis["nis_encoder"] >= lo2) & (nis["nis_encoder"] <= hi2)).mean()],
                            "imu_in_95": [((nis["nis_imu"] >= lo1) & (nis["nis_imu"] <= hi1)).mean()]},
                           index=["all"])
    table = pd.concat([table, overall])
    print("\nclosed-loop NIS by phase (50 EKF runs; expected mean 2 / 1, V2 open loop "
          f"{V2_OPEN_LOOP_NIS['encoder']} / {V2_OPEN_LOOP_NIS['imu']})")
    print(table.round(3).to_string())

    follow = load_runs("drift", "S5_follow_seed*.csv.gz")
    grid = np.linspace(0.0, 34.0, 171)
    nees = []
    nees_th = []
    for _, df in follow.values():
        ex = (df["truth_x"] - df["ekf_x"]).to_numpy()
        ey = (df["truth_y"] - df["ekf_y"]).to_numpy()
        sxx, syy, sxy = (df[c].to_numpy() for c in ("ekf_sigma_xx", "ekf_sigma_yy", "ekf_sigma_xy"))
        det = sxx * syy - sxy * sxy
        n2 = (syy * ex * ex - 2 * sxy * ex * ey + sxx * ey * ey) / det
        eth = np.angle(np.exp(1j * (df["truth_yaw"] - df["ekf_yaw"]).to_numpy()))
        n1 = eth * eth / df["ekf_sigma_thth"].to_numpy()
        s = traveled(df)
        nees.append(np.interp(grid, s, n2))
        nees_th.append(np.interp(grid, s, n1))
    runs_n = len(nees)
    anees = np.mean(nees, axis=0)
    anees_th = np.mean(nees_th, axis=0)
    lo_a, hi_a = chi2_quantile(0.025, 2 * runs_n) / runs_n, chi2_quantile(0.975, 2 * runs_n) / runs_n
    lo_t, hi_t = chi2_quantile(0.025, runs_n) / runs_n, chi2_quantile(0.975, runs_n) / runs_n
    beyond = grid > 1.0
    print(f"position ANEES over {runs_n} runs (S5, no map): mean {anees[beyond].mean():.2f} "
          f"(expected 2, 95% band [{lo_a:.2f}, {hi_a:.2f}]), inside band {100 * np.mean((anees[beyond] >= lo_a) & (anees[beyond] <= hi_a)):.0f}% of the path; "
          f"heading ANEES mean {anees_th[beyond].mean():.2f} (expected 1, band [{lo_t:.2f}, {hi_t:.2f}])")

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=(14, 4.8))
    phases = ["accel", "cruise", "rotate", "stopped", "all"]
    present = [p for p in phases if p in table.index]
    x = np.arange(len(present))
    ax_a.bar(x - 0.2, table.loc[present, "nis_encoder"], width=0.38, color="tab:green", label="encoder NIS (dof 2)")
    ax_a.bar(x + 0.2, table.loc[present, "nis_imu"], width=0.38, color="tab:blue", label="gyro NIS (dof 1)")
    ax_a.axhline(2.0, color="tab:green", lw=1.0, ls="--")
    ax_a.axhline(1.0, color="tab:blue", lw=1.0, ls="--")
    ax_a.axhline(V2_OPEN_LOOP_NIS["encoder"], color="tab:green", lw=0.8, ls=":")
    ax_a.axhline(V2_OPEN_LOOP_NIS["imu"], color="tab:blue", lw=0.8, ls=":")
    ax_a.text(len(present) - 0.55, V2_OPEN_LOOP_NIS["encoder"] + 0.1, "V2 open loop", fontsize=7,
              color="tab:green")
    ax_a.text(len(present) - 0.55, V2_OPEN_LOOP_NIS["imu"] + 0.1, "V2 open loop", fontsize=7, color="tab:blue")
    names = {"accel": "accelerating\n|dv/dt| > 0.05 m/s²", "cruise": "steady", "rotate": "rotate in place",
             "stopped": "stopped", "all": "all"}
    ax_a.set_xticks(x, [f"{names[p]}\n({100 * table.loc[p, 'samples'] / table.loc['all', 'samples']:.0f}% of samples)"
                        for p in present])
    ax_a.set_ylabel("mean NIS")
    ax_a.set_title("closed-loop NIS by phase (50 EKF runs; dashed: expected)")
    ax_a.legend(fontsize=8)

    ax_b.plot(grid, anees, color="tab:purple", lw=1.4, label=f"position ANEES ({runs_n} runs, dof 2)")
    ax_b.plot(grid, anees_th, color="tab:orange", lw=1.2, label=f"heading ANEES ({runs_n} runs, dof 1)")
    ax_b.axhspan(lo_a, hi_a, color="tab:purple", alpha=0.12, label="95% band, position")
    ax_b.axhspan(lo_t, hi_t, color="tab:orange", alpha=0.10, label="95% band, heading")
    ax_b.set_xlabel("distance travelled [m]")
    ax_b.set_ylabel("ANEES")
    ax_b.set_title("S5 (34 m), EKF in, no map: is the reported covariance honest?")
    ax_b.set_xlim(0, 34)
    ax_b.set_ylim(0, max(6.0, float(np.percentile(anees, 99)) * 1.1))
    ax_b.legend(fontsize=8, loc="upper right")
    fig.suptitle("MiniNav V4 — E4: EKF consistency in the closed loop")
    fig.tight_layout()
    fig.savefig(RESULTS_DIR / "e4_consistency.png", dpi=130)
    print(f"wrote {RESULTS_DIR / 'e4_consistency.png'}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("parts", nargs="*", choices=("e3", "e4"), help="default: both")
    args = ap.parse_args()
    parts = args.parts or ["e3", "e4"]
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    if "e3" in parts:
        e3()
    if "e4" in parts:
        summary = pd.read_csv(DATA_DIR / "e4_summary.csv")
        runs = load_runs("e4", "*.csv.gz")
        e4_tables(summary)
        e4_decomposition(runs)
        e4_drift(summary[summary["preset"] == "default"])
        e4_consistency(runs)


if __name__ == "__main__":
    main()
