#!/usr/bin/env python3
"""
corner_cutting.py — E2:折线转角的切角尺度律与安全裕度预算(docs/math/pure_pursuit.md §4–§5)。

  E2a  尺度律:经典 PP(固定 L、不限速、不原地转)、理想底盘(τ = 0)、无噪声、真值进
       控制器、100 Hz。单转角路径(左转 θ),车从转角前 15 L 处、在路径上出发。
       θ ∈ {22.5°, 45°, 67.5°, 90°, 112.5°, 135°} × L ∈ {0.1, 0.2, 0.4} m。
       单个转角没有内禀长度 ⇒ 内侧切角 δ_in = f(θ)·L、出弯外侧偏移 δ_out = g(θ)·L。
       小角度线性理论:f(θ) ≈ e^(−1)·cos(1)·θ ≈ 0.199·θ,g(θ) ≈ e^(−3π/4)/√2·θ ≈ 0.067·θ。
  E2b  完整 RPP + 默认底盘(τ = 0.1 s、限幅)+ 20 Hz + 平滑器,L 形路径(90° 左转),
       扫 T_L × R_min:内侧切角、出弯外侧偏移、转角速度 vs 有效裕度 m_eff。
  E2c  真实地图:apartment S2 / S3 与 office500 S5,无噪声 + 真值进控制器(只剩控制误差),
       同一组 T_L × R_min:真值车体的最小净空 vs 路径本身的净空(车完全在路径上时)。

  m_eff = r_infl − r_robot − √2·res = 0.25 − 0.136 − 0.0707 ≈ 0.043 m(§5 的最坏情况裕度)。

产出:
  results/v4/e2_corner_scaling.png   E2a:f(θ)、g(θ) 与 L 无关 + 线性理论
  results/v4/e2_safety.png           E2b / E2c:切角、出弯偏移、最小净空 vs T_L、R_min
  stdout                             各表

Run:
    python scripts/v4/corner_cutting.py
"""

from __future__ import annotations

import argparse
import math

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from _navio import (DATA_DIR, IDEAL_ROBOT, REPO, RESULTS_DIR, classic_pp, clearance, densify, find_sim,
                    load_grid, no_approach_slowdown, occupied_centers, plan_path, read_yaml_subset, run_and_load,
                    signed_cross_track, variant_configs, write_path_csv)
from scenarios import BY_KEY

OUT = DATA_DIR / "e2"
THETAS_DEG = (22.5, 45.0, 67.5, 90.0, 112.5, 135.0)
LOOKAHEADS = (0.1, 0.2, 0.4)
LOOKAHEAD_TIMES = (0.5, 0.6, 0.7, 0.8, 1.0, 1.2, 1.4)
MIN_RADII = (0.3, 0.6, 0.9)
F_LINEAR = math.exp(-1.0) * math.cos(1.0)                    # ≈ 0.199
G_LINEAR = math.exp(-0.75 * math.pi) / math.sqrt(2.0)        # ≈ 0.067
PLAN_TABLE = {22.5: 0.077, 45.0: 0.151, 90.0: 0.271, 135.0: 0.303}  # docs/v4_plan.md §3.4(规划阶段预估)


def margin_budget() -> float:
    robot = read_yaml_subset(REPO / "config" / "robot.yaml")
    nav = read_yaml_subset(REPO / "config" / "nav.yaml")
    r_robot = math.hypot(robot["footprint"]["length"] / 2, robot["footprint"]["width"] / 2)
    return nav["planner"]["inflation_radius"] - r_robot - math.sqrt(2.0) * 0.05


def corner_deviation(df, path: np.ndarray, near: np.ndarray, radius: float) -> tuple[float, float]:
    """转角附近(距拐点 radius 以内)的最大内侧切角与最大外侧偏移(左转:内侧为正)。"""
    pts = df[["truth_x", "truth_y"]].to_numpy()
    sel = np.linalg.norm(pts - near, axis=1) <= radius
    e = signed_cross_track(pts[sel], path)
    return float(max(e.max(), 0.0)), float(max(-e.min(), 0.0))


# ---- E2a ----------------------------------------------------------------------------

def e2a(sim, ax_f) -> None:
    print("\nE2a — single corner, classic PP, τ = 0 (δ = f(θ)·L inside, g(θ)·L outside)")
    print(f"{'theta':>6} | " + " | ".join(f"f, L={L:.1f}  g, L={L:.1f}" for L in LOOKAHEADS)
          + " | f lin   g lin | plan f")
    results: dict[float, list[tuple[float, float]]] = {}
    for theta in THETAS_DEG:
        row = []
        th = math.radians(theta)
        for L in LOOKAHEADS:
            v = 0.3
            name = f"e2a_th{theta}_L{L}"
            robot, nav = variant_configs(name, robot=IDEAL_ROBOT, nav={"controller": {
                "control_frequency": 100.0, **classic_pp(L, v), **no_approach_slowdown(v),
                "max_accel": 1000.0, "max_angular_accel": 1000.0}}, out_dir=OUT / "configs")
            pts = [(-20.0 * L, 0.0), (0.0, 0.0), (20.0 * L * math.cos(th), 20.0 * L * math.sin(th))]
            path_csv = write_path_csv(pts, OUT / "paths" / f"corner_{theta}_{L}.csv", f"E2a corner {theta} deg")
            csv = OUT / f"{name}.csv"
            _, df = run_and_load(sim, csv, path_csv=path_csv, start=f"{-15.0 * L},0,0", seed=1, preset="none",
                                 controller_input="truth", robot=robot, nav=nav)
            inside, outside = corner_deviation(df, np.asarray(pts), np.zeros(2), 8.0 * L)
            row.append((inside / L, outside / L))
        results[theta] = row
        print(f"{theta:6.1f} | " + " | ".join(f"{f:9.3f}  {g:9.3f}" for f, g in row)
              + f" | {F_LINEAR * th:5.3f} {G_LINEAR * th:5.3f} | {PLAN_TABLE.get(theta, float('nan')):6.3f}")

    th_grid = np.linspace(0, 140, 200)
    ax_f.plot(th_grid, F_LINEAR * np.radians(th_grid), color="tab:blue", lw=1.0, ls="--",
              label="linear theory f ≈ e⁻¹cos(1)·θ")
    ax_f.plot(th_grid, G_LINEAR * np.radians(th_grid), color="tab:red", lw=1.0, ls="--",
              label="linear theory g ≈ e^(−3π/4)/√2·θ")
    for k, (L, marker) in enumerate(zip(LOOKAHEADS, ("o", "s", "^"))):
        f = [results[t][k][0] for t in THETAS_DEG]
        g = [results[t][k][1] for t in THETAS_DEG]
        ax_f.plot(THETAS_DEG, f, marker, color="tab:blue", mfc="none" if k else "tab:blue", ms=7,
                  label=f"inside cut δ_in / L (L = {L} m)")
        ax_f.plot(THETAS_DEG, g, marker, color="tab:red", mfc="none" if k else "tab:red", ms=7,
                  label=f"exit overshoot δ_out / L (L = {L} m)")
    ax_f.plot(list(PLAN_TABLE), list(PLAN_TABLE.values()), "x", color="black", ms=8,
              label="plan-stage estimate (v4_plan §3.4)")
    ax_f.set_xlabel("corner angle θ [deg]")
    ax_f.set_ylabel("deviation / L")
    ax_f.set_title("E2a: corner deviation scales with L (classic PP, τ = 0)")
    ax_f.set_xlim(0, 140)
    ax_f.set_ylim(0, 0.42)
    ax_f.legend(fontsize=7, loc="upper left")


# ---- E2b ----------------------------------------------------------------------------

def e2b(sim, ax_in, ax_out, ax_time, m_eff: float) -> None:
    print("\nE2b — full RPP, default robot (τ = 0.1 s, 20 Hz, smoother), L-shaped path, noise-free")
    print(f"{'T_L':>4} {'R_min':>5} {'r_eff':>5} | {'inside':>7} {'outside':>7} {'v_corner':>8} {'time':>6}")
    pts = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]
    path_csv = write_path_csv(pts, OUT / "paths" / "l_path.csv", "E2b L-shaped path")
    styles = {0.3: (":", "^"), 0.6: ("-", "o"), 0.9: ("--", "s")}
    colors = {0.3: "tab:gray", 0.6: "tab:blue", 0.9: "tab:green"}
    for r_min in MIN_RADII:
        inside_v, outside_v, time_v = [], [], []
        for t_l in LOOKAHEAD_TIMES:
            name = f"e2b_TL{t_l}_R{r_min}"
            robot, nav = variant_configs(name, nav={"controller": {"lookahead_time": t_l,
                                                                   "regulated_min_radius": r_min}},
                                         out_dir=OUT / "configs")
            csv = OUT / f"{name}.csv"
            meta, df = run_and_load(sim, csv, path_csv=path_csv, seed=1, preset="none",
                                    controller_input="truth", nav=nav)
            inside, outside = corner_deviation(df, np.asarray(pts), np.array([2.0, 0.0]), 1.5)
            near = np.hypot(df["truth_x"] - 2.0, df["truth_y"]) < 0.05
            v_corner = float(df.loc[near, "true_v"].min()) if near.any() else float("nan")
            r_eff = (0.1 + 0.025) / t_l
            print(f"{t_l:4.1f} {r_min:5.1f} {r_eff:5.3f} | {100 * inside:6.2f}cm {100 * outside:6.2f}cm "
                  f"{v_corner:7.3f} {meta['end_time_s']:6.2f}s  {meta['status']}")
            inside_v.append(inside)
            outside_v.append(outside)
            time_v.append(meta["end_time_s"])
        ls, marker = styles[r_min]
        for ax, values, scale in ((ax_in, inside_v, 100), (ax_out, outside_v, 100), (ax_time, time_v, 1)):
            ax.plot(LOOKAHEAD_TIMES, scale * np.asarray(values), marker=marker, ls=ls, color=colors[r_min],
                    label=f"R_min = {r_min} m" + (" (default)" if r_min == 0.6 else ""))
    for ax, title in ((ax_in, "E2b: inside corner cut"), (ax_out, "E2b: exit overshoot (outside)")):
        ax.axhline(100 * m_eff, color="black", lw=1.0, ls="--")
        ax.text(1.12, 100 * m_eff + 0.15, f"m_eff = {100 * m_eff:.1f} cm", fontsize=8)
        ax.set_ylabel("deviation from path [cm]")
        ax.set_title(title + " — 90° corner, full RPP, τ = 0.1 s, 20 Hz")
    ax_time.set_ylabel("time to goal [s]")
    ax_time.set_title("E2b: time to goal (2 + 2 m L path)")
    ax_time.annotate("rotate ↔ track heading\nlimit cycle after the corner", xy=(0.5, 35.0), xytext=(0.75, 30.0),
                     arrowprops={"arrowstyle": "->", "lw": 0.8}, fontsize=8)
    for ax in (ax_in, ax_out, ax_time):
        ax.axvspan(0.45, 0.625, color="tab:orange", alpha=0.12)
        ax.text(0.47, ax.get_ylim()[1] * 0.92, "r > 0.2", fontsize=8, color="tab:orange")
        ax.axvline(1.0, color="gray", lw=0.6, ls=":")
        ax.set_xlabel("look-ahead time T_L [s]")
        ax.set_xlim(0.45, 1.45)
        ax.legend(fontsize=7, loc="upper right")


# ---- E2c ----------------------------------------------------------------------------

def path_min_clearance(key: str, r_robot: float) -> float:
    sc = BY_KEY[key]
    sim = find_sim()
    path = plan_path(sim, sc.map_yaml, sc.start, sc.goal, OUT / "paths" / f"{key.lower()}_plan.csv")
    grid = load_grid(REPO / sc.map_yaml)
    pts = densify(path, 0.01)
    return float(np.min(clearance(pts, grid, r_robot, centers=occupied_centers(grid))))


def e2c(sim, ax, m_eff: float) -> None:
    robot_cfg = read_yaml_subset(REPO / "config" / "robot.yaml")
    r_robot = math.hypot(robot_cfg["footprint"]["length"] / 2, robot_cfg["footprint"]["width"] / 2)
    print("\nE2c — real maps, noise-free, truth into the controller: min true clearance [cm]")
    print(f"{'scenario':>8} {'path':>6} | " + " ".join(f"T_L={t:.1f}" for t in LOOKAHEAD_TIMES) + "   (R_min = 0.6)")
    colors = {"S2": "tab:orange", "S3": "tab:green", "S5": "tab:purple"}
    for key in ("S2", "S3", "S5"):
        nominal = path_min_clearance(key, r_robot)
        for r_min, ls, marker in ((0.6, "-", "o"), (0.9, "--", "s")):
            values = []
            for t_l in LOOKAHEAD_TIMES:
                name = f"e2c_TL{t_l}_R{r_min}"
                _, nav = variant_configs(name, nav={"controller": {"lookahead_time": t_l,
                                                                   "regulated_min_radius": r_min}},
                                         out_dir=OUT / "configs")
                meta, _ = run_and_load(sim, OUT / f"{name}_{key}.csv", **BY_KEY[key].nav_args(), seed=1,
                                       preset="none", controller_input="truth", nav=nav)
                values.append(meta["min_clearance_m"] if meta["status"] != "collision" else -0.001)
            if r_min == 0.6:
                print(f"{key:>8} {100 * nominal:6.2f} | " + " ".join(f"{100 * c:7.2f}" for c in values))
            else:
                print(f"{'':>8} {'R=0.9':>6} | " + " ".join(f"{100 * c:7.2f}" for c in values))
            ax.plot(LOOKAHEAD_TIMES, 100 * np.asarray(values), marker, ls=ls, color=colors[key],
                    mfc="none" if r_min == 0.9 else colors[key],
                    label=f"{key}, R_min = {r_min} m")
        ax.axhline(100 * nominal, color=colors[key], lw=0.8, ls=":")
    ax.axhline(0.0, color="black", lw=1.0)
    ax.set_xlabel("look-ahead time T_L [s]")
    ax.set_ylabel("min true clearance [cm]")
    ax.set_title("E2c: real maps, control error only (dotted: robot exactly on the path)")
    ax.set_xlim(0.45, 1.45)
    ax.legend(fontsize=7, loc="lower left", ncol=2)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.parse_args()
    sim = find_sim()
    m_eff = margin_budget()
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(7.5, 5.2))
    e2a(sim, ax)
    fig.tight_layout()
    fig.savefig(RESULTS_DIR / "e2_corner_scaling.png", dpi=130)
    print(f"wrote {RESULTS_DIR / 'e2_corner_scaling.png'}")

    fig, axes = plt.subplots(2, 2, figsize=(14, 9.5))
    e2b(sim, axes[0, 0], axes[0, 1], axes[1, 0], m_eff)
    e2c(sim, axes[1, 1], m_eff)
    fig.suptitle(f"MiniNav V4 — E2: corner deviation and clearance vs look-ahead time / min radius "
                 f"(m_eff = {100 * m_eff:.1f} cm)")
    fig.tight_layout()
    fig.savefig(RESULTS_DIR / "e2_safety.png", dpi=130)
    print(f"wrote {RESULTS_DIR / 'e2_safety.png'}")


if __name__ == "__main__":
    main()
