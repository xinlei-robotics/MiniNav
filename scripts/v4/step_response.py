#!/usr/bin/env python3
"""
step_response.py — E1:Pure Pursuit 直线响应与 docs/math/pure_pursuit.md §2–§3 的解析值对照。

三组实验,全部走 `sim nav --path`(无噪声、真值进控制器、100 Hz 控制):

  E1a  τ = 0 的理想底盘,静止起步、与路径平行、初始横向偏置 e0。
       L_d ∈ {0.2, 0.3, 0.5} m × v ∈ {0.1, 0.3, 0.5} m/s,e0 ∈ {0.05, 0.3} m。
       线性化预测:e(s)/e0 = e^(−s/L)·(cos(s/L) + sin(s/L)),与速度无关;
       反向超调 e^(−π) ≈ 4.32%(在 s = πL),2% 调节距离 ≈ 4.21 L(包络界 4.26 L)。
  E1b  一阶执行器滞后 τ:L = 0.3 m、v = 0.3 m/s(T_L = L/v = 1 s),τ 扫描。
       起步时 v 也有滞后,所以改成"在直线上先跑 3 m 达到稳态,再遇到 h = 5 cm 的
       横向台阶"—— look-ahead 点越过台阶时,线性系统就从 (e, ψ, ω_a) = (−h, 0, 0)
       出发,与初始偏置响应同构。线性模型 r·σ³ + σ² + 2σ + 2 = 0(r = τ_eff/T_L),
       零阶保持按 τ_eff = τ + T_c/2 折算;100 Hz 与 20 Hz 两组控制频率各跑一遍。
  E1c  速度自适应 look-ahead 的依据:τ = 0.2 s 固定,v ∈ {0.15, 0.3, 0.45} m/s。
       固定 L = 0.3 m 时 r = τ·v/L 随速度变(响应不同);L = T_L·v(T_L = 1 s)时
       r 恒为 0.2(三条曲线重合)。

产出:
  results/v4/e1_step_response.png   E1a:距离域曲线坍缩 + 大偏置的非线性偏离
  results/v4/e1_lag.png             E1b / E1c:滞后比扫描 vs 线性模型、速度自适应 look-ahead
  stdout                            超调 / 极值位置 / 调节距离与解析值的相对偏差

Run:
    cmake --build --preset build-release -j
    python scripts/v4/step_response.py
"""

from __future__ import annotations

import argparse
import math

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from _navio import (DATA_DIR, IDEAL_ROBOT, RESULTS_DIR, classic_pp, find_sim, no_approach_slowdown,
                    run_and_load, variant_configs, write_path_csv)

OUT = DATA_DIR / "e1"
OVERSHOOT_RATIO = math.exp(-math.pi)            # ≈ 0.0432
SETTLE_ENVELOPE = math.log(50.0 * math.sqrt(2))  # ≈ 4.26(包络 √2·e^(−s/L) 落进 2% 带)
FAST_SMOOTHER = {"max_accel": 1000.0, "max_angular_accel": 1000.0}  # 平滑器不起作用


# ---- 线性模型 -------------------------------------------------------------------

def analytic_offset(s_over_l: np.ndarray) -> np.ndarray:
    """τ = 0:e(s)/e0 = e^(−s/L)(cos(s/L) + sin(s/L))。"""
    x = np.asarray(s_over_l)
    return np.exp(-x) * (np.cos(x) + np.sin(x))


def lag_model(r: float, sigma: np.ndarray) -> np.ndarray:
    """带一阶滞后的线性化闭环,归一化到 σ = s/L = t/T_L、ẽ = e/e0。

    状态 (ẽ, ψ̃, w̃):ẽ' = ψ̃,ψ̃' = w̃,r·w̃' = −2ẽ − 2ψ̃ − w̃(r = 0 时退化为二阶)。
    初值 (1, 0, 0);用特征分解求精确解。"""
    sigma = np.asarray(sigma, dtype=float)
    if r <= 0.0:
        return analytic_offset(sigma)
    A = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [-2.0 / r, -2.0 / r, -1.0 / r]])
    lam, V = np.linalg.eig(A)
    c = np.linalg.solve(V, np.array([1.0, 0.0, 0.0], dtype=complex))
    return np.real((V[0] * c) @ np.exp(np.outer(lam, sigma)))


def response_metrics(sigma: np.ndarray, e: np.ndarray) -> tuple[float, float, float]:
    """(反向超调比, 极值位置 σ, 2% 调节距离 σ);e 已按 e0 归一化,终值 0。"""
    i = int(np.argmin(e))
    outside = np.nonzero(np.abs(e) > 0.02)[0]
    settle = float(sigma[outside[-1]]) if len(outside) else 0.0
    return float(-e[i]), float(sigma[i]), settle


SIGMA_DENSE = np.linspace(0.0, 60.0, 60001)
EXACT_SETTLE = response_metrics(SIGMA_DENSE, analytic_offset(SIGMA_DENSE))[2]  # ≈ 4.21


# ---- 运行 -----------------------------------------------------------------------

def run_offset(sim, L: float, v: float, e0: float) -> tuple[np.ndarray, np.ndarray]:
    """E1a:直线 y = 0,从 (0, e0, 0) 静止起步。返回 (s/L, e/e0)。"""
    name = f"e1a_L{L}_v{v}_e{e0}"
    robot, nav = variant_configs(name, robot=IDEAL_ROBOT, nav={"controller": {
        "control_frequency": 100.0, **classic_pp(L, v), **no_approach_slowdown(v), **FAST_SMOOTHER}},
        out_dir=OUT / "configs")
    length = 14.0 * L + 1.0
    path = write_path_csv([(-1.0, 0.0), (length, 0.0)], OUT / "paths" / f"line_{length:.2f}.csv",
                          "E1a straight line y = 0")
    csv = OUT / f"{name}.csv"
    _, df = run_and_load(sim, csv, path_csv=path, start=f"0,{e0},0", seed=1, preset="none",
                         controller_input="truth", robot=robot, nav=nav)
    s = df["truth_x"].to_numpy()
    keep = s <= 12.0 * L  # 远离终点:剩余路程不足 L 时 look-ahead 点钉在终点上
    return s[keep] / L, df["truth_y"].to_numpy()[keep] / e0


def run_lateral_step(sim, tau: float, v: float, freq: float, scaled: bool, L: float = 0.3,
                     h: float = 0.05) -> tuple[np.ndarray, np.ndarray, float]:
    """E1b/E1c:先在 y = 0 上跑 3 m,再跟 y = h 的台阶。返回 (s/L, e/e0, 本次的 L)。"""
    controller = {"control_frequency": freq, **classic_pp(L, v), **no_approach_slowdown(v), **FAST_SMOOTHER}
    if scaled:
        controller.update({"use_velocity_scaled_lookahead": True, "lookahead_time": 1.0,
                           "min_lookahead": 0.1, "max_lookahead": 0.6})
        L = 1.0 * v
    name = f"e1b_tau{tau}_v{v}_f{freq:g}_{'scaled' if scaled else f'L{L}'}"
    robot, nav = variant_configs(name, robot={**IDEAL_ROBOT, "actuator_time_constant": tau},
                                 nav={"controller": controller}, out_dir=OUT / "configs")
    run_length = 35.0 * L
    path = write_path_csv([(-3.0, 0.0), (0.0, 0.0), (0.0, h), (run_length, h)],
                          OUT / "paths" / f"step_{run_length:.2f}.csv", f"E1b lateral step h = {h}")
    csv = OUT / f"{name}.csv"
    _, df = run_and_load(sim, csv, path_csv=path, seed=1, preset="none", controller_input="truth",
                         robot=robot, nav=nav)
    x = df["truth_x"].to_numpy()
    # look-ahead 点越过台阶(x ≈ −L)时线性系统从 e = −h 出发;s 从那里起算。
    s = x + L
    keep = (s >= 0.0) & (x <= run_length - 1.5 * L)
    e = (df["truth_y"].to_numpy() - h) / (-h)
    return s[keep] / L, e[keep], L


# ---- 主流程 -----------------------------------------------------------------------

def e1a(sim, ax_small, ax_large) -> None:
    print("\nE1a — τ = 0, initial offset, distance domain (analytic: overshoot "
          f"{100 * OVERSHOOT_RATIO:.2f}% at s = πL, 2% settling {EXACT_SETTLE:.2f} L, envelope bound "
          f"{SETTLE_ENVELOPE:.2f} L)")
    print(f"{'e0':>5} {'L':>4} {'v':>4} | {'overshoot':>9} {'dev':>6} | {'s_min/L':>7} {'dev':>6} | "
          f"{'settle/L':>8} {'dev':>6}")
    sigma = np.linspace(0, 12, 1200)
    ax_small.plot(sigma, analytic_offset(sigma), color="black", lw=2.2, label="linearized, analytic", zorder=5)
    ax_large.plot(sigma, analytic_offset(sigma), color="black", lw=2.2, label="linearized, analytic", zorder=5)
    cmap = plt.get_cmap("viridis")
    for e0, ax in ((0.05, ax_small), (0.3, ax_large)):
        for i, L in enumerate((0.2, 0.3, 0.5)):
            for j, v in enumerate((0.1, 0.3, 0.5)):
                x, e = run_offset(sim, L, v, e0)
                ov, smin, settle = response_metrics(x, e)
                print(f"{e0:5.2f} {L:4.1f} {v:4.1f} | {100 * ov:8.2f}% {100 * (ov / OVERSHOOT_RATIO - 1):+5.1f}% | "
                      f"{smin:7.2f} {100 * (smin / math.pi - 1):+5.1f}% | {settle:8.2f} "
                      f"{100 * (settle / EXACT_SETTLE - 1):+5.1f}%")
                if e0 == 0.3 and v != 0.3:
                    continue  # 大偏置图只画 v = 0.3,三条 L
                ax.plot(x, e, color=cmap(0.15 + 0.35 * i), lw=1.2, alpha=0.85,
                        ls=("-", "--", ":")[j], label=f"L = {L} m, v = {v} m/s")
    for ax, e0 in ((ax_small, 0.05), (ax_large, 0.3)):
        ax.axhline(0.0, color="gray", lw=0.6)
        ax.axhline(-OVERSHOOT_RATIO, color="tab:red", lw=0.8, ls="--")
        ax.axvline(math.pi, color="tab:red", lw=0.6, ls=":")
        ax.set_xlim(0, 8)
        ax.set_xlabel("distance travelled s / L")
        ax.set_ylabel("cross-track error e / e₀")
        ax.legend(fontsize=7, ncol=1 if e0 == 0.3 else 2, loc="upper right")
    ax_small.set_title("E1a: e₀ = 5 cm — 9 (L, v) runs collapse onto one curve")
    ax_large.set_title("E1a: e₀ = 30 cm — outside the linear region")
    ax_small.annotate("−e^(−π) ≈ −4.3% at s = πL", xy=(math.pi, -OVERSHOOT_RATIO), xytext=(4.2, 0.22),
                      arrowprops={"arrowstyle": "->", "lw": 0.8}, fontsize=8)


def e1b(sim, ax_resp, ax_ov) -> None:
    print("\nE1b — first-order actuator lag, lateral step (L = 0.3 m, v = 0.3 m/s, T_L = 1 s)")
    print(f"{'freq':>5} {'tau':>5} {'r_eff':>6} | {'overshoot':>9} {'model':>7} | {'settle/L':>8} {'model':>7}")
    taus = (0.0, 0.1, 0.2, 0.3, 0.5, 0.7)
    cmap = plt.get_cmap("plasma")
    sigma = np.linspace(0, 30, 3001)
    r_grid = np.linspace(0.0, 0.95, 96)
    model_ov = []
    model_settle = []
    for r in r_grid:
        e = lag_model(r, SIGMA_DENSE)
        ov, _, settle = response_metrics(SIGMA_DENSE, e)
        model_ov.append(ov)
        model_settle.append(settle)
    ax_ov.plot(r_grid, 100 * np.asarray(model_ov), color="black", lw=1.8, label="linear model (3rd order)")
    for freq, marker in ((100.0, "o"), (20.0, "s")):
        for k, tau in enumerate(taus):
            x, e, L = run_lateral_step(sim, tau, 0.3, freq, scaled=False)
            r_eff = (tau + 0.5 / freq) / (L / 0.3)
            ov, _, settle = response_metrics(x, e)
            m = lag_model(r_eff, SIGMA_DENSE)
            mov, _, msettle = response_metrics(SIGMA_DENSE, m)
            print(f"{freq:5.0f} {tau:5.2f} {r_eff:6.3f} | {100 * ov:8.2f}% {100 * mov:6.2f}% | "
                  f"{settle:8.2f} {msettle:7.2f}")
            ax_ov.plot(r_eff, 100 * ov, marker, color=cmap(k / len(taus)), ms=7 if freq == 100 else 6,
                       mfc="none" if freq == 20 else cmap(k / len(taus)),
                       label=f"sim, {freq:.0f} Hz control" if k == 0 else None)
            if freq == 100.0:
                ax_resp.plot(x, e, color=cmap(k / len(taus)), lw=1.3, label=f"τ = {tau:.1f} s (r = {r_eff:.2f})")
                ax_resp.plot(sigma, lag_model(r_eff, sigma), color=cmap(k / len(taus)), lw=0.8, ls="--")
    ax_resp.axhline(0.0, color="gray", lw=0.6)
    ax_resp.set_xlim(0, 20)
    ax_resp.set_xlabel("distance after the step s / L")
    ax_resp.set_ylabel("e / e₀")
    ax_resp.set_title("E1b: lag ratio r = τ/T_L (solid: sim 100 Hz, dashed: linear model)")
    ax_resp.legend(fontsize=7, loc="upper right")
    ax_ov.axvline(0.2, color="tab:green", lw=0.8, ls="--")
    ax_ov.text(0.21, 45, "design rule\nr ≤ 0.2", fontsize=8, color="tab:green")
    ax_ov.set_xlabel("effective lag ratio r = (τ + T_c/2) / T_L")
    ax_ov.set_ylabel("reverse overshoot [%]")
    ax_ov.set_title("E1b: overshoot vs lag ratio (● 100 Hz, □ 20 Hz)")
    ax_ov.set_xlim(0, 0.95)
    ax_ov.set_ylim(0, 60)
    ax_ov.legend(fontsize=8, loc="upper left")


def e1c(sim, ax) -> None:
    print("\nE1c — velocity-scaled look-ahead keeps r constant (τ = 0.2 s, 100 Hz)")
    print(f"{'mode':>14} {'v':>5} {'L':>5} {'r':>5} | {'overshoot':>9}")
    for scaled, ls in ((False, "-"), (True, "--")):
        for k, v in enumerate((0.15, 0.3, 0.45)):
            x, e, L = run_lateral_step(sim, 0.2, v, 100.0, scaled=scaled)
            r = (0.2 + 0.005) * v / L
            ov, _, _ = response_metrics(x, e)
            mode = "L = T_L·v" if scaled else "fixed L = 0.3"
            print(f"{mode:>14} {v:5.2f} {L:5.2f} {r:5.2f} | {100 * ov:8.2f}%")
            color = ("tab:blue", "tab:orange", "tab:red")[k]
            ax.plot(x, e, color=color, ls=ls, lw=1.4 if scaled else 1.1,
                    label=f"{mode}, v = {v} (r = {r:.2f})")
    ax.axhline(0.0, color="gray", lw=0.6)
    ax.set_xlim(0, 12)
    ax.set_xlabel("distance after the step s / L")
    ax.set_ylabel("e / e₀")
    ax.set_title("E1c: τ = 0.2 s — fixed L (solid) vs velocity-scaled L (dashed)")
    ax.legend(fontsize=7, loc="upper right")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.parse_args()
    sim = find_sim()
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    fig, (ax_s, ax_l) = plt.subplots(1, 2, figsize=(13, 4.8))
    e1a(sim, ax_s, ax_l)
    fig.suptitle("MiniNav V4 — E1a: Pure Pursuit straight-line response vs linearized theory "
                 "(noise-free, τ = 0, 100 Hz)")
    fig.tight_layout()
    fig.savefig(RESULTS_DIR / "e1_step_response.png", dpi=130)
    print(f"wrote {RESULTS_DIR / 'e1_step_response.png'}")

    fig, (ax_r, ax_o, ax_c) = plt.subplots(1, 3, figsize=(18, 4.8))
    e1b(sim, ax_r, ax_o)
    e1c(sim, ax_c)
    fig.suptitle("MiniNav V4 — E1b/E1c: actuator lag, zero-order hold and velocity-scaled look-ahead")
    fig.tight_layout()
    fig.savefig(RESULTS_DIR / "e1_lag.png", dpi=130)
    print(f"wrote {RESULTS_DIR / 'e1_lag.png'}")


if __name__ == "__main__":
    main()
