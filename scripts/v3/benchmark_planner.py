#!/usr/bin/env python3
"""
benchmark_planner.py — 规划耗时基准(对齐里程碑 200×200 ≤ 50ms,Release build)。

为每个尺寸 N 生成一张 N×N 的程序化测试地图(确定性:固定 numpy 种子的随机
障碍 + 保证起止 free + 保证连通的"安全走廊"),反复驱动 `sim --map ... --no-viz`,
从 stdout 的 `plan_time_ms=` 解析耗时,聚合成分布。

产出:
  - results/v3/planner_timing.png   各尺寸耗时箱线 + 50ms 参考线
  - stdout 表格(中位数 / p95 / max,以及 200×200 是否达标)

注意:50ms 指标针对 **Release** build。默认用 build/clang18-release/sim;
找不到则回退 debug(debug 会慢很多,仅作功能冒烟,不代表达标)。

Run:
    cmake --build --preset build-release -j
    python scripts/v3/benchmark_planner.py --sizes 50 100 200 --trials 10
"""

from __future__ import annotations

import argparse
import re
import subprocess
import tempfile
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

TIME_RE = re.compile(r"plan_time_ms=([0-9.eE+-]+)")
EXPANDED_RE = re.compile(r"expanded_nodes=([0-9]+)")
SUCCESS_RE = re.compile(r"success=([01])")


def make_map(n: int, seed: int, obstacle_frac: float, out_dir: Path) -> Path:
    """生成 n×n 程序化地图:外墙 + 随机障碍,但沿主对角保留一条 free 走廊保证可达。"""
    rng = np.random.default_rng(seed)
    occ = rng.random((n, n)) < obstacle_frac  # True = 障碍
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True  # 外墙
    # 保留一条 L 形安全走廊连通 start↔goal,确保可达。
    # 注意 PGM 行序自上而下、C++ 加载时 y 翻转:occ 行 r ↔ grid_y = n-1-r。
    # start = grid(1,1) ↔ occ[n-2, 1];goal = grid(n-2,n-2) ↔ occ[1, n-2]。
    # L 形(grid 系):grid_y=1 底排 + grid_x=n-2 右列,在角 (n-2,1) 处相接。
    occ[-2, 1:-1] = False  # grid_y=1 底排(occ 行 n-2)
    occ[1:-1, -2] = False  # grid_x=n-2 右列

    img = np.where(occ, 0, 255).astype(int)  # 0=占据 255=free
    pgm = out_dir / f"bench_{n}.pgm"
    yaml = out_dir / f"bench_{n}.yaml"
    lines = ["P2", f"# benchmark map {n}x{n} seed={seed}", f"{n} {n}", "255"]
    lines += [" ".join(f"{v:3d}" for v in row) for row in img]
    pgm.write_text("\n".join(lines) + "\n")
    yaml.write_text(
        f"image: {pgm.name}\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n"
        f"occupied_thresh: 0.65\nfree_thresh: 0.25\nnegate: 0\n"
    )
    return yaml


def run_once(sim_bin: Path, map_yaml: Path, n: int, out_csv: Path) -> tuple[float, int, int]:
    # start/goal 落在保留走廊的两端(cell (1,1) 与 (n-2,n-2),world = (gx+0.5)*res)。
    res = 0.05
    start = f"{1.5 * res},{1.5 * res}"
    goal = f"{(n - 1.5) * res},{(n - 1.5) * res}"
    cmd = [str(sim_bin), "--map", str(map_yaml), "--start", start, "--goal", goal,
           "--connectivity", "8", "--heuristic", "octile",
           "--inflation-radius", "0.0", "--no-viz", "--out", str(out_csv)]
    res_proc = subprocess.run(cmd, check=True, capture_output=True, text=True)
    blob = res_proc.stdout + res_proc.stderr
    t = float(TIME_RE.search(blob).group(1))
    exp = int(EXPANDED_RE.search(blob).group(1))
    ok = int(SUCCESS_RE.search(blob).group(1))
    return t, exp, ok


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sizes", nargs="+", type=int, default=[50, 100, 200])
    ap.add_argument("--trials", type=int, default=10)
    ap.add_argument("--obstacle-frac", type=float, default=0.20)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--sim-bin", type=Path, default=Path("build/clang18-release/sim"))
    ap.add_argument("--output", type=Path, default=Path("results/v3"))
    args = ap.parse_args()

    sim_bin = args.sim_bin
    build_note = "release"
    if not sim_bin.exists():
        sim_bin = Path("build/clang18-debug/sim")
        build_note = "DEBUG (not milestone-representative)"
    args.output.mkdir(parents=True, exist_ok=True)

    timings: dict[int, list[float]] = {}
    print(f"sim = {sim_bin} [{build_note}]")
    print(f"{'size':>6}{'trials':>8}{'median[ms]':>12}{'p95[ms]':>10}"
          f"{'max[ms]':>10}{'expanded':>10}")
    print("-" * 56)
    with tempfile.TemporaryDirectory() as td:
        td_path = Path(td)
        out_csv = td_path / "bench_path.csv"
        for n in args.sizes:
            ts: list[float] = []
            last_exp = 0
            for k in range(args.trials):
                # 每个 trial 同尺寸不同种子,采样不同障碍布局。
                map_yaml = make_map(n, args.seed + k, args.obstacle_frac, td_path)
                t, exp, ok = run_once(sim_bin, map_yaml, n, out_csv)
                if ok:
                    ts.append(t)
                    last_exp = exp
            arr = np.array(ts)
            timings[n] = ts
            print(f"{n:>6}{len(ts):>8}{np.median(arr):>12.3f}"
                  f"{np.percentile(arr, 95):>10.3f}{arr.max():>10.3f}{last_exp:>10}")

    # 箱线图
    fig, ax = plt.subplots(figsize=(8, 5))
    sizes = sorted(timings)
    ax.boxplot([timings[s] for s in sizes], tick_labels=[f"{s}×{s}" for s in sizes])
    ax.axhline(50.0, color="#e64646", ls="--", lw=1.5, label="50 ms milestone")
    ax.set_ylabel("plan time [ms]")
    ax.set_xlabel("map size")
    ax.set_title(f"A* planning time vs map size ({build_note} build, "
                 f"{args.trials} trials/size, 8-conn octile)")
    ax.legend()
    ax.grid(True, axis="y", ls=":", alpha=0.4)
    out = args.output / "planner_timing.png"
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    print(f"\nwrote {out}")

    if 200 in timings and timings[200]:
        p95 = np.percentile(timings[200], 95)
        verdict = "PASS" if (p95 <= 50.0 and build_note == "release") else "see note"
        print(f"200×200 p95 = {p95:.3f} ms  -> {verdict}")


if __name__ == "__main__":
    main()
