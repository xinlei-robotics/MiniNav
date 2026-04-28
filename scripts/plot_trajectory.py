import csv
from pathlib import Path

import matplotlib.pyplot as plt


def load_csv(path: Path):
    xs, ys = [], []
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))
    return xs, ys


def main():
    data_path = Path("../data/traj.csv")
    result_dir = Path("../results")
    result_dir.mkdir(parents=True, exist_ok=True)

    xs, ys = load_csv(data_path)

    plt.figure()
    plt.plot(xs, ys)
    plt.xlabel("x")
    plt.ylabel("y")
    plt.axis("equal")
    plt.title("MiniNav V0 Trajectory")
    plt.grid(True)

    out_path = result_dir / "traj_v0.png"
    plt.savefig(out_path, dpi=150)
    print(f"Saved figure to {out_path}")


if __name__ == "__main__":
    main()