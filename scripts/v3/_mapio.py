#!/usr/bin/env python3
"""
共享工具:在 Python 端复刻 C++ `mininav.planning.map_io` 的地图加载语义,
并解析规划器产出的 path.csv。

复刻的目的是让 optimality_check.py 的 Dijkstra/BFS ground-truth 跑在与 C++
A* **逐 cell 一致**的占据栅格上 —— 否则"最优性偏差"会被两端地图不一致污染。

约定(对齐 map_io.cpp):
  - 占据三值:kFree=0 / kOccupied=100 / kUnknown=-1。
  - occ = negate ? norm : (1 - norm),norm = pixel/maxval;暗像素 → 占据。
  - y 轴翻转:图像第一行在顶部 = world 最大 y = grid 最高 row。
    返回的 occ 数组以 grid 行序存储:occ[grid_y][col],grid_y=0 对应最小 world y。
  - world_to_grid:gx = floor((x-ox)/res),gy = floor((y-oy)/res)。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

K_FREE = 0
K_OCCUPIED = 100
K_UNKNOWN = -1


@dataclass
class Grid:
    occ: np.ndarray  # int8, shape (height, width); 行 = grid_y(0 = 最小 world y)
    resolution: float
    origin: tuple[float, float]  # 左下角 world 坐标 (x, y)

    @property
    def height(self) -> int:
        return self.occ.shape[0]

    @property
    def width(self) -> int:
        return self.occ.shape[1]

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        gx = math.floor((x - self.origin[0]) / self.resolution)
        gy = math.floor((y - self.origin[1]) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        wx = self.origin[0] + (gx + 0.5) * self.resolution
        wy = self.origin[1] + (gy + 0.5) * self.resolution
        return wx, wy

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height


def _read_pgm(path: Path) -> tuple[int, int, int, np.ndarray]:
    """解析 PGM(P2 ASCII / P5 binary),跳过 '#' 注释。返回 (w, h, maxval, pixels)。

    pixels 为长度 w*h 的 np.int16 数组,行优先、自上而下(图像原始行序),与
    C++ map_io 的 Pgm.pixels 一致。"""
    raw = path.read_bytes()
    if raw[:2] not in (b"P2", b"P5"):
        raise ValueError(f"{path}: not a PGM (bad magic, expected P2/P5)")
    ascii_pgm = raw[:2] == b"P2"

    # 解析 header 的三个整型 token(width/height/maxval),跳过 '#' 注释。
    pos = 2
    header: list[int] = []
    while len(header) < 3:
        while pos < len(raw) and raw[pos : pos + 1].isspace():
            pos += 1
        if raw[pos : pos + 1] == b"#":  # 注释行
            while pos < len(raw) and raw[pos : pos + 1] != b"\n":
                pos += 1
            continue
        start = pos
        while pos < len(raw) and raw[pos : pos + 1].isdigit():
            pos += 1
        header.append(int(raw[start:pos]))
    w, h, maxval = header
    count = w * h

    if ascii_pgm:
        body = raw[pos:].split()
        pixels = np.array([int(t) for t in body[:count]], dtype=np.int16)
    else:
        pos += 1  # maxval 后恰好一个空白分隔
        pixels = np.frombuffer(raw, dtype=np.uint8, count=count, offset=pos).astype(np.int16)

    if pixels.size != count:
        raise ValueError(f"{path}: pixel count {pixels.size} != {count}")
    return w, h, maxval, pixels


def _parse_yaml(path: Path) -> dict:
    """极简 map.yaml 解析(无 yaml 依赖):key: value / origin: [a, b, c]。"""
    out: dict = {}
    for line in path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, val = line.split(":", 1)
        out[key.strip()] = val.strip()
    return out


def load_grid(map_yaml: Path) -> Grid:
    map_yaml = Path(map_yaml)
    meta = _parse_yaml(map_yaml)
    image = meta["image"].strip().strip('"')
    resolution = float(meta["resolution"])
    origin_raw = meta["origin"].strip().lstrip("[").rstrip("]")
    ox, oy = (float(v) for v in origin_raw.split(",")[:2])
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    free_thresh = float(meta.get("free_thresh", 0.25))
    negate = int(meta.get("negate", 0)) != 0

    img_path = map_yaml.parent / image
    w, h, maxval, pixels = _read_pgm(img_path)

    # 灰度 → 占据概率(暗=占据),三值化;矢量化以支持大图(500×500)。
    norm = pixels.reshape(h, w).astype(float) / maxval
    occ_prob = norm if negate else (1.0 - norm)
    cls = np.full((h, w), K_UNKNOWN, dtype=np.int8)
    cls[occ_prob > occupied_thresh] = K_OCCUPIED
    cls[occ_prob < free_thresh] = K_FREE
    occ = cls[::-1].copy()  # y 翻转:图像第一行在顶部 = grid 最高 row
    return Grid(occ=occ, resolution=resolution, origin=(ox, oy))


def inflate(grid: Grid, radius_m: float) -> np.ndarray:
    """复刻 C++ `inflate`:到最近障碍欧氏距离 ≤ radius 的 free/unknown cell 标占据。

    返回与 grid.occ 同形、同三值编码的 int8 数组(占据 cell 保持占据)。
    radius ≤ 0 时返回原图副本。仅用于 Python 端的可视化/搜索复刻。"""
    occ = grid.occ.copy()
    if radius_m <= 0.0:
        return occ
    r_cells = radius_m / grid.resolution
    r = int(math.ceil(r_cells))
    offs = [
        (dx, dy)
        for dy in range(-r, r + 1)
        for dx in range(-r, r + 1)
        if dx * dx + dy * dy <= r_cells * r_cells
    ]
    obst = grid.occ == K_OCCUPIED
    out = occ.copy()
    ys, xs = np.where(obst)
    H, W = grid.height, grid.width
    for cy, cx in zip(ys, xs):
        for dx, dy in offs:
            ny, nx = cy + dy, cx + dx
            if 0 <= ny < H and 0 <= nx < W and out[ny, nx] != K_OCCUPIED:
                out[ny, nx] = K_OCCUPIED
    return out


def read_path_csv(path: Path) -> tuple[dict, np.ndarray]:
    """解析 path.csv:返回 (header_meta, waypoints Nx3 [x, y, yaw])。"""
    meta: dict = {}
    rows: list[tuple[float, float, float]] = []
    for line in Path(path).read_text().splitlines():
        if line.startswith("#"):
            body = line[1:].strip()
            if "=" in body:
                k, v = body.split("=", 1)
                meta[k.strip()] = v.strip()
            continue
        if line.startswith("idx") or not line.strip():
            continue
        parts = line.split(",")
        rows.append((float(parts[1]), float(parts[2]), float(parts[3])))
    return meta, np.array(rows, dtype=float).reshape(-1, 3)
