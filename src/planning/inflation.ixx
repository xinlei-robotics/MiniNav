module;

#include <vector>

export module mininav.planning.inflation;

import mininav.planning.occupancy_grid;

export namespace mininav::planning
{
    // ---------------------------------------------------------------------------
    // obstacle_distance_cells: 多源距离变换。
    //
    // 以所有 kOccupied cell 为源,算出每个 cell 到**最近障碍**的欧氏距离(单位:
    // cell)。占据 cell 自身距离 0;没有任何障碍时全部为 +inf。
    //
    // 算法对齐 ROS costmap_2d 的膨胀层:优先队列按距离递增出队,每个 cell 记住
    // 其"最近障碍坐标"并向 8 邻居传播,邻居距离 = 到该记忆源的直线距离。这给出
    // (近乎)精确的欧氏距离,而非 BFS 的 Manhattan/Chebyshev 阶梯。
    //
    // 返回值行优先(row-major):index = y * width + x,与 OccupancyGrid 内部
    // 存储一致。这是膨胀(本文件)与未来代价梯度(PR4 的 distance→uint8 cost,
    // cost_weight)共同的底座。
    // ---------------------------------------------------------------------------
    [[nodiscard]] std::vector<double> obstacle_distance_cells(const OccupancyGrid& grid);

    // ---------------------------------------------------------------------------
    // inflate: 障碍物膨胀,产出 A* 可直接消费的 costmap。
    //
    // MVP 布尔膨胀:到最近障碍欧氏距离 ≤ radius_m 的非占据 cell 一律标 kOccupied,
    // 把"机器人有半径 + 安全裕度"编码进地图。占据 cell 保持占据;其余(free /
    // unknown)若不在半径内则原样保留。
    //
    // radius_m ≤ 0 时返回原图副本(costmap == 原图)。半径换算成 cell 数为
    // radius_m / resolution。
    // ---------------------------------------------------------------------------
    [[nodiscard]] OccupancyGrid inflate(const OccupancyGrid& grid, double radius_m);
}
