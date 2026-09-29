export module mininav.planning.grid_types;

export namespace mininav::planning
{
    // ---------------------------------------------------------------------------
    // GridCoord: 离散栅格索引
    //   x = col(列), y = row(行),均为整型 cell 下标。
    // world↔grid 的转换由 OccupancyGrid 持有(分辨率 / 原点),见 PR1。
    // ---------------------------------------------------------------------------
    struct GridCoord
    {
        int x{0}; // col
        int y{0}; // row

        bool operator==(const GridCoord&) const noexcept = default;
    };

    // ---------------------------------------------------------------------------
    // Heuristic: A* 启发式选择。
    //   admissible 性(≤ 真实代价,保证最优)的证明留给 docs/math/astar_planning.md。
    // ---------------------------------------------------------------------------
    enum class Heuristic
    {
        Manhattan,
        Euclidean,
        Octile,
    };

    // ---------------------------------------------------------------------------
    // Connectivity: 栅格邻接度。数值即邻居数,便于 YAML 序列化为 4 / 8。
    // ---------------------------------------------------------------------------
    enum class Connectivity
    {
        Four = 4,
        Eight = 8,
    };

    // ---------------------------------------------------------------------------
    // is_admissible: 启发式在给定连通度下是否 admissible(从不高估真实代价)。
    //
    // 唯一的反例是 Manhattan + 8 连通:对角一步的真实代价是 √2,Manhattan 却记为
    // 2 —— 高估,A* 因此失去最优性保证(office500 实测比最优路径长 1.13 cell)。
    // Euclidean / Octile 在 4、8 连通下都不高估;Manhattan 在无障碍 4 连通栅格上
    // 恰好等于真实最短代价,有障碍时是它的下界。推导见 docs/math/astar_planning.md。
    // ---------------------------------------------------------------------------
    [[nodiscard]] constexpr bool is_admissible(const Heuristic h, const Connectivity c) noexcept
    {
        return !(h == Heuristic::Manhattan && c == Connectivity::Eight);
    }

    // ---------------------------------------------------------------------------
    // PlannerConfig: 规划器配置(来自 planner.yaml)。
    //   PR0 仅定义数据形态;inflation_radius / cost_weight 等字段在 PR3 / PR4
    //   才被膨胀与 A* 算法消费。cost_weight 在 MVP 布尔膨胀下为 0。
    // ---------------------------------------------------------------------------
    struct PlannerConfig
    {
        double inflation_radius{0.0};                 // 米
        Heuristic heuristic{Heuristic::Euclidean};
        Connectivity connectivity{Connectivity::Eight};
        bool allow_unknown{false};
        double cost_weight{0.0};                      // 代价梯度权重,MVP 下为 0
    };
}
