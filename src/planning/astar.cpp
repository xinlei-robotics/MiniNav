module;

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <queue>
#include <vector>

module mininav.planning.astar;

import mininav.core.types;
import mininav.planning.grid_types;
import mininav.planning.occupancy_grid;
import mininav.planning.inflation;

namespace mininav::planning
{
    namespace
    {
        // 代价梯度衰减:cost(d) = exp(-k * (d - 1)),d 为到最近障碍的 cell 距离。
        // 紧邻障碍(d=1)代价 1,每远离 1 cell 衰减 e 倍,远处趋近 0。固定衰减率,
        // 强度由 cfg.cost_weight 调。
        constexpr double kCostDecayPerCell = 1.0;

        // 4 / 8 连通邻居偏移;前 4 个是正交,后 4 个是对角。
        constexpr int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        constexpr int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

        [[nodiscard]] bool traversable(const OccupancyGrid& grid, const PlannerConfig& cfg,
                                       const GridCoord c)
        {
            const std::int8_t v = grid.at(c); // 越界 → kOccupied
            if (v == kOccupied)
            {
                return false;
            }
            if (v == kUnknown)
            {
                return cfg.allow_unknown;
            }
            return true; // kFree
        }

        [[nodiscard]] double heuristic(const Heuristic h, const GridCoord a, const GridCoord b)
        {
            const double dx = std::abs(static_cast<double>(a.x - b.x));
            const double dy = std::abs(static_cast<double>(a.y - b.y));
            switch (h)
            {
            case Heuristic::Manhattan:
                return dx + dy;
            case Heuristic::Euclidean:
                return std::hypot(dx, dy);
            case Heuristic::Octile:
                return (dx + dy) + (std::numbers::sqrt2 - 2.0) * std::min(dx, dy);
            }
            return 0.0;
        }

        [[nodiscard]] std::vector<double> compute_gradient_cost(const OccupancyGrid& grid)
        {
            const std::vector<double> dist = obstacle_distance_cells(grid);
            std::vector<double> cost(dist.size(), 0.0);
            for (std::size_t i = 0; i < dist.size(); ++i)
            {
                if (std::isinf(dist[i]))
                {
                    continue; // 没有障碍影响 → 0
                }
                const double c = std::exp(-kCostDecayPerCell * (dist[i] - 1.0));
                cost[i] = std::min(1.0, c); // 占据 cell(d=0)夹到 1,反正不会被进入
            }
            return cost;
        }

        // A* open set 节点。tie-break:f 相等时优先 h 小的(更靠近目标,减少展开)。
        struct Node
        {
            double f{0.0};
            double h{0.0};
            int idx{0};
        };

        struct NodeWorse
        {
            [[nodiscard]] bool operator()(const Node& a, const Node& b) const noexcept
            {
                if (a.f != b.f)
                {
                    return a.f > b.f; // 小 f 先出队(最小堆)
                }
                return a.h > b.h; // tie-break:小 h 先出队
            }
        };
    } // namespace

    AStarPlanner::AStarPlanner(OccupancyGrid grid, PlannerConfig cfg)
        : grid_{inflate(grid, cfg.inflation_radius)}, cfg_{cfg}
    {
        if (cfg_.cost_weight > 0.0)
        {
            cost_ = compute_gradient_cost(grid_);
        }
    }

    PlanResult AStarPlanner::plan(const Pose2D& start, const Pose2D& goal) const
    {
        const auto t0 = std::chrono::steady_clock::now();
        PlanResult result{};

        const int w = grid_.width();
        const int h = grid_.height();

        const GridCoord start_c = grid_.world_to_grid(start.position());
        const GridCoord goal_c = grid_.world_to_grid(goal.position());

        const auto finish = [&](const bool success) {
            const auto t1 = std::chrono::steady_clock::now();
            result.success = success;
            result.plan_time_ms =
                std::chrono::duration<double, std::milli>(t1 - t0).count();
            return result;
        };

        // 起止点必须在界内且可走(障碍上的起止点直接判失败,不做就近吸附)。
        if (!grid_.in_bounds(start_c) || !grid_.in_bounds(goal_c) ||
            !traversable(grid_, cfg_, start_c) || !traversable(grid_, cfg_, goal_c))
        {
            return finish(false);
        }

        const auto encode = [w](const GridCoord c) { return c.y * w + c.x; };
        const int start_idx = encode(start_c);
        const int goal_idx = encode(goal_c);

        // 起止同格:单点路径(格心),长度 0。
        if (start_idx == goal_idx)
        {
            const Eigen::Vector2d p = grid_.grid_to_world(start_c);
            result.path.poses.emplace_back(p.x(), p.y(), start.yaw());
            result.expanded_nodes = 1;
            return finish(true);
        }

        const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
        constexpr double kInf = std::numeric_limits<double>::infinity();
        std::vector<double> g(n, kInf);
        std::vector<int> parent(n, -1);
        std::vector<char> closed(n, 0);

        const int neighbor_count = (cfg_.connectivity == Connectivity::Eight) ? 8 : 4;

        std::priority_queue<Node, std::vector<Node>, NodeWorse> open;
        g[static_cast<std::size_t>(start_idx)] = 0.0;
        const double h0 = heuristic(cfg_.heuristic, start_c, goal_c);
        open.push(Node{h0, h0, start_idx});

        bool found = false;
        while (!open.empty())
        {
            const Node cur = open.top();
            open.pop();
            const std::size_t uc = static_cast<std::size_t>(cur.idx);
            if (closed[uc])
            {
                continue; // 过期堆条目
            }
            closed[uc] = 1;
            ++result.expanded_nodes;

            if (cur.idx == goal_idx)
            {
                found = true;
                break; // 一致启发式下,首次出队目标即最优
            }

            const int cx = cur.idx % w;
            const int cy = cur.idx / w;

            for (int k = 0; k < neighbor_count; ++k)
            {
                const int nx = cx + kDx[k];
                const int ny = cy + kDy[k];
                const GridCoord nc{nx, ny};
                if (!traversable(grid_, cfg_, nc))
                {
                    continue;
                }

                const bool diagonal = (kDx[k] != 0 && kDy[k] != 0);
                if (diagonal)
                {
                    // 防穿角:对角移动要求两个正交邻居都可走。
                    if (!traversable(grid_, cfg_, GridCoord{cx + kDx[k], cy}) ||
                        !traversable(grid_, cfg_, GridCoord{cx, cy + kDy[k]}))
                    {
                        continue;
                    }
                }

                const int nidx = ny * w + nx;
                const std::size_t un = static_cast<std::size_t>(nidx);
                if (closed[un])
                {
                    continue;
                }

                const double step = diagonal ? std::numbers::sqrt2 : 1.0;
                const double extra = cost_.empty() ? 0.0 : cfg_.cost_weight * cost_[un];
                const double tentative = g[uc] + step + extra;

                if (tentative < g[un])
                {
                    g[un] = tentative;
                    parent[un] = cur.idx;
                    const double hn = heuristic(cfg_.heuristic, nc, goal_c);
                    open.push(Node{tentative + hn, hn, nidx});
                }
            }
        }

        if (!found)
        {
            return finish(false);
        }

        // 回溯 cell 序列(goal → start),反转成 start → goal。
        std::vector<int> cells;
        for (int c = goal_idx; c != -1; c = parent[static_cast<std::size_t>(c)])
        {
            cells.push_back(c);
        }
        std::reverse(cells.begin(), cells.end());

        result.path.poses.reserve(cells.size());
        for (const int c : cells)
        {
            const GridCoord gc{c % w, c / w};
            const Eigen::Vector2d p = grid_.grid_to_world(gc);
            result.path.poses.emplace_back(p.x(), p.y(), 0.0);
        }

        // yaw:每个 waypoint 指向下一个;末点沿用前一段方向。
        auto& poses = result.path.poses;
        for (std::size_t i = 0; i + 1 < poses.size(); ++i)
        {
            const double yaw = std::atan2(poses[i + 1].y() - poses[i].y(),
                                          poses[i + 1].x() - poses[i].x());
            poses[i].set_yaw(yaw);
        }
        if (poses.size() >= 2)
        {
            poses.back().set_yaw(poses[poses.size() - 2].yaw());
        }

        return finish(true);
    }
}
