module;

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

module mininav.planning.path_smoothing;

import mininav.core.types;
import mininav.core.path;
import mininav.core.logger;
import mininav.planning.grid_types;
import mininav.planning.occupancy_grid;

namespace mininav::planning
{
    namespace
    {
        // 两个方向的下一次跨界参数 t 相差小于此值时,视为恰好穿过格角。
        constexpr double kCornerTie = 1e-9;

        // DDA 的单轴状态:下一次跨越该轴格线时的参数 t,以及每跨一格 t 的增量。
        struct Axis
        {
            int step{0};
            double t_max{std::numeric_limits<double>::infinity()};
            double t_delta{std::numeric_limits<double>::infinity()};
        };

        [[nodiscard]] Axis make_axis(const double from, const double delta, const int cell) noexcept
        {
            Axis axis{};
            if (delta > 0.0)
            {
                axis.step = 1;
                axis.t_max = (static_cast<double>(cell) + 1.0 - from) / delta;
                axis.t_delta = 1.0 / delta;
            }
            else if (delta < 0.0)
            {
                axis.step = -1;
                axis.t_max = (static_cast<double>(cell) - from) / delta;
                axis.t_delta = -1.0 / delta;
            }
            return axis;
        }

        using Points = std::vector<Eigen::Vector2d>;

        // 首尾替换:替换后与相邻 waypoint 的连线不可通行时,保留格心作过渡点。
        [[nodiscard]] Points snap_endpoints(Points pts, const Eigen::Vector2d& start,
                                            const Eigen::Vector2d& goal, const auto& is_free)
        {
            if (pts.size() == 1)
            {
                // 起止同格:A* 只给了一个格心;同一 cell 内的线段必可通行。
                return start == goal ? Points{start} : Points{start, goal};
            }
            if (is_free(start, pts[1]))
            {
                pts.front() = start;
            }
            else
            {
                pts.insert(pts.begin(), start);
            }
            if (is_free(pts[pts.size() - 2], goal))
            {
                pts.back() = goal;
            }
            else
            {
                pts.push_back(goal);
            }
            return pts;
        }

        // 视线捷径:贪心向前延伸,直到下一个 waypoint 不可见。输入相邻段都可通行时,
        // j = i + 1 总是可见,所以一定能推进;输入有坏段时退化为保留原段。
        [[nodiscard]] Points shortcut(const Points& pts, const auto& is_free)
        {
            Points out{pts.front()};
            std::size_t i = 0;
            while (i + 1 < pts.size())
            {
                std::size_t j = i + 1;
                while (j + 1 < pts.size() && is_free(pts[i], pts[j + 1]))
                {
                    ++j;
                }
                out.push_back(pts[j]);
                i = j;
            }
            return out;
        }

        // yaw:指向下一个 waypoint;末点沿用前一段方向;单点取 fallback_yaw。
        [[nodiscard]] Path to_path(const Points& pts, const double fallback_yaw)
        {
            Path path;
            path.poses.reserve(pts.size());
            for (std::size_t i = 0; i < pts.size(); ++i)
            {
                double yaw = fallback_yaw;
                if (i + 1 < pts.size())
                {
                    yaw = std::atan2(pts[i + 1].y() - pts[i].y(), pts[i + 1].x() - pts[i].x());
                }
                else if (i > 0)
                {
                    yaw = path.poses.back().yaw();
                }
                path.poses.emplace_back(pts[i], yaw);
            }
            return path;
        }
    }

    PathSmoothingConfig parse_path_smoothing_config(const std::string_view yaml_text)
    {
        PathSmoothingConfig cfg{};
        const YAML::Node node = YAML::Load(std::string{yaml_text});
        if (!node.IsDefined() || node.IsNull())
        {
            return cfg;
        }
        if (!node.IsMap())
        {
            throw std::runtime_error{"path_smoothing_config: expected a YAML mapping"};
        }
        for (const auto& entry : node)
        {
            const std::string key = entry.first.as<std::string>();
            bool* target = nullptr;
            if (key == "snap_endpoints")
            {
                target = &cfg.snap_endpoints;
            }
            else if (key == "shortcut")
            {
                target = &cfg.shortcut;
            }
            else
            {
                throw std::runtime_error{"path_smoothing_config: unknown key '" + key +
                                         "' (expected snap_endpoints, shortcut)"};
            }
            try
            {
                *target = entry.second.as<bool>();
            }
            catch (const YAML::Exception&)
            {
                throw std::runtime_error{"path_smoothing_config: '" + key + "' must be true or false"};
            }
        }
        return cfg;
    }

    bool segment_is_free(const OccupancyGrid& costmap, const Eigen::Vector2d& a,
                         const Eigen::Vector2d& b, const bool allow_unknown)
    {
        const auto passable = [&](const int x, const int y)
        {
            return costmap.is_traversable(GridCoord{x, y}, allow_unknown);
        };

        // 换到以 cell 为单位的连续栅格坐标:cell (x, y) 覆盖 [x, x+1) × [y, y+1)。
        const double res = costmap.resolution();
        const Eigen::Vector2d o = costmap.origin();
        const double ax = (a.x() - o.x()) / res;
        const double ay = (a.y() - o.y()) / res;
        const double bx = (b.x() - o.x()) / res;
        const double by = (b.y() - o.y()) / res;

        int cx = static_cast<int>(std::floor(ax));
        int cy = static_cast<int>(std::floor(ay));
        if (!passable(cx, cy))
        {
            return false;
        }

        Axis x_axis = make_axis(ax, bx - ax, cx);
        Axis y_axis = make_axis(ay, by - ay, cy);

        // 逐次跨越格线,直到下一次跨越落在线段之外(t > 1)。
        while (std::min(x_axis.t_max, y_axis.t_max) <= 1.0)
        {
            if (std::abs(x_axis.t_max - y_axis.t_max) <= kCornerTie)
            {
                // 恰好穿过格角:角上的两个正交邻居都算被经过。
                if (!passable(cx + x_axis.step, cy) || !passable(cx, cy + y_axis.step))
                {
                    return false;
                }
                cx += x_axis.step;
                cy += y_axis.step;
                x_axis.t_max += x_axis.t_delta;
                y_axis.t_max += y_axis.t_delta;
            }
            else if (x_axis.t_max < y_axis.t_max)
            {
                cx += x_axis.step;
                x_axis.t_max += x_axis.t_delta;
            }
            else
            {
                cy += y_axis.step;
                y_axis.t_max += y_axis.t_delta;
            }
            if (!passable(cx, cy))
            {
                return false;
            }
        }
        return true;
    }

    Path smooth_path(const Path& raw, const OccupancyGrid& costmap, const PlannerConfig& planner,
                     const Pose2D& start, const Pose2D& goal, const PathSmoothingConfig& cfg)
    {
        if (raw.empty())
        {
            return Path{};
        }
        const auto is_free = [&](const Eigen::Vector2d& a, const Eigen::Vector2d& b)
        {
            return segment_is_free(costmap, a, b, planner.allow_unknown);
        };

        Points pts;
        pts.reserve(raw.size());
        for (const Pose2D& p : raw.poses)
        {
            pts.push_back(p.position());
        }

        if (cfg.snap_endpoints)
        {
            pts = snap_endpoints(std::move(pts), start.position(), goal.position(), is_free);
        }
        if (cfg.shortcut && pts.size() > 2)
        {
            if (planner.cost_weight > 0.0)
            {
                log::warning("path_smoothing: shortcut skipped because cost_weight > 0 "
                             "(straightening would pull the path back towards obstacles)");
            }
            else
            {
                pts = shortcut(pts, is_free);
            }
        }
        return to_path(pts, start.yaw());
    }
}
