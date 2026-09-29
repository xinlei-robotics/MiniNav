module;

#include <Eigen/Core>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

module mininav.apps.sim;

import mininav.core.types;
import mininav.core.path;
import mininav.core.logger;
import mininav.viz.rerun_sink;
import mininav.viz.plan_log;
import mininav.planning.grid_types;
import mininav.planning.occupancy_grid;
import mininav.planning.map_io;
import mininav.planning.inflation;
import mininav.planning.planner_config;
import mininav.planning.astar;

// ===========================================================================
// sim plan —— V3 的一次性全局规划。
//
// 一次性、**无 RNG** 的全局规划:load map -> inflate -> A* -> path.csv + viz。
//
// 起点 start 是一个普通 Pose2D —— 在完整系统里它就是 EKF 的估计位姿(V4 的
// sim nav 在闭环里注入)。但规划入口刻意保持无 RNG / 逐字节确定(docs/v3_summary.md
// §3.8:同 map+start+goal+config → path.csv 逐字节一致),所以这里不跑消耗 RNG 的 EKF。
// ===========================================================================

namespace mininav::apps
{
    namespace
    {
        namespace fs = std::filesystem;

        [[nodiscard]] planning::Heuristic heuristic_from_name(const std::string& s)
        {
            using planning::Heuristic;
            if (s == "manhattan") { return Heuristic::Manhattan; }
            if (s == "euclidean") { return Heuristic::Euclidean; }
            return Heuristic::Octile; // CLI 已用 IsMember 校验
        }

        [[nodiscard]] std::string heuristic_name_of(const planning::Heuristic h)
        {
            using planning::Heuristic;
            switch (h)
            {
            case Heuristic::Manhattan: return "manhattan";
            case Heuristic::Euclidean: return "euclidean";
            case Heuristic::Octile: return "octile";
            }
            return "euclidean";
        }

        // 栅格中心的 world 坐标(默认起点)。origin 是左下角,故 +半幅宽高。
        [[nodiscard]] Eigen::Vector2d grid_center(const planning::OccupancyGrid& g)
        {
            return g.origin() + Eigen::Vector2d{
                       g.width() * g.resolution() * 0.5,
                       g.height() * g.resolution() * 0.5
                   };
        }

        // 把原图占据 cell 与"膨胀新增"cell 分开收集,供 viz 的安全裕度展示。
        [[nodiscard]] PlanScene build_plan_scene(const planning::OccupancyGrid& grid,
                                                 const planning::OccupancyGrid& inflated,
                                                 const planning::PlanResult& result,
                                                 const Pose2D& start, const Pose2D& goal)
        {
            using namespace planning;
            PlanScene scene;
            scene.start = start;
            scene.goal = goal;
            scene.cell_radius = static_cast<float>(grid.resolution() * 0.5);

            for (int y = 0; y < grid.height(); ++y)
            {
                for (int x = 0; x < grid.width(); ++x)
                {
                    const GridCoord c{x, y};
                    const Eigen::Vector2d w = grid.grid_to_world(c);
                    if (grid.at(c) == kOccupied)
                    {
                        scene.obstacle_cells.push_back(w);
                    }
                    else if (inflated.at(c) == kOccupied)
                    {
                        scene.inflated_cells.push_back(w); // 膨胀新增的安全裕度
                    }
                }
            }

            scene.path.reserve(result.path.poses.size());
            for (const Pose2D& p : result.path.poses)
            {
                scene.path.emplace_back(p.x(), p.y());
            }
            return scene;
        }

        // path.csv:逐字节确定(无时间戳、无 plan_time_ms)。header 嵌入 map/start/goal/
        // heuristic/connectivity/inflation_radius/success/expanded_nodes/path_length_m,
        // 一次规划自包含、可复现、可比对(docs/v3_summary.md §4.3)。
        void write_path_csv(const fs::path& path, const std::string& map_path,
                            const planning::PlannerConfig& cfg, const Pose2D& start,
                            const Pose2D& goal, const planning::PlanResult& result)
        {
            if (path.has_parent_path())
            {
                fs::create_directories(path.parent_path());
            }
            std::ofstream out{path};
            if (!out)
            {
                throw std::runtime_error{"Failed to open path CSV for writing: " + path.string()};
            }

            out << "# MiniNav global plan\n";
            out << "# map = " << map_path << '\n';
            out << "# start = " << start.x() << ',' << start.y() << '\n';
            out << "# goal = " << goal.x() << ',' << goal.y() << '\n';
            out << "# heuristic = " << heuristic_name_of(cfg.heuristic) << '\n';
            out << "# connectivity = " << static_cast<int>(cfg.connectivity) << '\n';
            out << "# inflation_radius = " << cfg.inflation_radius << '\n';
            out << "# success = " << (result.success ? 1 : 0) << '\n';
            out << "# expanded_nodes = " << result.expanded_nodes << '\n';
            out << "# path_length_m = " << result.path.length() << '\n';
            // 注:plan_time_ms 是非确定量,刻意不入 CSV(保 path.csv 逐字节一致)。

            out << "idx,x,y,yaw\n";
            for (std::size_t i = 0; i < result.path.poses.size(); ++i)
            {
                const Pose2D& p = result.path.poses[i];
                out << i << ',' << p.x() << ',' << p.y() << ',' << p.yaw() << '\n';
            }
        }
    }

    void run_plan(const PlanOptions& opts)
    {
        using namespace planning;

        // 1. 地图
        const OccupancyGrid grid = load_occupancy_grid(opts.map_path);

        // 2. 配置:planner.yaml 默认 + CLI 逐项覆盖
        PlannerConfig cfg = opts.config_path.has_value()
                                ? load_planner_config(*opts.config_path)
                                : PlannerConfig{};
        if (opts.heuristic_name.has_value())
        {
            cfg.heuristic = heuristic_from_name(*opts.heuristic_name);
        }
        if (opts.connectivity.has_value())
        {
            if (*opts.connectivity != 4 && *opts.connectivity != 8)
            {
                throw std::runtime_error{
                    "--connectivity must be 4 or 8, got " + std::to_string(*opts.connectivity)};
            }
            cfg.connectivity =
                (*opts.connectivity == 8) ? Connectivity::Eight : Connectivity::Four;
        }
        if (opts.inflation_radius.has_value())
        {
            cfg.inflation_radius = *opts.inflation_radius;
        }

        // 3. 起止点(start 缺省 = 栅格中心)
        const Eigen::Vector2d goal_xy = parse_xy(opts.goal_str);
        const Eigen::Vector2d start_xy = opts.start_str.has_value()
                                             ? parse_xy(*opts.start_str)
                                             : grid_center(grid);
        const Pose2D start{start_xy.x(), start_xy.y(), 0.0};
        const Pose2D goal{goal_xy.x(), goal_xy.y(), 0.0};

        // 4. 规划
        const AStarPlanner planner{grid, cfg};
        const PlanResult result = planner.plan(start, goal);

        std::ostringstream metrics;
        metrics << "plan: map=" << opts.map_path
            << " heuristic=" << heuristic_name_of(cfg.heuristic)
            << " conn=" << static_cast<int>(cfg.connectivity)
            << " inflation=" << cfg.inflation_radius
            << " success=" << (result.success ? 1 : 0)
            << " expanded_nodes=" << result.expanded_nodes
            << " plan_time_ms=" << result.plan_time_ms
            << " path_length_m=" << result.path.length();
        log::info(metrics.str());
        if (!result.success)
        {
            log::warning("No path found (goal unreachable or start/goal blocked).");
        }

        // 5. path.csv(确定性产出)
        const fs::path out_path = output_csv_path(opts.output, "path.csv");
        write_path_csv(out_path, opts.map_path, cfg, start, goal, result);
        log::info("Plan CSV written to " + out_path.string());

        // 6. 可视化(地图 + 膨胀 + 路径 + 起止)
        std::optional<RerunSink> sink = make_sink(opts.output);
        if (!sink.has_value())
        {
            return;
        }
        sink->log_axes_static("/world/origin", 0.5F);
        sink->set_time(0.0);
        const OccupancyGrid inflated = inflate(grid, cfg.inflation_radius);
        const PlanScene scene = build_plan_scene(grid, inflated, result, start, goal);
        log_plan(*sink, scene, "/world");
    }
}
