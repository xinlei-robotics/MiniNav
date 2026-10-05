module;

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

module mininav.apps.sim;

import mininav.core.types;
import mininav.core.path;
import mininav.core.random;
import mininav.core.logger;
import mininav.core.robot_description;
import mininav.core.csv_format;
import mininav.core.trajectory;
import mininav.simulation.noise_presets;
import mininav.simulation.plant;
import mininav.localization.wheel_odometry;
import mininav.localization.ekf;
import mininav.localization.ekf_pipeline;
import mininav.planning.grid_types;
import mininav.planning.occupancy_grid;
import mininav.planning.map_io;
import mininav.planning.inflation;
import mininav.planning.planner_config;
import mininav.planning.astar;
import mininav.planning.path_smoothing;
import mininav.control.controller;
import mininav.control.pure_pursuit;
import mininav.control.velocity_smoother;
import mininav.control.goal_checker;
import mininav.control.progress_checker;
import mininav.control.controller_config;
import mininav.viz.rerun_sink;
import mininav.viz.sim_state_log;
import mininav.viz.plan_log;
import mininav.viz.nav_log;

// ===========================================================================
// sim nav —— V4 闭环导航(docs/v4_plan.md §2.2、§5.2)。
//
//   1. 加载 robot.yaml、nav.yaml、地图;膨胀半径 < 车体外接圆半径时直接失败。
//   2. 以 EKF 的初始估计(= 已知起点)为起点规划,路径后处理,交给控制器。
//   3. 每个仿真步(100 Hz):
//        - 控制节拍(默认 20 Hz):先判到达 / 卡住,再算指令;两拍之间零阶保持;
//        - Plant 推进一步(执行器动力学 → 执行噪声 → 传感器 → 真值);
//        - EkfPipeline 用传感器读数推进;记录一行 NavStep;
//        - 真值车体(外接圆)碰到原始地图的占据 cell 即判碰撞。
//   4. nav.csv:头部元数据 + 每步一行;不含墙钟时间,同 seed 逐字节一致。
//
// --path FILE(对应 Nav2 的 FollowPath):跳过规划,直接跟随给定路径。用于在合成
// 路径(直线、单转角)上做控制实验;此时地图可选,给了才判碰撞 / 算净空。
// ===========================================================================

namespace mininav::apps
{
    namespace
    {
        namespace fs = std::filesystem;
        using control::PursuitRegime;

        constexpr std::string_view kRobotEntityPath = "/world/robot";
        constexpr std::string_view kWorldEntityPath = "/world";

        // 净空只在车体外 kClearanceCap 以内精确计算,更远的截断(画图与统计都够用)。
        constexpr double kClearanceCap = 0.5;
        // 没有地图(--path 且未给 --map)时净空无定义,CSV 里写 nan。
        constexpr double kNoClearance = std::numeric_limits<double>::quiet_NaN();

        // ---- nav.yaml ---------------------------------------------------------
        // 段名与 V5 的 Nav2 参数文件对应;各段交给所属库的解析函数(严格模式)。
        struct NavConfig
        {
            planning::PlannerConfig planner{};
            planning::PathSmoothingConfig smoothing{};
            control::ControllerConfig controller{};
            control::GoalCheckerConfig goal{};
            control::ProgressCheckerConfig progress{};
        };

        constexpr std::array<std::string_view, 5> kNavSections{
            "planner", "path_smoothing", "controller", "goal_checker", "progress_checker"};

        [[nodiscard]] NavConfig load_nav_config(const std::string& path)
        {
            std::ifstream in{path};
            if (!in)
            {
                throw std::runtime_error{"nav_config: cannot open '" + path + "'"};
            }
            std::ostringstream text;
            text << in.rdbuf();
            const YAML::Node root = YAML::Load(text.str());
            if (root.IsDefined() && !root.IsNull() && !root.IsMap())
            {
                throw std::runtime_error{"nav_config: expected a mapping of sections"};
            }
            if (root.IsMap())
            {
                for (const auto& entry : root)
                {
                    const std::string key = entry.first.as<std::string>();
                    if (std::ranges::find(kNavSections, key) == kNavSections.end())
                    {
                        throw std::runtime_error{"nav_config: unknown section '" + key +
                                                 "' (expected planner, path_smoothing, controller, "
                                                 "goal_checker, progress_checker)"};
                    }
                }
            }
            const auto section = [&root](const char* key)
            {
                return root.IsMap() && root[key] ? YAML::Dump(root[key]) : std::string{};
            };

            NavConfig cfg{};
            cfg.planner = planning::parse_planner_config(section("planner"));
            cfg.smoothing = planning::parse_path_smoothing_config(section("path_smoothing"));
            cfg.controller = control::parse_controller_config(section("controller"));
            cfg.goal = control::parse_goal_checker_config(section("goal_checker"));
            cfg.progress = control::parse_progress_checker_config(section("progress_checker"));
            return cfg;
        }

        // 控制周期(以仿真步计):控制频率必须整除 100 Hz 的仿真频率。
        [[nodiscard]] std::size_t control_period_steps(const double control_frequency)
        {
            const double ratio = 1.0 / (control_frequency * kSimDt);
            const double steps = std::round(ratio);
            if (steps < 1.0 || std::abs(ratio - steps) > 1e-9)
            {
                throw std::runtime_error{"nav: control_frequency must divide the 100 Hz simulation rate"};
            }
            return static_cast<std::size_t>(steps);
        }

        // ---- 几何评估 -----------------------------------------------------------
        // 车体外接圆(圆心 center、半径 radius)到最近占据 cell(方块)的净空;负值即碰撞。
        // 用原始地图(未膨胀):碰撞是真值车体与真实障碍的接触。越界按占据处理。
        [[nodiscard]] double footprint_clearance(const planning::OccupancyGrid& map,
                                                 const Eigen::Vector2d& center, const double radius)
        {
            using planning::GridCoord;
            const double half = 0.5 * map.resolution();
            const double reach = radius + kClearanceCap;
            const GridCoord lo = map.world_to_grid(Eigen::Vector2d{center.x() - reach, center.y() - reach});
            const GridCoord hi = map.world_to_grid(Eigen::Vector2d{center.x() + reach, center.y() + reach});

            double nearest = reach;
            for (int y = lo.y; y <= hi.y; ++y)
            {
                for (int x = lo.x; x <= hi.x; ++x)
                {
                    if (map.at(GridCoord{x, y}) != planning::kOccupied)
                    {
                        continue;
                    }
                    const Eigen::Vector2d c = map.grid_to_world(GridCoord{x, y});
                    const double dx = std::max(std::abs(center.x() - c.x()) - half, 0.0);
                    const double dy = std::max(std::abs(center.y() - c.y()) - half, 0.0);
                    nearest = std::min(nearest, std::hypot(dx, dy));
                }
            }
            return std::min(nearest - radius, kClearanceCap);
        }

        // 点到整条折线的距离(全局最近,不是控制器的窗口投影):误差分解里的 dist(·, P)。
        [[nodiscard]] double distance_to_path(const Path& path, const Eigen::Vector2d& p)
        {
            return path.empty() ? 0.0 : project_onto(path, p, 0, path.size()).distance;
        }

        // ---- 一次运行的结果 -----------------------------------------------------
        enum class NavStatus
        {
            Arrived,
            Collision,
            Stuck,
            Timeout,
            PlanFailed,
        };

        [[nodiscard]] std::string_view to_string(const NavStatus s) noexcept
        {
            switch (s)
            {
            case NavStatus::Arrived: return "arrived";
            case NavStatus::Collision: return "collision";
            case NavStatus::Stuck: return "stuck";
            case NavStatus::Timeout: return "timeout";
            case NavStatus::PlanFailed: return "plan_failed";
            }
            return "plan_failed";
        }

        struct NavOutcome
        {
            NavStatus status{NavStatus::PlanFailed};
            double end_time{0.0};
            Pose2D final_truth{};
            Pose2D final_input{};
            double min_clearance{kClearanceCap};
            double max_e_ctrl{0.0};
            double max_e_true{0.0};
        };

        // 运行中不变的上下文,写 nav.csv 头部用。
        struct NavRunInfo
        {
            const NavOptions& opts;
            const RobotDescription& robot;
            const NavConfig& nav;
            const PoseArg& start;
            const PoseArg& goal;
            std::uint64_t seed;
            double max_time;
            const Path& raw_path;
            const Path& path;
        };

        void write_nav_csv(const fs::path& out_path, const NavRunInfo& info, const NavOutcome& outcome,
                           const Trajectory<NavStep>& trajectory)
        {
            if (out_path.has_parent_path())
            {
                fs::create_directories(out_path.parent_path());
            }
            std::ofstream out{out_path};
            if (!out)
            {
                throw std::runtime_error{"Failed to open nav CSV for writing: " + out_path.string()};
            }

            const control::PurePursuitConfig& pp = info.nav.controller.pursuit;
            const Pose2D& g = info.goal.pose;
            out << "# MiniNav closed-loop navigation\n";
            out << "# map = " << info.opts.map_path.value_or("none") << '\n';
            if (info.opts.path_file.has_value())
            {
                out << "# path_file = " << *info.opts.path_file << '\n';
            }
            out << "# start = " << info.start.pose.x() << ',' << info.start.pose.y() << ','
                << info.start.pose.yaw() << '\n';
            out << "# goal = " << g.x() << ',' << g.y();
            if (info.goal.has_yaw)
            {
                out << ',' << g.yaw();
            }
            out << '\n';
            out << "# seed = " << info.seed << '\n';
            out << "# preset = " << info.opts.preset_name << '\n';
            out << "# controller_input = " << info.opts.controller_input << '\n';
            out << "# dt = " << kSimDt << '\n';
            out << "# control_frequency = " << info.nav.controller.control_frequency << '\n';
            out << "# max_time = " << info.max_time << '\n';
            // 机器人(robot.yaml):只写数值,不写文件路径(路径与检出位置有关)。
            out << "# wheel_radius = " << info.robot.wheel_radius << '\n';
            out << "# wheel_base = " << info.robot.wheel_base << '\n';
            out << "# footprint_radius = " << info.robot.footprint.circumscribed_radius() << '\n';
            out << "# max_linear_vel = " << info.robot.limits.max_linear_vel << '\n';
            out << "# max_angular_vel = " << info.robot.limits.max_angular_vel << '\n';
            out << "# actuator_time_constant = " << info.robot.actuator_time_constant << '\n';
            // 规划与控制的关键参数(nav.yaml)。
            out << "# inflation_radius = " << info.nav.planner.inflation_radius << '\n';
            out << "# heuristic = " << heuristic_name_of(info.nav.planner.heuristic) << '\n';
            out << "# connectivity = " << static_cast<int>(info.nav.planner.connectivity) << '\n';
            out << "# snap_endpoints = " << (info.nav.smoothing.snap_endpoints ? 1 : 0) << '\n';
            out << "# shortcut = " << (info.nav.smoothing.shortcut ? 1 : 0) << '\n';
            out << "# desired_linear_vel = " << pp.desired_linear_vel << '\n';
            out << "# velocity_scaled_lookahead = " << (pp.use_velocity_scaled_lookahead ? 1 : 0) << '\n';
            out << "# lookahead_time = " << pp.lookahead_time << '\n';
            out << "# min_lookahead = " << pp.min_lookahead << '\n';
            out << "# max_lookahead = " << pp.max_lookahead << '\n';
            out << "# lookahead_dist = " << pp.lookahead_dist << '\n';
            out << "# curvature_regulation = " << (pp.use_curvature_regulation ? 1 : 0) << '\n';
            out << "# curvature_lookahead_dist = " << pp.curvature_lookahead_dist << '\n';
            out << "# regulated_min_radius = " << pp.regulated_min_radius << '\n';
            out << "# approach_dist = " << pp.approach_dist << '\n';
            out << "# max_accel = " << info.nav.controller.smoother.max_accel << '\n';
            out << "# max_angular_accel = " << info.nav.controller.smoother.max_angular_accel << '\n';
            out << "# xy_tolerance = " << info.nav.goal.xy_tolerance << '\n';
            out << "# yaw_tolerance = " << info.nav.goal.yaw_tolerance << '\n';
            // 结果。
            out << "# status = " << to_string(outcome.status) << '\n';
            out << "# end_time_s = " << outcome.end_time << '\n';
            out << "# goal_error_true_m = " << (outcome.final_truth.position() - g.position()).norm() << '\n';
            out << "# goal_error_input_m = " << (outcome.final_input.position() - g.position()).norm() << '\n';
            out << "# raw_waypoints = " << info.raw_path.size() << '\n';
            out << "# raw_length_m = " << info.raw_path.length() << '\n';
            out << "# path_waypoints = " << info.path.size() << '\n';
            out << "# path_length_m = " << info.path.length() << '\n';
            out << "# min_clearance_m = " << outcome.min_clearance << '\n';
            out << "# max_e_ctrl_m = " << outcome.max_e_ctrl << '\n';
            out << "# max_e_true_m = " << outcome.max_e_true << '\n';

            out << csv_header(NavStep{}) << '\n';
            for (const NavStep& step : trajectory.records())
            {
                out << csv_row(step) << '\n';
            }
        }
    }

    void run_nav(const NavOptions& opts)
    {
        using namespace planning;
        using namespace control;

        // 1. 配置、机器人、地图(--path 模式:给定路径,地图可选)
        const bool follow = opts.path_file.has_value();
        if (!follow && (!opts.map_path.has_value() || !opts.goal_str.has_value()))
        {
            throw std::runtime_error{"nav: --map and --goal are required unless --path is given"};
        }
        const RobotDescription robot = load_robot_description(opts.robot_path);
        const NavConfig nav = load_nav_config(opts.nav_path);
        const double footprint = robot.footprint.circumscribed_radius();
        if (!follow && nav.planner.inflation_radius < footprint)
        {
            std::ostringstream msg;
            msg << "nav: planner.inflation_radius (" << nav.planner.inflation_radius
                << " m) is smaller than the robot's circumscribed radius (" << footprint
                << " m); the planned path could graze obstacles";
            throw std::runtime_error{msg.str()};
        }
        const bool oracle = opts.controller_input == "truth";
        const std::optional<OccupancyGrid> map =
            opts.map_path.has_value() ? std::optional{load_occupancy_grid(*opts.map_path)} : std::nullopt;
        const Path given = follow ? load_path_csv(*opts.path_file) : Path{};

        PoseArg start{};
        if (opts.start_str.has_value())
        {
            start = parse_pose(*opts.start_str);
        }
        else if (follow)
        {
            start = PoseArg{.pose = given.poses.front(), .has_yaw = true};
        }
        else
        {
            start = PoseArg{.pose = Pose2D{grid_center(*map), 0.0}, .has_yaw = false};
        }
        // --path:终点是路径末点,不检查朝向。
        const PoseArg goal = follow ? PoseArg{.pose = given.poses.back(), .has_yaw = false}
                                    : parse_pose(*opts.goal_str);
        const std::uint64_t seed = resolve_seed(opts.seed);
        const simulation::NoisePreset& preset = simulation::noise_preset(opts.preset_name);
        {
            std::ostringstream banner;
            banner << "MiniNav nav: map = " << opts.map_path.value_or("none");
            if (follow)
            {
                banner << ", path = " << *opts.path_file;
            }
            banner << ", preset = " << preset.name << ", seed = " << seed
                << ", controller_input = " << opts.controller_input;
            log::info(banner.str());
        }

        // 2. 被控对象与估计器:起点已知(相当于 RViz 的 2D Pose Estimate)。
        const double dt = kSimDt;
        simulation::Plant plant{robot, preset, RngFactory{seed}, start.pose,
                                simulation::actuator_dynamics_of(robot)};
        WheelOdometry odometry{
            WheelOdometryParams{.wheel_base = robot.wheel_base, .distance_per_tick = robot.distance_per_tick()},
            start.pose};
        const double a_dt = nav.controller.smoother.max_accel * dt;
        const double alpha_dt = nav.controller.smoother.max_angular_accel * dt;
        ekf::EkfPipeline estimator{
            make_pipeline_config(robot, preset, EkfTuning{.q_dv = a_dt * a_dt, .q_dw = alpha_dt * alpha_dt}),
            start.pose};

        // 3. 规划:从 EKF 的初始估计出发,在同一张膨胀 costmap 上做路径后处理。
        //    --path 模式直接用给定路径(raw 与最终路径相同)。
        std::optional<AStarPlanner> planner;
        Path raw_path;
        Path path;
        bool have_path = false;
        if (follow)
        {
            raw_path = given;
            path = given;
            have_path = true;
            std::ostringstream msg;
            msg << "nav: following " << path.size() << " waypoints / " << path.length() << " m";
            log::info(msg.str());
        }
        else
        {
            planner.emplace(*map, nav.planner);
            PlanResult plan = planner->plan(estimator.pose(), goal.pose);
            have_path = plan.success;
            if (plan.success)
            {
                path = smooth_path(plan.path, planner->costmap(), planner->config(), estimator.pose(),
                                   goal.pose, nav.smoothing);
                if (goal.has_yaw)
                {
                    path.poses.back().set_yaw(goal.pose.yaw());
                }
            }
            raw_path = std::move(plan.path);
            std::ostringstream msg;
            msg << "nav: plan success=" << (have_path ? 1 : 0) << " raw " << raw_path.size()
                << " waypoints / " << raw_path.length() << " m -> " << path.size() << " waypoints / "
                << path.length() << " m";
            log::info(msg.str());
        }

        // 4. 控制器、平滑器、到达与卡住判定。
        PurePursuitController controller{nav.controller.pursuit, robot.limits};
        controller.set_plan(path);
        VelocitySmoother smoother{nav.controller.smoother};
        SimpleGoalChecker goal_checker{nav.goal, goal.has_yaw};
        SimpleProgressChecker progress{nav.progress};
        const std::size_t control_every = control_period_steps(nav.controller.control_frequency);
        const double control_dt = static_cast<double>(control_every) * dt;
        const double max_time = opts.max_time.value_or(
            3.0 * path.length() / nav.controller.pursuit.desired_linear_vel + 10.0);

        std::optional<RerunSink> sink = make_sink(opts.output);
        if (sink.has_value())
        {
            register_statics(*sink, kRobotEntityPath);
            sink->set_time(0.0);
            NavScene scene{};
            if (map.has_value())
            {
                // --path 模式没有规划器,膨胀层只为显示而算。
                scene.plan = build_plan_scene(*map,
                                              planner.has_value() ? planner->costmap()
                                                                  : inflate(*map, nav.planner.inflation_radius),
                                              path, start.pose, goal.pose);
            }
            else
            {
                scene.plan.start = start.pose;
                scene.plan.goal = goal.pose;
                for (const Pose2D& p : path.poses)
                {
                    scene.plan.path.emplace_back(p.x(), p.y());
                }
            }
            for (const Pose2D& p : raw_path.poses)
            {
                scene.raw_path.emplace_back(p.x(), p.y());
            }
            log_nav_scene(*sink, scene, kWorldEntityPath);
        }

        // 5. 闭环
        Trajectory<NavStep> trajectory;
        NavOutcome outcome{};
        if (!map.has_value())
        {
            outcome.min_clearance = kNoClearance;
        }
        outcome.final_truth = plant.truth();
        outcome.final_input = estimator.pose();
        NavDiagnostics held{};   // 最近一个控制节拍的控制器输出(零阶保持)
        Twist2D cmd{};
        Twist2D true_velocity{}; // oracle 模式下作为控制器的速度输入

        if (have_path)
        {
            outcome.status = NavStatus::Timeout;
            for (std::size_t k = 0;; ++k)
            {
                const double t = static_cast<double>(k) * dt;
                const Pose2D truth = plant.truth();
                const Pose2D input = oracle ? truth : estimator.pose();
                const Twist2D input_velocity = oracle ? true_velocity : estimator.velocity();
                outcome.end_time = t;
                outcome.final_truth = truth;
                outcome.final_input = input;

                if (k % control_every == 0)
                {
                    if (goal_checker.is_goal_reached(input, goal.pose, input_velocity))
                    {
                        outcome.status = NavStatus::Arrived;
                        break;
                    }
                    if (!progress.check(input, t))
                    {
                        outcome.status = NavStatus::Stuck;
                        break;
                    }
                    cmd = smoother.smooth(
                        controller.compute_velocity_commands(input, input_velocity, &goal_checker), control_dt);
                    const PursuitDebug& dbg = controller.debug();
                    held.lookahead = dbg.lookahead_point;
                    held.lookahead_dist = dbg.lookahead_dist;
                    held.curvature = dbg.curvature;
                    held.regime = std::string{to_string(dbg.regime)};
                    held.regime_code = static_cast<std::int32_t>(dbg.regime);
                    held.arclength = dbg.arclength;
                    held.control_pose = input;
                }
                if (t >= max_time)
                {
                    break; // status 保持 Timeout
                }

                // 本步的 prior 快照(与 sim ekf 相同:truth / odom / ekf 为推进前的值)。
                NavStep step{};
                step.sim.t = t;
                step.sim.cmd = cmd;
                step.sim.truth_pose = truth;
                step.sim.odom_pose = odometry.current_estimate();
                step.sim.ekf_mean = estimator.filter().mu();
                step.sim.ekf_cov = estimator.filter().Sigma();

                const simulation::SensorReadings readings = plant.step(cmd, dt);
                step.sim.true_velocity = readings.true_velocity;
                step.sim.enc_dticks = readings.dticks;
                step.sim.imu_omega = readings.imu_omega;
                odometry.update(readings.dticks, dt);
                const ekf::EkfNis nis = estimator.step(readings.dticks, readings.imu_omega, dt);
                step.sim.nis_encoder = nis.encoder;
                step.sim.nis_imu = nis.imu;
                true_velocity = readings.true_velocity;

                step.nav = held;
                step.nav.actuator = plant.actuator_output();
                step.nav.e_ctrl = distance_to_path(path, input.position());
                step.nav.e_true = distance_to_path(path, truth.position());
                step.nav.e_est = (truth.position() - input.position()).norm();
                step.nav.clearance =
                    map.has_value() ? footprint_clearance(*map, truth.position(), footprint) : kNoClearance;

                if (map.has_value())
                {
                    outcome.min_clearance = std::min(outcome.min_clearance, step.nav.clearance);
                }
                outcome.max_e_ctrl = std::max(outcome.max_e_ctrl, step.nav.e_ctrl);
                outcome.max_e_true = std::max(outcome.max_e_true, step.nav.e_true);

                if (sink.has_value())
                {
                    sink->set_time(t);
                    log_to_rerun(*sink, step, kRobotEntityPath);
                }
                trajectory.append(step);

                if (step.nav.clearance < 0.0)
                {
                    outcome.status = NavStatus::Collision;
                    break;
                }
            }
        }

        // 6. 结果与 nav.csv
        {
            std::ostringstream msg;
            msg << "nav: status=" << to_string(outcome.status) << " t=" << outcome.end_time
                << "s goal_error_true=" << (outcome.final_truth.position() - goal.pose.position()).norm()
                << "m goal_error_input=" << (outcome.final_input.position() - goal.pose.position()).norm()
                << "m min_clearance=" << outcome.min_clearance << "m max_e_ctrl=" << outcome.max_e_ctrl
                << "m max_e_true=" << outcome.max_e_true << 'm';
            if (outcome.status == NavStatus::Arrived)
            {
                log::info(msg.str());
            }
            else
            {
                log::warning(msg.str());
            }
        }
        const fs::path out_path = output_csv_path(opts.output, "nav.csv");
        write_nav_csv(out_path,
                      NavRunInfo{
                          .opts = opts, .robot = robot, .nav = nav, .start = start, .goal = goal,
                          .seed = seed, .max_time = max_time, .raw_path = raw_path, .path = path,
                      },
                      outcome, trajectory);
        log::info("Nav CSV written to " + out_path.string());
    }
}
