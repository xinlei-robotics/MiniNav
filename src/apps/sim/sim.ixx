module;

#include <Eigen/Core>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

export module mininav.apps.sim;

import mininav.core.types;
import mininav.core.path;
import mininav.core.robot_description;
import mininav.simulation.noise_presets;
import mininav.localization.ekf;
import mininav.localization.ekf_pipeline;
import mininav.planning.grid_types;
import mininav.planning.occupancy_grid;
import mininav.viz.plan_log;
import mininav.viz.rerun_sink;

// ===========================================================================
// MiniNav simulation app。
//
// 一个 sim 二进制,按子命令分派(main.cpp 负责 CLI11 与分派):
//   sim ekf    V2 室内差分驱动机器人的定位仿真(ekf_mode.cpp):
//              写死的指令剖面 → Plant(执行噪声 → encoder + IMU → 真值)
//              → wheel-odometry 基线 + EkfPipeline → traj.csv + Rerun。
//   sim plan   V3 一次性、无 RNG 的全局规划(plan_mode.cpp):
//              load map → inflate → A* →(可选)路径后处理 → path.csv + Rerun 规划视图。
//   sim nav    V4 闭环导航(nav_mode.cpp):规划 → 路径后处理 → Pure Pursuit 跟踪
//              (控制器吃 EKF 估计或真值)→ Plant → EkfPipeline,直到到达 / 碰撞 /
//              卡住 / 超时 → nav.csv + Rerun 闭环视图。
//
// 导出的部分是 CLI 层需要的选项结构与入口;不导出的部分是各模式共享的内部
// 工具(common.cpp),只对本模块的实现单元可见。
// ===========================================================================

export namespace mininav::apps
{
    // 所有子命令共享的输出选项。
    struct OutputOptions
    {
        std::optional<std::string> out_path; // CSV 路径;缺省为 data/<子命令默认文件名>
        std::optional<std::string> rrd_path; // 存 .rrd 而不是 spawn Viewer
        bool disable_viz{false};             // 只写 CSV(CI / 回归)
    };

    // `sim ekf` 的选项。
    struct EkfOptions
    {
        std::optional<std::uint64_t> seed; // 缺省由 std::random_device 取,并打印出来
        std::string preset_name{"default"};
        std::string integrator_name{"rk4"};
        double q_scale{1.0};
        double r_scale{1.0};
        bool no_bias{false};
        std::string robot_path; // robot.yaml;main 里缺省为 config/robot.yaml
        OutputOptions output;
    };

    // `sim plan` 的选项。
    struct PlanOptions
    {
        std::string map_path;                   // map.yaml(ROS map_server 形式)
        std::optional<std::string> config_path; // planner.yaml;CLI 覆盖项优先
        std::optional<std::string> start_str;   // "x,y";缺省为栅格中心
        std::string goal_str;                   // "x,y"
        std::optional<std::string> heuristic_name;
        std::optional<int> connectivity;
        std::optional<double> inflation_radius;
        bool smooth{false}; // A* 之后做路径后处理(首尾替换 + 视线捷径)
        OutputOptions output;
    };

    // `sim nav` 的选项。
    struct NavOptions
    {
        std::optional<std::string> map_path;  // 规划时必需;--path 模式下可选(给了才判碰撞)
        std::optional<std::string> path_file; // 跟随给定路径(path.csv 格式),不规划(Nav2 FollowPath)
        std::optional<std::string> start_str; // "x,y[,yaw]";缺省为栅格中心(--path:路径首点)
        std::optional<std::string> goal_str;  // "x,y[,yaw]";给了 yaw 才检查到达朝向(--path:路径末点)
        std::optional<std::uint64_t> seed;
        std::string preset_name{"default"};
        std::string robot_path;               // main 里缺省为 config/robot.yaml
        std::string nav_path;                 // main 里缺省为 config/nav.yaml
        std::string controller_input{"ekf"};  // ekf | truth(oracle:真值进控制器)
        std::optional<double> max_time;       // 缺省 3 × 路径长度 / 期望速度 + 10 s
        OutputOptions output;
    };

    void run_ekf(const EkfOptions& opts);
    void run_plan(const PlanOptions& opts);
    void run_nav(const NavOptions& opts);
}

// ---------------------------------------------------------------------------
// 模块内部(不导出):各模式共享的工具,实现见 common.cpp。
// ---------------------------------------------------------------------------
namespace mininav::apps
{
    inline constexpr std::string_view kApplicationId = "mininav";

    // 仿真步长(ekf 与 nav 模式共用):100 Hz。
    inline constexpr double kSimDt = 0.01;

    // 未给 --out 时的 CSV 路径:<仓库根>/data/<file_name>。
    [[nodiscard]] std::filesystem::path output_csv_path(const OutputOptions& output,
                                                        std::string_view default_file_name);

    // 按输出选项装配 Rerun sink:--no-viz → 不创建;--rrd → 存文件;否则 spawn Viewer。
    [[nodiscard]] std::optional<RerunSink> make_sink(const OutputOptions& output);

    // 解析 "x,y" world 坐标(米)。格式错误抛 std::runtime_error。
    [[nodiscard]] Eigen::Vector2d parse_xy(const std::string& s);

    // 解析 "x,y" 或 "x,y,yaw"(米、弧度)。has_yaw 记录是否给了朝向。
    struct PoseArg
    {
        Pose2D pose;
        bool has_yaw{false};
    };
    [[nodiscard]] PoseArg parse_pose(const std::string& s);

    // 读 path.csv 形式的路径(`sim plan` 的输出):'#' 开头的行是元数据;表头必须含
    // x、y 列,yaw 列可选(缺省按线段方向补,末点沿用前一段)。至少两个 waypoint。
    [[nodiscard]] Path load_path_csv(const std::string& file);

    // 未指定 --seed 时从 std::random_device 取一个(调用方负责打印,保证可复现)。
    [[nodiscard]] std::uint64_t resolve_seed(std::optional<std::uint64_t> requested);

    // EKF 的 Q / R 由同一噪声档位推导。Q 旋钮:(α₁..₄, q_bias_omega) 整体乘 q_scale
    // (只缩放 EKF 的 Q,不动真实噪声);bias_on = false 强制 q_bias_omega = 0。
    // q_dv / q_dw 是闭环里未建模的加减速(ProcessNoiseParams 的说明),EKF 模式为 0。
    struct EkfTuning
    {
        double q_scale{1.0};
        double r_scale{1.0};
        bool bias_on{true};
        ekf::Integrator integrator{ekf::Integrator::Rk4};
        double q_dv{0.0};
        double q_dw{0.0};
    };
    [[nodiscard]] ekf::EkfPipelineConfig make_pipeline_config(const RobotDescription& robot,
                                                              const simulation::NoisePreset& preset,
                                                              const EkfTuning& tuning);

    [[nodiscard]] std::string heuristic_name_of(planning::Heuristic h);

    // 栅格中心的 world 坐标(默认起点)。origin 是左下角,故 +半幅宽高。
    [[nodiscard]] Eigen::Vector2d grid_center(const planning::OccupancyGrid& g);

    // 规划场景的纯几何快照:原图占据 cell 与"膨胀新增"cell 分开收集(安全裕度展示)。
    [[nodiscard]] PlanScene build_plan_scene(const planning::OccupancyGrid& grid,
                                             const planning::OccupancyGrid& inflated,
                                             const Path& path, const Pose2D& start, const Pose2D& goal);
}
