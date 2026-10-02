module;

#include <Eigen/Core>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

export module mininav.apps.sim;

import mininav.viz.rerun_sink;

// ===========================================================================
// MiniNav simulation app。
//
// 一个 sim 二进制,按子命令分派(main.cpp 负责 CLI11 与分派):
//   sim ekf    V2 室内差分驱动机器人的定位仿真(ekf_mode.cpp):
//              写死的指令剖面 → Plant(执行噪声 → encoder + IMU → 真值)
//              → wheel-odometry 基线 + EkfPipeline → traj.csv + Rerun。
//   sim plan   V3 一次性、无 RNG 的全局规划(plan_mode.cpp):
//              load map → inflate → A* → path.csv + Rerun 规划视图。
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

    void run_ekf(const EkfOptions& opts);
    void run_plan(const PlanOptions& opts);
}

// ---------------------------------------------------------------------------
// 模块内部(不导出):各模式共享的工具,实现见 common.cpp。
// ---------------------------------------------------------------------------
namespace mininav::apps
{
    inline constexpr std::string_view kApplicationId = "mininav";

    // 未给 --out 时的 CSV 路径:<仓库根>/data/<file_name>。
    [[nodiscard]] std::filesystem::path output_csv_path(const OutputOptions& output,
                                                        std::string_view default_file_name);

    // 按输出选项装配 Rerun sink:--no-viz → 不创建;--rrd → 存文件;否则 spawn Viewer。
    [[nodiscard]] std::optional<RerunSink> make_sink(const OutputOptions& output);

    // 解析 "x,y" world 坐标(米)。格式错误抛 std::runtime_error。
    [[nodiscard]] Eigen::Vector2d parse_xy(const std::string& s);
}
