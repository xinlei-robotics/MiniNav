import mininav.core.types;
import mininav.core.robot_model;
import mininav.core.command_source;
import mininav.core.trajectory;
import mininav.core.csv_writer;
import mininav.core.logger;
import mininav.viz.rerun_sink;
import mininav.viz.sim_state_log;

#include <cstddef>
#include <exception>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace
{
    constexpr double kSimulationDt = 0.01;
    constexpr double kSimulationTotalTime = 20.0;
    constexpr std::string_view kRobotEntityPath = "/world/robot";
    constexpr std::string_view kApplicationId = "mininav_v0";

    // -------------------------------------------------------------------------
    // CLI:V0 阶段支持两个开关。
    //   --rrd <path>   保存到指定 .rrd 文件,不启动 Viewer
    //   --no-viz       完全关闭 Rerun 输出,只生成 CSV
    // -------------------------------------------------------------------------
    struct CliOptions
    {
        std::optional<std::filesystem::path> rrd_path;
        bool disable_viz{false};
    };

    [[nodiscard]] CliOptions parse_cli(const int argc, char** argv)
    {
        CliOptions opts{};
        for (int i = 1; i < argc; ++i)
        {
            if (const std::string_view arg{argv[i]}; arg == "--no-viz")
            {
                opts.disable_viz = true;
            }
            else if (arg == "--rrd")
            {
                if (i + 1 >= argc)
                {
                    throw std::runtime_error{"--rrd requires a path argument"};
                }
                opts.rrd_path = std::filesystem::path{argv[++i]};
            }
            else
            {
                throw std::runtime_error{
                    "Unknown argument: " + std::string{arg}
                    + "\nUsage: sim_v0 [--rrd <path.rrd>] [--no-viz]"
                };
            }
        }
        if (opts.disable_viz && opts.rrd_path.has_value())
        {
            throw std::runtime_error{
                "--rrd and --no-viz are mutually exclusive"
            };
        }
        return opts;
    }
}

int main(const int argc, char** argv)
{
    using namespace mininav;
    namespace fs = std::filesystem;

    try
    {
        const auto [rrd_path, disable_viz] = parse_cli(argc, argv);

        // ---- Trajectory + CSV 输出准备 -------------------------------------
        Trajectory<SimStateV0> trajectory;
        constexpr auto step_count =
            static_cast<std::size_t>(kSimulationTotalTime / kSimulationDt) + 1;
        trajectory.reserve(step_count);

        const fs::path csv_path = fs::path{PROJECT_ROOT_DIR} / "data" / "traj.csv";

        // ---- Rerun sink ---------------------------------------------------
        // 三种状态:
        //   --no-viz                → sink 是 nullopt,不 log 任何 Rerun 数据
        //   --rrd <path>            → sink 处于 Save 模式,写入指定文件
        //   (无参数)                → sink 处于 Spawn 模式,自动拉起 Viewer
        std::optional<RerunSink> sink;
        if (!disable_viz)
        {
            if (rrd_path.has_value())
            {
                sink.emplace(kApplicationId, *rrd_path);
                mininav::log::info("Rerun: writing to " + rrd_path->string());
            }
            else
            {
                sink.emplace(kApplicationId);
                mininav::log::info("Rerun: Viewer spawned (gRPC).");
            }
        }
        else
        {
            mininav::log::info("Rerun: disabled by --no-viz.");
        }

        const RobotModel model;
        const StagedCommandSource command_source;
        Pose2D pose{0.0, 0.0, 0.0};

        mininav::log::info("MiniNav V0 simulation started.");

        for (std::size_t i = 0; i < step_count; ++i)
        {
            const double t = static_cast<double>(i) * kSimulationDt;
            const auto cmd = command_source.command_at(t);

            const SimStateV0 state{t, pose, cmd};
            trajectory.append(state);

            if (sink.has_value())
            {
                sink->set_time(t);
                log_to_rerun(*sink, state, kRobotEntityPath);
            }

            pose = model.step(pose, cmd, kSimulationDt);
        }

        write_csv(trajectory, csv_path);
        mininav::log::info("Trajectory CSV written to " + csv_path.string());
        mininav::log::info("MiniNav V0 simulation ended.");
    }
    catch (const std::exception& ex)
    {
        mininav::log::error(ex.what());
        return 1;
    }
    return 0;
}
