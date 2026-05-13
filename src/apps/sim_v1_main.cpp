import mininav.core.types;
import mininav.core.command_source;
import mininav.core.trajectory;
import mininav.core.csv_writer;
import mininav.core.csv_format;
import mininav.core.kinematics;
import mininav.core.random;
import mininav.core.logger;
import mininav.sensors.actuator_model;
import mininav.sensors.wheel_encoder;
import mininav.localization.wheel_odometry;
import mininav.viz.rerun_sink;
import mininav.viz.sim_state_log;

#include <CLI/CLI.hpp>

#include <chrono>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <numbers>
#include <optional>
#include <random>
#include <string>
#include <string_view>

namespace
{
    constexpr double kSimulationDt = 0.01;
    constexpr double kSimulationTotalTime = 20.0;
    constexpr std::string_view kRobotEntityPath = "/world/robot";
    constexpr std::string_view kApplicationId = "mininav_v1";

    // cmd_traj 是 main 自己维护的"参考轨迹" entity, 不属于 SimStateV1.
    // 因此不能由 log_to_rerun(SimStateV1) 统一处理, 要在主循环里显式 log.
    // trail 路径按 sim_state_log 的命名约定: /world/trails/{name}.
    constexpr std::string_view kCmdTrajEntity = "/world/robot/cmd_traj";
    constexpr std::string_view kCmdTrajTrail = "/world/trails/cmd_traj";

    // -------------------------------------------------------------------------
    // V1 噪声 preset : 三档可选,工程合理值。
    // -------------------------------------------------------------------------
    struct V1Preset
    {
        std::string_view name;
        double alpha1, alpha2, alpha3, alpha4; // Velocity Motion Model
        double slip_sigma; // 编码器打滑标准差
    };

    constexpr V1Preset kPresetLowNoise{
        "low-noise",
        0.01, 0.005, 0.005, 0.01,
        0.005,
    };
    constexpr V1Preset kPresetDefault{
        "default",
        0.05, 0.02, 0.02, 0.05,
        0.02,
    };
    constexpr V1Preset kPresetHighNoise{
        "high-noise",
        0.15, 0.08, 0.08, 0.15,
        0.05,
    };

    [[nodiscard]] const V1Preset* find_preset(std::string_view name) noexcept
    {
        if (name == kPresetLowNoise.name) return &kPresetLowNoise;
        if (name == kPresetDefault.name) return &kPresetDefault;
        if (name == kPresetHighNoise.name) return &kPresetHighNoise;
        return nullptr;
    }

    // 物理常数
    constexpr double kWheelRadius = 0.032;
    constexpr double kWheelBase = 0.150;
    constexpr std::int64_t kTicksPerRev = 1024;

    // -------------------------------------------------------------------------
    // CliOptions / parse_cli
    // -------------------------------------------------------------------------
    struct CliOptions
    {
        std::optional<std::filesystem::path> rrd_path;
        bool disable_viz{false};
        std::string preset_name{"default"};
        std::optional<std::uint64_t> seed;
    };

    [[nodiscard]] CliOptions parse_cli(int argc, char** argv)
    {
        CLI::App app{
            "MiniNav V1 simulation with actuator + encoder noise and "
            "wheel-odometry estimation."
        };

        // CLI11 直接绑定到 std::optional, 命令行未传时 has_value() 为 false。
        std::optional<std::uint64_t> seed_opt;
        std::optional<std::string> rrd_path_str;
        std::string preset_name{"default"};
        bool disable_viz{false};

        app.add_option("--seed", seed_opt,
                       "Master RNG seed; if omitted, seeded from std::random_device.");

        app.add_option("--preset", preset_name,
                       "Noise preset.")
           ->capture_default_str()
           ->check(CLI::IsMember({"low-noise", "default", "high-noise"}));

        auto* rrd_opt = app.add_option("--rrd", rrd_path_str,
                                       "Save Rerun recording to the given .rrd path.");

        auto* noviz_opt = app.add_flag("--no-viz", disable_viz,
                                       "Disable Rerun output entirely; CSV-only run (for CI / regression).");

        // 互斥: --rrd 与 --no-viz 不能共存
        rrd_opt->excludes(noviz_opt);

        try
        {
            app.parse(argc, argv);
        }
        catch (const CLI::ParseError& e)
        {
            // app.exit 会自动: --help → stdout + exit 0; 错误 → stderr + exit !=0
            std::exit(app.exit(e));
        }

        CliOptions opts{};
        opts.seed = seed_opt;
        opts.preset_name = preset_name;
        opts.disable_viz = disable_viz;
        if (rrd_path_str.has_value())
        {
            opts.rrd_path = std::filesystem::path{*rrd_path_str};
        }
        return opts;
    }

    // -------------------------------------------------------------------------
    // resolve_seed
    // -------------------------------------------------------------------------
    [[nodiscard]] std::uint64_t resolve_seed(std::optional<std::uint64_t> requested)
    {
        if (requested.has_value())
        {
            return *requested;
        }
        std::random_device rd;
        const auto hi = static_cast<std::uint64_t>(rd());
        const auto lo = static_cast<std::uint64_t>(rd());
        return (hi << 32) ^ lo;
    }

    // -------------------------------------------------------------------------
    // write_csv_with_metadata
    // -------------------------------------------------------------------------
    void write_csv_with_metadata(
        const mininav::Trajectory<mininav::SimStateV1>& traj,
        const std::filesystem::path& path,
        std::uint64_t master_seed,
        std::string_view preset_name)
    {
        if (path.has_parent_path())
        {
            std::filesystem::create_directories(path.parent_path());
        }
        std::ofstream out{path};
        if (!out)
        {
            throw std::runtime_error{
                "Failed to open CSV for writing: " + path.string()
            };
        }

        const auto now = std::chrono::system_clock::now();
        const auto now_t = std::chrono::system_clock::to_time_t(now);

        out << "# MiniNav V1 trajectory\n";
        out << "# seed = " << master_seed << '\n';
        out << "# preset = " << preset_name << '\n';
        out << "# dt = " << kSimulationDt << '\n';
        out << "# duration = " << kSimulationTotalTime << '\n';
        out << "# generated_at = "
            << std::put_time(std::gmtime(&now_t), "%Y-%m-%dT%H:%M:%SZ")
            << '\n';

        out << mininav::csv_header(mininav::SimStateV1{}) << '\n';

        for (const auto& s : traj.records())
        {
            out << mininav::csv_row(s) << '\n';
        }
    }
}

int main(int argc, char** argv)
{
    using namespace mininav;
    namespace fs = std::filesystem;

    try
    {
        const auto opts = parse_cli(argc, argv);
        const auto* preset_ptr = find_preset(opts.preset_name);
        const V1Preset preset = *preset_ptr;

        const std::uint64_t master_seed = resolve_seed(opts.seed);

        {
            std::ostringstream banner;
            banner << "MiniNav V1: preset = " << preset.name
                << ", seed = " << master_seed;
            log::info(banner.str());
        }

        // ---- RNG 工厂 ---------------------------------------------------
        const RngFactory rng_factory{master_seed};

        // ---- 仿真组件 ---------------------------------------------------
        const StagedCommandSource command_source;

        ActuatorModel actuator{
            ActuatorNoiseParams{
                .alpha1 = preset.alpha1, .alpha2 = preset.alpha2,
                .alpha3 = preset.alpha3, .alpha4 = preset.alpha4,
            },
            rng_factory.make_engine("actuator")
        };

        WheelEncoderModel encoder{
            WheelEncoderParams{
                .wheel_radius = kWheelRadius,
                .wheel_base = kWheelBase,
                .ticks_per_rev = kTicksPerRev,
                .slip_sigma = preset.slip_sigma,
            },
            rng_factory.make_engine("encoder_slip_left"),
            rng_factory.make_engine("encoder_slip_right")
        };

        WheelOdometry odometry{
            WheelOdometryParams{
                .wheel_base = kWheelBase,
                .distance_per_tick = 2.0 * std::numbers::pi * kWheelRadius
                / static_cast<double>(kTicksPerRev),
            },
            Pose2D{0.0, 0.0, 0.0}
        };

        // ---- Trajectory + CSV 输出准备 ----------------------------------
        Trajectory<SimStateV1> trajectory;
        constexpr auto step_count =
            static_cast<std::size_t>(kSimulationTotalTime / kSimulationDt) + 1;
        trajectory.reserve(step_count);

        const fs::path csv_path =
            fs::path{PROJECT_ROOT_DIR} / "data" / "traj_v1.csv";

        // ---- Rerun sink (三模式: spawn / save / off) --------------------
        std::optional<RerunSink> sink;
        if (!opts.disable_viz)
        {
            if (opts.rrd_path.has_value())
            {
                sink.emplace(kApplicationId, *opts.rrd_path);
                log::info("Rerun: writing to " + opts.rrd_path->string());
            }
            else
            {
                sink.emplace(kApplicationId);
                log::info("Rerun: Viewer spawned (gRPC).");
            }

            // 一次性注册静态可视化元素 (世界轴 + truth/odom 本体轴).
            // 通过 log_static 写入, 所有时间点可见, 不需要每帧重复 log.
            register_v1_statics(*sink, kRobotEntityPath);
        }
        else
        {
            log::info("Rerun: disabled by --no-viz.");
        }

        log::info("MiniNav V1 simulation started.");

        // ---- 主循环 ------------------------------------------------------
        Pose2D truth_pose{0.0, 0.0, 0.0};
        Pose2D cmd_pose{0.0, 0.0, 0.0};
        Pose2D odom_pose{0.0, 0.0, 0.0};

        for (std::size_t i = 0; i < step_count; ++i)
        {
            const double t = static_cast<double>(i) * kSimulationDt;
            const Twist2D cmd = command_source.command_at(t);

            // 当前时刻 t 的传感测量:cmd → 执行噪声 → 编码器
            const Twist2D true_velocity = actuator.apply(cmd);
            const EncoderTicks dticks = encoder.measure(true_velocity, kSimulationDt);

            const SimStateV1 state{
                .t = t,
                .cmd = cmd,
                .true_velocity = true_velocity,
                .truth_pose = truth_pose,
                .enc_dticks = dticks,
                .odom_pose = odom_pose,
            };
            trajectory.append(state);

            if (sink.has_value())
            {
                sink->set_time(t);
                log_to_rerun(*sink, state, kRobotEntityPath);

                // cmd_traj 通道(假设完美执行 cmd 时的参考轨迹)
                sink->log_pose(kCmdTrajEntity, cmd_pose);
                sink->log_trail_point(kCmdTrajTrail, cmd_pose.x(), cmd_pose.y());
            }

            truth_pose = differential_drive_step(truth_pose, true_velocity, kSimulationDt);
            cmd_pose = differential_drive_step(cmd_pose, cmd, kSimulationDt);
            odom_pose = odometry.update(dticks, kSimulationDt);
        }

        write_csv_with_metadata(trajectory, csv_path, master_seed, preset.name);
        log::info("Trajectory CSV written to " + csv_path.string());
        log::info("MiniNav V1 simulation ended.");
    }
    catch (const std::exception& ex)
    {
        log::error(ex.what());
        return 1;
    }
    return 0;
}