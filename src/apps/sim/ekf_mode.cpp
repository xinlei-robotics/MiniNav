module;

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

module mininav.apps.sim;

import mininav.core.types;
import mininav.core.command_source;
import mininav.core.trajectory;
import mininav.core.csv_format;
import mininav.core.kinematics;
import mininav.core.random;
import mininav.core.logger;
import mininav.core.robot_description;
import mininav.simulation.noise_presets;
import mininav.simulation.plant;
import mininav.localization.wheel_odometry;
import mininav.localization.ekf_state;
import mininav.localization.ekf;
import mininav.localization.ekf_pipeline;
import mininav.localization.encoder_observation;
import mininav.viz.rerun_sink;
import mininav.viz.sim_state_log;

// ===========================================================================
// sim ekf —— V2 的定位仿真。
//
//   EkfRunConfig   —— EkfOptions 解析 / 求值后的运行配置(preset + seed + 旋钮)。
//   EkfSimulation  —— 指令剖面 + Plant + wheel-odometry 基线 + EkfPipeline,
//                     step(t) 推进一帧并返回该帧的 SimState 快照。
//   run_ekf        —— 只做 I/O / 可视化 / CSV 落盘,不含仿真逻辑。
// ===========================================================================

namespace mininav::apps
{
    namespace
    {
        namespace fs = std::filesystem;
        using simulation::NoisePreset;

        // ---- 仿真时间 ----------------------------------------------------------
        constexpr double kDt = kSimDt;
        constexpr double kTotalTime = 20.0;
        constexpr std::size_t kStepCount =
            static_cast<std::size_t>(kTotalTime / kDt) + 1;

        // ---- Rerun 实体路径 ----------------------------------------------------
        constexpr std::string_view kRobotEntityPath = "/world/robot";
        // cmd_traj(假设完美执行 cmd 的参考轨迹)与 bias 对照曲线不属于 SimState,
        // 由主循环单独 log。trail 路径遵循 sim_state_log 约定: /world/trails/{name}。
        constexpr std::string_view kCmdTrajEntity = "/world/robot/cmd_traj";
        constexpr std::string_view kCmdTrajTrail = "/world/trails/cmd_traj";
        constexpr std::string_view kEkfBiasOmega = "/plots/bias_omega/ekf";
        constexpr std::string_view kTrueBiasOmega = "/plots/bias_omega/truth";

        [[nodiscard]] ekf::Integrator integrator_from_name(const std::string_view name) noexcept
        {
            return name == "euler" ? ekf::Integrator::Euler : ekf::Integrator::Rk4;
        }

        // ===================================================================
        // EkfRunConfig: EkfOptions 解析 / 求值后的运行配置。
        // ===================================================================
        struct EkfRunConfig
        {
            const NoisePreset& preset;
            std::uint64_t seed;
            ekf::Integrator integrator;
            std::string_view integrator_name;
            double q_scale;
            double r_scale;
            bool bias_on;

            [[nodiscard]] static EkfRunConfig from(const EkfOptions& opts)
            {
                return EkfRunConfig{
                    .preset = simulation::noise_preset(opts.preset_name),
                    .seed = resolve_seed(opts.seed),
                    .integrator = integrator_from_name(opts.integrator_name),
                    .integrator_name = opts.integrator_name,
                    .q_scale = opts.q_scale,
                    .r_scale = opts.r_scale,
                    .bias_on = !opts.no_bias,
                };
            }
        };

        // ===================================================================
        // EkfSimulation: 每次 step 推进一帧并返回该帧的 SimState 快照(其中
        // truth / odom / ekf 均为推进前的 prior belief, NIS 为本帧 update 的产物)。
        // RNG 只在 Plant 内消耗(actuator → encoder → imu)。
        // ===================================================================
        class EkfSimulation
        {
        public:
            EkfSimulation(const RobotDescription& robot, const EkfRunConfig& cfg)
                : plant_{robot, cfg.preset, RngFactory{cfg.seed}, Pose2D{0.0, 0.0, 0.0}},
                  odometry_{
                      WheelOdometryParams{
                          .wheel_base = robot.wheel_base,
                          .distance_per_tick = robot.distance_per_tick(),
                      },
                      Pose2D{0.0, 0.0, 0.0}
                  },
                  pipeline_{
                      make_pipeline_config(robot, cfg.preset,
                                           EkfTuning{
                                               .q_scale = cfg.q_scale,
                                               .r_scale = cfg.r_scale,
                                               .bias_on = cfg.bias_on,
                                               .integrator = cfg.integrator,
                                           }),
                      Pose2D{0.0, 0.0, 0.0}
                  }
            {
            }

            [[nodiscard]] SimState step(const double t)
            {
                // 当前时刻 t 的指令 → 被控对象推进一步(真值在测量之后推进)。
                const Twist2D cmd = command_source_.command_at(t);
                const Pose2D truth_prior = plant_.truth();
                const simulation::SensorReadings readings = plant_.step(cmd, kDt);

                // 快照本帧开始时的 belief(prior): truth / odom / ekf 均为推进前的值。
                SimState state{
                    .t = t,
                    .cmd = cmd,
                    .true_velocity = readings.true_velocity,
                    .truth_pose = truth_prior,
                    .enc_dticks = readings.dticks,
                    .imu_omega = readings.imu_omega,
                    .odom_pose = odometry_.current_estimate(),
                    .ekf_mean = pipeline_.filter().mu(),
                    .ekf_cov = pipeline_.filter().Sigma(),
                };

                // 估计器只看传感器读数:wheel-odometry 基线 + EKF 三阶段。
                odometry_.update(readings.dticks, kDt);
                const ekf::EkfNis nis = pipeline_.step(readings.dticks, readings.imu_omega, kDt);
                state.nis_encoder = nis.encoder;
                state.nis_imu = nis.imu;
                return state;
            }

            // gyro bias 真值, 供 viz 的 bias 估计曲线对照(filter 估计应趋近它)。
            [[nodiscard]] double true_bias_omega() const noexcept { return plant_.true_bias_omega(); }

        private:
            StagedCommandSource command_source_{};
            simulation::Plant plant_;
            WheelOdometry odometry_;
            ekf::EkfPipeline pipeline_;
        };

        // ===================================================================
        // write_csv_with_metadata: 带可复现实验元数据的 CSV 落盘。
        // 头部注释嵌入 seed / preset / dt / duration / 旋钮, 供 Python 脚本精确重放。
        // ===================================================================
        void write_csv_with_metadata(const Trajectory<SimState>& traj, const fs::path& path,
                                     const EkfRunConfig& cfg)
        {
            if (path.has_parent_path())
            {
                fs::create_directories(path.parent_path());
            }
            std::ofstream out{path};
            if (!out)
            {
                throw std::runtime_error{"Failed to open CSV for writing: " + path.string()};
            }

            const auto now = std::chrono::system_clock::now();
            const auto now_t = std::chrono::system_clock::to_time_t(now);

            out << "# MiniNav trajectory\n";
            out << "# seed = " << cfg.seed << '\n';
            out << "# preset = " << cfg.preset.name << '\n';
            out << "# dt = " << kDt << '\n';
            out << "# duration = " << kTotalTime << '\n';
            out << "# mode = encoder+imu\n";
            out << "# integrator = " << cfg.integrator_name << '\n';
            out << "# q_scale = " << cfg.q_scale << '\n';
            out << "# r_scale = " << cfg.r_scale << '\n';
            out << "# bias = " << (cfg.bias_on ? "on" : "off") << '\n';
            out << "# generated_at = "
                << std::put_time(std::gmtime(&now_t), "%Y-%m-%dT%H:%M:%SZ") << '\n';

            out << csv_header(SimState{}) << '\n';
            for (const auto& record : traj.records())
            {
                out << csv_row(record) << '\n';
            }
        }

        [[nodiscard]] std::string make_banner(const EkfRunConfig& cfg)
        {
            std::ostringstream banner;
            banner << "MiniNav: preset = " << cfg.preset.name
                << ", seed = " << cfg.seed << ", mode = encoder+imu"
                << ", integrator = " << cfg.integrator_name
                << ", q_scale = " << cfg.q_scale << ", r_scale = " << cfg.r_scale
                << ", bias = " << (cfg.bias_on ? "on" : "off");
            return banner.str();
        }
    }

    void run_ekf(const EkfOptions& opts)
    {
        using ekf::kBiasOmega;

        const RobotDescription robot = load_robot_description(opts.robot_path);
        const EkfRunConfig cfg = EkfRunConfig::from(opts);
        log::info(make_banner(cfg));

        const fs::path csv_path = output_csv_path(opts.output, "traj.csv");

        EkfSimulation simulation{robot, cfg};
        std::optional<RerunSink> sink = make_sink(opts.output);
        if (sink.has_value())
        {
            register_statics(*sink, kRobotEntityPath);
        }

        Trajectory<SimState> trajectory;
        trajectory.reserve(kStepCount);

        log::info("MiniNav simulation started.");

        // cmd_traj 是纯 viz 的"完美执行 cmd 参考轨迹", 不进入估计管线, 故由主循环维护。
        Pose2D cmd_pose{0.0, 0.0, 0.0};

        for (std::size_t i = 0; i < kStepCount; ++i)
        {
            const double t = static_cast<double>(i) * kDt;
            const SimState state = simulation.step(t);

            if (sink.has_value())
            {
                sink->set_time(t);
                log_to_rerun(*sink, state, kRobotEntityPath);

                // gyro bias 估计曲线: filter 估计 vs 仿真真值(state augmentation 演示)。
                sink->log_scalar(kEkfBiasOmega, state.ekf_mean(kBiasOmega));
                sink->log_scalar(kTrueBiasOmega, simulation.true_bias_omega());

                // cmd_traj 参考轨迹(prior, 与其余通道同相)。
                sink->log_pose(kCmdTrajEntity, cmd_pose);
                sink->log_trail_point(kCmdTrajTrail, cmd_pose.x(), cmd_pose.y());
            }

            trajectory.append(state);
            cmd_pose = differential_drive_step(cmd_pose, state.cmd, kDt);
        }

        write_csv_with_metadata(trajectory, csv_path, cfg);
        log::info("Trajectory CSV written to " + csv_path.string());
        log::info("MiniNav simulation ended.");
    }
}
