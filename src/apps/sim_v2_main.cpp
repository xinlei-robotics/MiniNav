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
import mininav.sensors.imu_model;
import mininav.localization.wheel_odometry;
import mininav.localization.ekf_state;
import mininav.localization.ekf;
import mininav.localization.encoder_observation;
import mininav.viz.rerun_sink;
import mininav.viz.sim_state_log;

#include <CLI/CLI.hpp>
#include <Eigen/Dense>

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
    constexpr std::string_view kApplicationId = "mininav_v2";

    constexpr std::string_view kCmdTrajEntity = "/world/robot/cmd_traj";
    constexpr std::string_view kCmdTrajTrail = "/world/trails/cmd_traj";

    constexpr std::string_view kEkfEntity = "/world/robot/ekf";
    constexpr std::string_view kEkfTrail = "/world/trails/ekf";

    // gyro bias 学习曲线(Time Series): 估计值与仿真真值各一条。
    constexpr std::string_view kEkfBiasOmega = "/plots/bias_omega/ekf";
    constexpr std::string_view kTrueBiasOmega = "/plots/bias_omega/truth";

    struct V2Preset
    {
        std::string_view name;
        double alpha1, alpha2, alpha3, alpha4; // Velocity Motion Model
        double slip_sigma; // 编码器打滑标准差
        double sigma_imu; // IMU gyro 白噪声标准差 [rad/s]
        double imu_bias_init; // IMU gyro bias 真值(filter 待学习)[rad/s]
        double imu_bias_rw; // IMU gyro bias 每步随机游走标准差 [rad/s]; 0 => 常数 bias
        double q_bias_omega; // EKF 对 bias 的过程噪声(单步方差)(rad/s)²
    };

    // 注: imu_bias_rw 默认置 0;
    // 确认后把它调成小正数(如 5e-5)即可观察 filter 跟踪【漂移 bias】的能力。
    // EKF 侧的 q_bias_omega 保持小正值, 使 filter 即便面对常数 bias 也维持一点
    // 自适应余量(标准工业做法)。
    constexpr V2Preset kPresetLowNoise{
        "low-noise",
        0.01, 0.005, 0.005, 0.01,
        0.005,
        0.002,
        0.01,  // imu_bias_init
        0.0,   // imu_bias_rw
        1e-8,  // q_bias_omega
    };
    constexpr V2Preset kPresetDefault{
        "default",
        0.05, 0.02, 0.02, 0.05,
        0.02,
        0.005,
        0.02,  // imu_bias_init
        0.0,   // imu_bias_rw
        1e-8,  // q_bias_omega
    };
    constexpr V2Preset kPresetHighNoise{
        "high-noise",
        0.15, 0.08, 0.08, 0.15,
        0.05,
        0.015,
        0.03,  // imu_bias_init
        0.0,   // imu_bias_rw
        4e-8,  // q_bias_omega
    };

    [[nodiscard]] const V2Preset* find_preset(std::string_view name) noexcept
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
        std::string integrator_name{"rk4"};
        std::optional<std::filesystem::path> out_path;
        double q_scale{1.0};
        double r_scale{1.0};
    };

    // CLI 名 → EKF 过程模型积分器。rk4 是生产默认; euler 仅供归因实验
    // (analyze_v2_integrator.py)。
    [[nodiscard]] mininav::Integrator integrator_from_name(std::string_view name) noexcept
    {
        return name == "euler" ? mininav::Integrator::Euler : mininav::Integrator::Rk4;
    }

    [[nodiscard]] CliOptions parse_cli(int argc, char** argv)
    {
        CLI::App app{
            "MiniNav V2 simulation: actuator + encoder + IMU noise, wheel-odometry "
            "baseline, and a 6D EKF fusing encoder + gyro with online bias estimation "
        };

        std::optional<std::uint64_t> seed_opt;
        std::optional<std::string> rrd_path_str;
        std::string preset_name{"default"};
        bool disable_viz{false};
        std::string integrator_name{"rk4"};
        std::optional<std::string> out_path_str;
        double q_scale{1.0};
        double r_scale{1.0};

        app.add_option("--seed", seed_opt,
                       "Master RNG seed; if omitted, seeded from std::random_device.");

        app.add_option("--preset", preset_name,
                       "Noise preset.")
           ->capture_default_str()
           ->check(CLI::IsMember({"low-noise", "default", "high-noise"}));

        app.add_option("--integrator", integrator_name,
                       "EKF process-model integrator. rk4 = production; euler is kept only "
                       "for the RK4-vs-Euler attribution experiment.")
           ->capture_default_str()
           ->check(CLI::IsMember({"euler", "rk4"}));

        app.add_option("--out", out_path_str,
                       "Output CSV path (default: data/traj_v2.csv). Set this to keep the "
                       "euler / rk4 runs in separate files for analyze_v2_integrator.py.");

        app.add_option("--q-scale", q_scale,
                       "Multiplier on the EKF process noise Q (sensitivity analysis). "
                       "Default 1.0 keeps the physics-derived value; >1 trusts the motion "
                       "model less, <1 trusts it more. Does NOT touch the simulated truth.")
           ->capture_default_str()
           ->check(CLI::PositiveNumber);

        app.add_option("--r-scale", r_scale,
                       "Multiplier on the EKF measurement noise R (encoder + IMU). "
                       "Default 1.0 keeps the physics-derived value; >1 trusts the sensors "
                       "less. Does NOT touch the simulated measurements.")
           ->capture_default_str()
           ->check(CLI::PositiveNumber);

        auto* rrd_opt = app.add_option("--rrd", rrd_path_str,
                                       "Save Rerun recording to the given .rrd path.");

        auto* noviz_opt = app.add_flag("--no-viz", disable_viz,
                                       "Disable Rerun output entirely; CSV-only run (for CI / regression).");

        rrd_opt->excludes(noviz_opt);

        try
        {
            app.parse(argc, argv);
        }
        catch (const CLI::ParseError& e)
        {
            std::exit(app.exit(e));
        }

        CliOptions opts{};
        opts.seed = seed_opt;
        opts.preset_name = preset_name;
        opts.disable_viz = disable_viz;
        opts.integrator_name = integrator_name;
        opts.q_scale = q_scale;
        opts.r_scale = r_scale;
        if (rrd_path_str.has_value())
        {
            opts.rrd_path = std::filesystem::path{*rrd_path_str};
        }
        if (out_path_str.has_value())
        {
            opts.out_path = std::filesystem::path{*out_path_str};
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
        const mininav::Trajectory<mininav::SimStateV2>& traj,
        const std::filesystem::path& path,
        std::uint64_t master_seed,
        std::string_view preset_name,
        std::string_view integrator_name,
        double q_scale,
        double r_scale)
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

        out << "# MiniNav V2 trajectory\n";
        out << "# seed = " << master_seed << '\n';
        out << "# preset = " << preset_name << '\n';
        out << "# dt = " << kSimulationDt << '\n';
        out << "# duration = " << kSimulationTotalTime << '\n';
        out << "# mode = encoder+imu\n";
        out << "# integrator = " << integrator_name << '\n';
        out << "# q_scale = " << q_scale << '\n';
        out << "# r_scale = " << r_scale << '\n';
        out << "# generated_at = "
            << std::put_time(std::gmtime(&now_t), "%Y-%m-%dT%H:%M:%SZ")
            << '\n';

        out << mininav::csv_header(mininav::SimStateV2{}) << '\n';

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
        const V2Preset preset = *preset_ptr;

        const std::uint64_t master_seed = resolve_seed(opts.seed);
        const Integrator integrator = integrator_from_name(opts.integrator_name);

        {
            std::ostringstream banner;
            banner << "MiniNav V2: preset = " << preset.name
                << ", seed = " << master_seed << ", mode = encoder+imu"
                << ", integrator = " << opts.integrator_name
                << ", q_scale = " << opts.q_scale << ", r_scale = " << opts.r_scale;
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

        ImuModel imu{
            ImuParams{
                .sigma_omega = preset.sigma_imu,
                .bias_omega_init = preset.imu_bias_init,
                .bias_random_walk = preset.imu_bias_rw,
            },
            rng_factory.make_engine("imu_gyro_noise"),
            rng_factory.make_engine("imu_gyro_bias")
        };

        // V1 的 wheel odometry 保留为 EKF 的对照基线。
        WheelOdometry odometry{
            WheelOdometryParams{
                .wheel_base = kWheelBase,
                .distance_per_tick = 2.0 * std::numbers::pi * kWheelRadius
                / static_cast<double>(kTicksPerRev),
            },
            Pose2D{0.0, 0.0, 0.0}
        };

        // ---- EKF (predict + encoder + IMU update, 6D 含 gyro bias) ------
        // 过程噪声 Q 用与 actuator 同源的 α(对 v、ω), 外加 bias 随机游走项
        // q_bias_omega; μ₀ = 0(无信息先验, 含 b_ω₀ = 0),
        // Σ₀ = diag(1e-6,1e-6,1e-6,1e-2,1e-2,1e-2)。每步执行三阶段:
        //   predict → update_encoder(2D 观测 v,ω) → update_imu(1D 观测 ω+b_ω)
        // 关键: bias 仅在 encoder+IMU 同时在场时可观测(见 ekf.ixx update_* 注释)。
        // Q 旋钮: 把 (α₁..₄, q_bias_omega) 整体乘 q_scale。Q 对这些参数线性, 故等价于
        // 缩放整个 Q 矩阵。注意只缩放【EKF 的 Q】, 上面 ActuatorModel 用的是未缩放的
        // preset.alpha*(真实噪声不变), 这样 q_scale 是纯粹的滤波器调参旋钮。
        Ekf ekf{
            make_initial_ekf_state(),
            ProcessNoiseParams{
                .alpha1 = preset.alpha1 * opts.q_scale, .alpha2 = preset.alpha2 * opts.q_scale,
                .alpha3 = preset.alpha3 * opts.q_scale, .alpha4 = preset.alpha4 * opts.q_scale,
                .q_bias_omega = preset.q_bias_omega * opts.q_scale,
            },
            integrator
        };

        // encoder 观测噪声参数 : 与 WheelEncoderModel 相同(slip_sigma、
        // distance_per_tick、wheel_base), 体现"R 由物理参数推导"。
        const EncoderNoiseParams enc_noise{
            .sigma_slip = preset.slip_sigma,
            .distance_per_tick = 2.0 * std::numbers::pi * kWheelRadius
            / static_cast<double>(kTicksPerRev),
            .wheel_base = kWheelBase,
        };

        // ---- Trajectory + CSV 输出准备 ----------------------------------
        Trajectory<SimStateV2> trajectory;
        constexpr auto step_count =
            static_cast<std::size_t>(kSimulationTotalTime / kSimulationDt) + 1;
        trajectory.reserve(step_count);

        const fs::path csv_path = opts.out_path.has_value()
            ? *opts.out_path
            : fs::path{PROJECT_ROOT_DIR} / "data" / "traj_v2.csv";

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

            register_v1_statics(*sink, kRobotEntityPath);
            sink->log_axes_static(kEkfEntity, 0.5F);
        }
        else
        {
            log::info("Rerun: disabled by --no-viz.");
        }

        log::info("MiniNav V2 simulation started.");

        // ---- 主循环 ------------------------------------------------------
        Pose2D truth_pose{0.0, 0.0, 0.0};
        Pose2D cmd_pose{0.0, 0.0, 0.0};
        Pose2D odom_pose{0.0, 0.0, 0.0};

        for (std::size_t i = 0; i < step_count; ++i)
        {
            const double t = static_cast<double>(i) * kSimulationDt;
            const Twist2D cmd = command_source.command_at(t);

            // 当前时刻 t 的传感测量:cmd → 执行噪声 → 编码器 + IMU
            const Twist2D true_velocity = actuator.apply(cmd);
            const EncoderTicks dticks = encoder.measure(true_velocity, kSimulationDt);
            const double imu_omega = imu.measure(true_velocity.w());

            // 记录本步开始时的 EKF belief 快照(mirror 了 odom_pose 的
            // "log-then-update" 约定: 末尾才推进)。
            const SimStateV2 state{
                .t = t,
                .cmd = cmd,
                .true_velocity = true_velocity,
                .truth_pose = truth_pose,
                .enc_dticks = dticks,
                .imu_omega = imu_omega,
                .odom_pose = odom_pose,
                .ekf_mean = ekf.mu(),
                .ekf_cov = ekf.Sigma(),
            };
            trajectory.append(state);

            if (sink.has_value())
            {
                sink->set_time(t);

                // 复用 V1 viz 通路处理 truth/odom(构造一个瞬时 V1 视图)。
                const SimStateV1 v1_view{
                    .t = t,
                    .cmd = cmd,
                    .true_velocity = true_velocity,
                    .truth_pose = truth_pose,
                    .enc_dticks = dticks,
                    .odom_pose = odom_pose,
                };
                log_to_rerun(*sink, v1_view, kRobotEntityPath);

                // EKF mean 轨迹
                const Pose2D ekf_pose{
                    ekf.mu()(kPx), ekf.mu()(kPy), ekf.mu()(kTheta)
                };
                sink->log_pose(kEkfEntity, ekf_pose);
                sink->log_trail_point(kEkfTrail, ekf_pose.x(), ekf_pose.y());

                // gyro bias 学习曲线: filter 估计 vs 仿真真值。
                // 在 Rerun Time Series 视图里能直接看到 b_ω 从 0 启动、几秒内
                // 收敛到真值附近, 这是 state augmentation 最有说服力的演示。
                sink->log_scalar(kEkfBiasOmega, ekf.mu()(kBiasOmega));
                sink->log_scalar(kTrueBiasOmega, imu.bias_omega());

                // cmd_traj 参考轨迹
                sink->log_pose(kCmdTrajEntity, cmd_pose);
                sink->log_trail_point(kCmdTrajTrail, cmd_pose.x(), cmd_pose.y());
            }

            truth_pose = differential_drive_step(truth_pose, true_velocity, kSimulationDt);
            cmd_pose = differential_drive_step(cmd_pose, cmd, kSimulationDt);
            odom_pose = odometry.update(dticks, kSimulationDt);

            // EKF 三阶段: predict → update_encoder → update_imu。
            ekf.predict(kSimulationDt);

            // encoder 观测: 解码 → 在预测速度处求 R → (R 旋钮) → Joseph update。
            // r_scale 只放大/缩小 EKF 对测量的信任, 不动上面真实生成的 dticks/imu_omega。
            const Eigen::Vector2d z_enc = decode_encoder(dticks, enc_noise, kSimulationDt);
            const Eigen::Matrix2d R_enc =
                encoder_noise_covariance(ekf.mu()(kV), ekf.mu()(kOmega), enc_noise, kSimulationDt)
                * opts.r_scale;
            ekf.update_encoder(z_enc, R_enc);

            // IMU 观测: 标量 ω, R = σ_imu²
            const double R_imu = preset.sigma_imu * preset.sigma_imu * opts.r_scale;
            ekf.update_imu(imu_omega, R_imu);
        }

        write_csv_with_metadata(trajectory, csv_path, master_seed, preset.name,
                                opts.integrator_name, opts.q_scale, opts.r_scale);
        log::info("Trajectory CSV written to " + csv_path.string());
        log::info("MiniNav V2 simulation ended.");
    }
    catch (const std::exception& ex)
    {
        log::error(ex.what());
        return 1;
    }
    return 0;
}
