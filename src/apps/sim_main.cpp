import mininav.core.types;
import mininav.core.command_source;
import mininav.core.trajectory;
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

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <numbers>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <string_view>

// ===========================================================================
// MiniNav simulation.
//
// 室内差分驱动机器人的定位仿真:
//   每步 cmd → 执行噪声 → encoder + IMU 测量 → wheel-odometry 基线 + 6D EKF
//   (predict + encoder update + IMU update, 含在线 gyro-bias 估计)。
//
// 代码分层:
//   NoisePreset   —— 三档噪声标定(纯数据)。
//   CliOptions    —— 命令行选项(CLI11 直接绑定到结构体成员)。
//   SimConfig     —— 解析/求值后的运行配置(preset + seed + 旋钮)。
//   Simulator     —— 估计管线: 拥有全部仿真组件与 EKF, step(t) 推进一帧。
//   main          —— 只做 I/O / 可视化 / CSV 落盘, 不含仿真逻辑。
// ===========================================================================

namespace
{
    namespace fs = std::filesystem;

    // ---- 仿真时间 ----------------------------------------------------------
    constexpr double kDt = 0.01;
    constexpr double kTotalTime = 20.0;
    constexpr std::size_t kStepCount =
        static_cast<std::size_t>(kTotalTime / kDt) + 1;

    // ---- 机器人几何 (单一来源, 所有派生量都从这里算) ----------------------
    constexpr double kWheelRadius = 0.032;       // m
    constexpr double kWheelBase = 0.150;         // m
    constexpr std::int64_t kTicksPerRev = 1024;
    constexpr double kDistancePerTick =
        2.0 * std::numbers::pi * kWheelRadius / static_cast<double>(kTicksPerRev);

    // ---- Rerun 实体路径 ----------------------------------------------------
    constexpr std::string_view kApplicationId = "mininav";
    constexpr std::string_view kRobotEntityPath = "/world/robot";
    // cmd_traj(假设完美执行 cmd 的参考轨迹)与 bias 对照曲线不属于 SimState,
    // 由主循环单独 log。trail 路径遵循 sim_state_log 约定: /world/trails/{name}。
    constexpr std::string_view kCmdTrajEntity = "/world/robot/cmd_traj";
    constexpr std::string_view kCmdTrajTrail = "/world/trails/cmd_traj";
    constexpr std::string_view kEkfBiasOmega = "/plots/bias_omega/ekf";
    constexpr std::string_view kTrueBiasOmega = "/plots/bias_omega/truth";

    // =======================================================================
    // NoisePreset: 三档噪声标定(工程合理值)。
    // =======================================================================
    struct NoisePreset
    {
        std::string_view name;
        double alpha1, alpha2, alpha3, alpha4; // Velocity Motion Model
        double slip_sigma;                     // 编码器打滑标准差
        double sigma_imu;                      // IMU gyro 白噪声标准差 [rad/s]
        double imu_bias_init;                  // IMU gyro bias 真值(filter 待估计)[rad/s]
        double imu_bias_rw;                    // bias 每步随机游走标准差 [rad/s]; 0 => 常数 bias
        double q_bias_omega;                   // EKF 对 bias 的过程噪声(单步方差)(rad/s)²
    };

    // imu_bias_rw 默认置 0(常数 bias); 调成小正数(如 5e-5)即可观察 filter 跟踪
    // 漂移 bias 的能力。EKF 侧 q_bias_omega 保持小正值, 使 filter 即便面对常数 bias
    // 也维持一点自适应余量(标准工业做法)。
    constexpr NoisePreset kPresetLowNoise{
        .name = "low-noise",
        .alpha1 = 0.01, .alpha2 = 0.005, .alpha3 = 0.005, .alpha4 = 0.01,
        .slip_sigma = 0.005,
        .sigma_imu = 0.002,
        .imu_bias_init = 0.01,
        .imu_bias_rw = 0.0,
        .q_bias_omega = 1e-8,
    };
    constexpr NoisePreset kPresetDefault{
        .name = "default",
        .alpha1 = 0.05, .alpha2 = 0.02, .alpha3 = 0.02, .alpha4 = 0.05,
        .slip_sigma = 0.02,
        .sigma_imu = 0.005,
        .imu_bias_init = 0.02,
        .imu_bias_rw = 0.0,
        .q_bias_omega = 1e-8,
    };
    constexpr NoisePreset kPresetHighNoise{
        .name = "high-noise",
        .alpha1 = 0.15, .alpha2 = 0.08, .alpha3 = 0.08, .alpha4 = 0.15,
        .slip_sigma = 0.05,
        .sigma_imu = 0.015,
        .imu_bias_init = 0.03,
        .imu_bias_rw = 0.0,
        .q_bias_omega = 4e-8,
    };

    constexpr std::array kPresetTable{kPresetLowNoise, kPresetDefault, kPresetHighNoise};

    // CLI 的 --preset 已用 IsMember 校验, 故此处必有命中; 兜底返回 default 仅为防御。
    [[nodiscard]] const NoisePreset& find_preset(std::string_view name) noexcept
    {
        for (const auto& preset : kPresetTable)
        {
            if (preset.name == name)
            {
                return preset;
            }
        }
        return kPresetDefault;
    }

    // =======================================================================
    // CliOptions / parse_cli
    // =======================================================================
    struct CliOptions
    {
        std::optional<std::uint64_t> seed;
        std::string preset_name{"default"};
        std::string integrator_name{"rk4"};
        bool disable_viz{false};
        std::optional<std::string> rrd_path;
        std::optional<std::string> out_path;
        double q_scale{1.0};
        double r_scale{1.0};
        bool no_bias{false};
    };

    [[nodiscard]] CliOptions parse_cli(int argc, char** argv)
    {
        CliOptions opts{};

        CLI::App app{
            "MiniNav simulation: actuator + encoder + IMU noise, wheel-odometry "
            "baseline, and a 6D EKF fusing encoder + gyro with online bias estimation."
        };

        // CLI11 直接绑定到 CliOptions 成员, 省去"声明局部 + 回填结构体"两遍。
        app.add_option("--seed", opts.seed,
                       "Master RNG seed; if omitted, seeded from std::random_device.");

        app.add_option("--preset", opts.preset_name, "Noise preset.")
           ->capture_default_str()
           ->check(CLI::IsMember({"low-noise", "default", "high-noise"}));

        app.add_option("--integrator", opts.integrator_name,
                       "EKF process-model integrator. rk4 = production; euler is kept only "
                       "for the RK4-vs-Euler attribution experiment.")
           ->capture_default_str()
           ->check(CLI::IsMember({"euler", "rk4"}));

        app.add_option("--out", opts.out_path,
                       "Output CSV path (default: data/traj.csv). Set this to keep the "
                       "euler / rk4 runs in separate files for analyze_integrator.py.");

        app.add_option("--q-scale", opts.q_scale,
                       "Multiplier on the EKF process noise Q (sensitivity analysis). "
                       "Default 1.0 keeps the physics-derived value; >1 trusts the motion "
                       "model less, <1 trusts it more. Does NOT touch the simulated truth.")
           ->capture_default_str()
           ->check(CLI::PositiveNumber);

        app.add_option("--r-scale", opts.r_scale,
                       "Multiplier on the EKF measurement noise R (encoder + IMU). "
                       "Default 1.0 keeps the physics-derived value; >1 trusts the sensors "
                       "less. Does NOT touch the simulated measurements.")
           ->capture_default_str()
           ->check(CLI::PositiveNumber);

        app.add_flag("--no-bias", opts.no_bias,
                     "Disable online gyro-bias estimation (force q_bias_omega = 0, the "
                     "no-bias compatibility path). Produces the 'ekf (no bias)' baseline for "
                     "the three-way RMSE comparison against 'ekf_with_bias'.");

        auto* rrd_opt = app.add_option("--rrd", opts.rrd_path,
                                       "Save Rerun recording to the given .rrd path.");
        auto* noviz_opt = app.add_flag("--no-viz", opts.disable_viz,
                                       "Disable Rerun output entirely; CSV-only run (for CI / regression).");
        rrd_opt->excludes(noviz_opt);

        try
        {
            app.parse(argc, argv);
        }
        catch (const CLI::ParseError& e)
        {
            std::exit(app.exit(e)); // --help → stdout/exit 0; 错误 → stderr/exit !=0
        }

        return opts;
    }

    [[nodiscard]] mininav::ekf::Integrator integrator_from_name(std::string_view name) noexcept
    {
        return name == "euler" ? mininav::ekf::Integrator::Euler : mininav::ekf::Integrator::Rk4;
    }

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

    // =======================================================================
    // SimConfig: CliOptions 解析/求值后的运行配置。
    // =======================================================================
    struct SimConfig
    {
        const NoisePreset& preset;
        std::uint64_t seed;
        mininav::ekf::Integrator integrator;
        std::string_view integrator_name;
        double q_scale;
        double r_scale;
        bool bias_on;

        [[nodiscard]] static SimConfig from(const CliOptions& opts)
        {
            return SimConfig{
                .preset = find_preset(opts.preset_name),
                .seed = resolve_seed(opts.seed),
                .integrator = integrator_from_name(opts.integrator_name),
                .integrator_name = opts.integrator_name,
                .q_scale = opts.q_scale,
                .r_scale = opts.r_scale,
                .bias_on = !opts.no_bias,
            };
        }
    };

    // =======================================================================
    // Simulator: 估计管线。拥有全部仿真组件与 EKF, 每次 step 推进一帧并返回
    // 该帧的 SimState 快照(其中 truth / odom / ekf 均为推进前的 prior belief,
    // NIS 为本帧 update 的产物)。RNG 消费顺序: actuator → encoder → imu。
    // =======================================================================
    class Simulator
    {
    public:
        Simulator(const SimConfig& cfg, const mininav::RngFactory& rng)
            : actuator_{
                  mininav::ActuatorNoiseParams{
                      .alpha1 = cfg.preset.alpha1, .alpha2 = cfg.preset.alpha2,
                      .alpha3 = cfg.preset.alpha3, .alpha4 = cfg.preset.alpha4,
                  },
                  rng.make_engine("actuator")
              },
              encoder_{
                  mininav::WheelEncoderParams{
                      .wheel_radius = kWheelRadius,
                      .wheel_base = kWheelBase,
                      .ticks_per_rev = kTicksPerRev,
                      .slip_sigma = cfg.preset.slip_sigma,
                  },
                  rng.make_engine("encoder_slip_left"),
                  rng.make_engine("encoder_slip_right")
              },
              imu_{
                  mininav::ImuParams{
                      .sigma_omega = cfg.preset.sigma_imu,
                      .bias_omega_init = cfg.preset.imu_bias_init,
                      .bias_random_walk = cfg.preset.imu_bias_rw,
                  },
                  rng.make_engine("imu_gyro_noise"),
                  rng.make_engine("imu_gyro_bias")
              },
              odometry_{
                  mininav::WheelOdometryParams{
                      .wheel_base = kWheelBase,
                      .distance_per_tick = kDistancePerTick,
                  },
                  mininav::Pose2D{0.0, 0.0, 0.0}
              },
              enc_noise_{
                  .sigma_slip = cfg.preset.slip_sigma,
                  .distance_per_tick = kDistancePerTick,
                  .wheel_base = kWheelBase,
              },
              ekf_{
                  mininav::ekf::make_initial_ekf_state(),
                  // Q 旋钮: (α₁..₄, q_bias_omega) 整体乘 q_scale(只缩放 EKF 的 Q,
                  // 不动真实噪声)。--no-bias 强制 q_bias_omega = 0 → IMU 走无 bias 兼容路径。
                  mininav::ekf::ProcessNoiseParams{
                      .alpha1 = cfg.preset.alpha1 * cfg.q_scale,
                      .alpha2 = cfg.preset.alpha2 * cfg.q_scale,
                      .alpha3 = cfg.preset.alpha3 * cfg.q_scale,
                      .alpha4 = cfg.preset.alpha4 * cfg.q_scale,
                      .q_bias_omega = cfg.bias_on ? cfg.preset.q_bias_omega * cfg.q_scale : 0.0,
                  },
                  cfg.integrator
              },
              r_scale_{cfg.r_scale},
              sigma_imu_{cfg.preset.sigma_imu}
        {
        }

        [[nodiscard]] mininav::SimState step(double t)
        {
            using namespace mininav;
            using namespace mininav::ekf; // kV / kOmega 下标

            // 当前时刻 t 的测量: cmd → 执行噪声 → encoder + IMU。
            const Twist2D cmd = command_source_.command_at(t);
            const Twist2D true_velocity = actuator_.apply(cmd);
            const EncoderTicks dticks = encoder_.measure(true_velocity, kDt);
            const double imu_omega = imu_.measure(true_velocity.w());

            // 快照本帧开始时的 belief(prior): truth / odom / ekf 均为推进前的值。
            SimState state{
                .t = t,
                .cmd = cmd,
                .true_velocity = true_velocity,
                .truth_pose = truth_pose_,
                .enc_dticks = dticks,
                .imu_omega = imu_omega,
                .odom_pose = odom_pose_,
                .ekf_mean = ekf_.mu(),
                .ekf_cov = ekf_.Sigma(),
            };

            // 推进真值与 wheel-odometry 基线(prior 已快照)。
            truth_pose_ = differential_drive_step(truth_pose_, true_velocity, kDt);
            odom_pose_ = odometry_.update(dticks, kDt);

            // EKF 三阶段: predict → update_encoder(2D 观测 v,ω) → update_imu(1D 观测 ω+b_ω)。
            ekf_.predict(kDt);

            // encoder 观测: 解码 → 在预测速度处求 R → (r_scale 旋钮) → Joseph update。
            const Eigen::Vector2d z_enc = decode_encoder(dticks, enc_noise_, kDt);
            const Eigen::Matrix2d R_enc =
                encoder_noise_covariance(ekf_.mu()(kV), ekf_.mu()(kOmega), enc_noise_, kDt)
                * r_scale_;
            state.nis_encoder = ekf_.update_encoder(z_enc, R_enc);

            // IMU 观测: 标量 ω, R = σ_imu²。
            const double R_imu = sigma_imu_ * sigma_imu_ * r_scale_;
            state.nis_imu = ekf_.update_imu(imu_omega, R_imu);

            return state;
        }

        // gyro bias 真值, 供 viz 的 bias 学习曲线对照(filter 估计应趋近它)。
        [[nodiscard]] double true_bias_omega() const noexcept { return imu_.bias_omega(); }

    private:
        mininav::StagedCommandSource command_source_{};
        mininav::ActuatorModel actuator_;
        mininav::WheelEncoderModel encoder_;
        mininav::ImuModel imu_;
        mininav::WheelOdometry odometry_;
        mininav::EncoderNoiseParams enc_noise_;
        mininav::ekf::Ekf ekf_;
        double r_scale_;
        double sigma_imu_;

        mininav::Pose2D truth_pose_{0.0, 0.0, 0.0};
        mininav::Pose2D odom_pose_{0.0, 0.0, 0.0};
    };

    // =======================================================================
    // make_sink: 按 CLI 选项装配 Rerun sink(spawn / save / off 三模式)。
    // =======================================================================
    [[nodiscard]] std::optional<mininav::RerunSink> make_sink(const CliOptions& opts)
    {
        using namespace mininav;

        if (opts.disable_viz)
        {
            log::info("Rerun: disabled by --no-viz.");
            return std::nullopt;
        }

        std::optional<RerunSink> sink;
        if (opts.rrd_path.has_value())
        {
            sink.emplace(kApplicationId, fs::path{*opts.rrd_path});
            log::info("Rerun: writing to " + *opts.rrd_path);
        }
        else
        {
            sink.emplace(kApplicationId);
            log::info("Rerun: Viewer spawned (gRPC).");
        }

        register_statics(*sink, kRobotEntityPath);
        return sink;
    }

    // =======================================================================
    // write_csv_with_metadata: 带可复现实验元数据的 CSV 落盘。
    // 头部注释嵌入 seed / preset / dt / duration / 旋钮, 供 Python 脚本精确重放。
    // =======================================================================
    void write_csv_with_metadata(const mininav::Trajectory<mininav::SimState>& traj,
                                 const fs::path& path,
                                 const SimConfig& cfg)
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

        out << mininav::csv_header(mininav::SimState{}) << '\n';
        for (const auto& record : traj.records())
        {
            out << mininav::csv_row(record) << '\n';
        }
    }

    [[nodiscard]] std::string make_banner(const SimConfig& cfg)
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

int main(int argc, char** argv)
{
    using namespace mininav;
    using namespace mininav::ekf; // kBiasOmega 下标

    try
    {
        const CliOptions opts = parse_cli(argc, argv);
        const SimConfig cfg = SimConfig::from(opts);
        log::info(make_banner(cfg));

        const fs::path csv_path = opts.out_path.has_value()
                                      ? fs::path{*opts.out_path}
                                      : fs::path{PROJECT_ROOT_DIR} / "data" / "traj.csv";

        Simulator simulator{cfg, RngFactory{cfg.seed}};
        std::optional<RerunSink> sink = make_sink(opts);

        Trajectory<SimState> trajectory;
        trajectory.reserve(kStepCount);

        log::info("MiniNav simulation started.");

        // cmd_traj 是纯 viz 的"完美执行 cmd 参考轨迹", 不进入估计管线, 故由主循环维护。
        Pose2D cmd_pose{0.0, 0.0, 0.0};

        for (std::size_t i = 0; i < kStepCount; ++i)
        {
            const double t = static_cast<double>(i) * kDt;
            const SimState state = simulator.step(t);

            if (sink.has_value())
            {
                sink->set_time(t);
                log_to_rerun(*sink, state, kRobotEntityPath);

                // gyro bias 学习曲线: filter 估计 vs 仿真真值(state augmentation 演示)。
                sink->log_scalar(kEkfBiasOmega, state.ekf_mean(kBiasOmega));
                sink->log_scalar(kTrueBiasOmega, simulator.true_bias_omega());

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
    catch (const std::exception& ex)
    {
        log::error(ex.what());
        return 1;
    }
    return 0;
}
