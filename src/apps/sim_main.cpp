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
import mininav.viz.plan_log;
import mininav.planning.grid_types;
import mininav.planning.occupancy_grid;
import mininav.planning.map_io;
import mininav.planning.inflation;
import mininav.planning.planner_config;
import mininav.planning.astar;

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
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

// ===========================================================================
// MiniNav simulation.
//
// 一个二进制,两种模式:
//   默认      室内差分驱动机器人的定位仿真:
//             每步 cmd → 执行噪声 → encoder + IMU 测量 → wheel-odometry 基线 + 6D EKF
//             (predict + encoder update + IMU update, 含在线 gyro-bias 估计)。
//   --map     一次性、无 RNG 的全局规划(V3):
//             load map → inflate → A* → path.csv + Rerun 规划视图(run_planning)。
//
// 代码分层:
//   NoisePreset   —— 三档噪声标定(纯数据)。
//   CliOptions    —— 命令行选项(CLI11 直接绑定到结构体成员)。
//   SimConfig     —— 解析/求值后的运行配置(preset + seed + 旋钮)。
//   Simulator     —— 估计管线: 拥有全部仿真组件与 EKF, step(t) 推进一帧。
//   run_planning  —— 规划模式入口: 地图 / 配置 / 起止点 → PlanResult → CSV + viz。
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

        // ---- V3 planning mode (triggered by --map) ----
        std::optional<std::string> map_path;       // map.yaml; presence => planning mode
        std::optional<std::string> config_path;    // planner.yaml (PlannerConfig)
        std::optional<std::string> start_str;      // "x,y" world start (default: grid center)
        std::optional<std::string> goal_str;       // "x,y" world goal (required in planning mode)
        std::optional<std::string> heuristic_name; // manhattan / euclidean / octile
        std::optional<int> connectivity;           // 4 / 8
        std::optional<double> inflation_radius;    // meters
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

        // ---- V3 planning mode --------------------------------------------------
        // Passing --map switches sim from the V2 EKF motion loop to a one-shot,
        // RNG-free global-planning run (load map -> inflate -> A* -> path.csv + viz).
        app.add_option("--map", opts.map_path,
                       "Map description (map.yaml, ROS map_server style). "
                       "Presence switches sim to one-shot planning mode.");
        app.add_option("--config", opts.config_path,
                       "Planner config (planner.yaml). Planning-mode CLI flags override it.");
        app.add_option("--start", opts.start_str,
                       "Planning start as \"x,y\" in world meters (default: grid center).");
        app.add_option("--goal", opts.goal_str,
                       "Planning goal as \"x,y\" in world meters (required in planning mode).");
        app.add_option("--heuristic", opts.heuristic_name,
                       "A* heuristic override (manhattan requires --connectivity 4).")
           ->check(CLI::IsMember({"manhattan", "euclidean", "octile"}));
        app.add_option("--connectivity", opts.connectivity,
                       "Grid connectivity override (4 or 8).");
        app.add_option("--inflation-radius", opts.inflation_radius,
                       "Obstacle inflation radius override (meters).")
           ->check(CLI::NonNegativeNumber);

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

    // =======================================================================
    // V3 planning mode (triggered by --map).
    //
    // 一次性、**无 RNG** 的全局规划:load map -> inflate -> A* -> path.csv + viz。
    // 与 V2 的逐步运动仿真分流(同一个 sim 二进制,演进而非并存,见 docs/v3_summary.md §2.3)。
    //
    // 起点 start 是一个普通 Pose2D —— 在完整系统里它就是 V2 EKF 的估计位姿
    // (这是 V2→V3 的接缝)。但规划入口刻意保持无 RNG / 逐字节确定(docs/v3_summary.md §3.8:
    // 同 map+start+goal+config → path.csv 逐字节一致),所以不在此入口里跑会消耗
    // RNG 的 EKF;EKF→start 的注入留给上层调用方。
    // =======================================================================

    // 解析 "x,y" world 坐标(米)。格式错误抛 std::runtime_error。
    [[nodiscard]] Eigen::Vector2d parse_xy(const std::string& s)
    {
        const auto comma = s.find(',');
        if (comma == std::string::npos)
        {
            throw std::runtime_error{"expected \"x,y\", got: " + s};
        }
        try
        {
            const double x = std::stod(s.substr(0, comma));
            const double y = std::stod(s.substr(comma + 1));
            return Eigen::Vector2d{x, y};
        }
        catch (const std::exception&)
        {
            throw std::runtime_error{"could not parse \"x,y\" from: " + s};
        }
    }

    [[nodiscard]] mininav::planning::Heuristic heuristic_from_name(const std::string& s)
    {
        using mininav::planning::Heuristic;
        if (s == "manhattan") { return Heuristic::Manhattan; }
        if (s == "euclidean") { return Heuristic::Euclidean; }
        return Heuristic::Octile; // CLI 已用 IsMember 校验
    }

    [[nodiscard]] std::string heuristic_name_of(const mininav::planning::Heuristic h)
    {
        using mininav::planning::Heuristic;
        switch (h)
        {
        case Heuristic::Manhattan: return "manhattan";
        case Heuristic::Euclidean: return "euclidean";
        case Heuristic::Octile: return "octile";
        }
        return "euclidean";
    }

    // 栅格中心的 world 坐标(默认起点)。origin 是左下角,故 +半幅宽高。
    [[nodiscard]] Eigen::Vector2d grid_center(const mininav::planning::OccupancyGrid& g)
    {
        return g.origin() + Eigen::Vector2d{
                   g.width() * g.resolution() * 0.5,
                   g.height() * g.resolution() * 0.5
               };
    }

    // 把原图占据 cell 与"膨胀新增"cell 分开收集,供 viz 的安全裕度展示。
    [[nodiscard]] mininav::PlanScene build_plan_scene(
        const mininav::planning::OccupancyGrid& grid,
        const mininav::planning::OccupancyGrid& inflated,
        const mininav::planning::PlanResult& result,
        const mininav::Pose2D& start, const mininav::Pose2D& goal)
    {
        using namespace mininav::planning;
        mininav::PlanScene scene;
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
        for (const mininav::Pose2D& p : result.path.poses)
        {
            scene.path.emplace_back(p.x(), p.y());
        }
        return scene;
    }

    // path.csv:逐字节确定(无时间戳、无 plan_time_ms)。header 嵌入 map/start/goal/
    // heuristic/connectivity/inflation_radius/success/expanded_nodes/path_length_m,
    // 一次规划自包含、可复现、可比对(docs/v3_summary.md §4.3)。
    void write_path_csv(const fs::path& path, const CliOptions& opts,
                        const mininav::planning::PlannerConfig& cfg,
                        const mininav::Pose2D& start, const mininav::Pose2D& goal,
                        const mininav::planning::PlanResult& result)
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
        out << "# map = " << *opts.map_path << '\n';
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
            const mininav::Pose2D& p = result.path.poses[i];
            out << i << ',' << p.x() << ',' << p.y() << ',' << p.yaw() << '\n';
        }
    }

    void run_planning(const CliOptions& opts)
    {
        using namespace mininav;
        using namespace mininav::planning;

        if (!opts.goal_str.has_value())
        {
            throw std::runtime_error{"planning mode (--map) requires --goal \"x,y\""};
        }

        // 1. 地图
        OccupancyGrid grid = load_occupancy_grid(*opts.map_path);

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

        // 3. 起止点(start 缺省 = 栅格中心;goal 必填)
        const Eigen::Vector2d goal_xy = parse_xy(*opts.goal_str);
        const Eigen::Vector2d start_xy = opts.start_str.has_value()
                                             ? parse_xy(*opts.start_str)
                                             : grid_center(grid);
        const Pose2D start{start_xy.x(), start_xy.y(), 0.0};
        const Pose2D goal{goal_xy.x(), goal_xy.y(), 0.0};

        // 4. 规划
        const AStarPlanner planner{grid, cfg};
        const PlanResult result = planner.plan(start, goal);

        std::ostringstream metrics;
        metrics << "plan: map=" << *opts.map_path
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
        const fs::path out_path = opts.out_path.has_value()
                                      ? fs::path{*opts.out_path}
                                      : fs::path{PROJECT_ROOT_DIR} / "data" / "path.csv";
        write_path_csv(out_path, opts, cfg, start, goal, result);
        log::info("Plan CSV written to " + out_path.string());

        // 6. 可视化(地图 + 膨胀 + 路径 + 起止)
        if (opts.disable_viz)
        {
            log::info("Rerun: disabled by --no-viz.");
            return;
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
        sink->log_axes_static("/world/origin", 0.5F);
        sink->set_time(0.0);
        const OccupancyGrid inflated = inflate(grid, cfg.inflation_radius);
        const PlanScene scene = build_plan_scene(grid, inflated, result, start, goal);
        log_plan(*sink, scene, "/world");
    }
}

int main(int argc, char** argv)
{
    using namespace mininav;
    using namespace mininav::ekf; // kBiasOmega 下标

    try
    {
        const CliOptions opts = parse_cli(argc, argv);

        // --map switches to one-shot global-planning mode (V3); otherwise the
        // V2 EKF motion simulation runs unchanged.
        if (opts.map_path.has_value())
        {
            run_planning(opts);
            return 0;
        }

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
