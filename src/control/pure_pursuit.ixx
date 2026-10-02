module;

#include <Eigen/Core>

#include <cstddef>
#include <string_view>
#include <vector>

export module mininav.control.pure_pursuit;

import mininav.core.types;
import mininav.core.path;
import mininav.core.robot_description;
import mininav.control.controller;

export namespace mininav::control
{
    // ---------------------------------------------------------------------------
    // PurePursuitConfig: Regulated Pure Pursuit(Nav2 RPP)核心子集的参数。
    // 默认值由 docs/v4_plan.md §3 的推导给出(对应 config/nav.yaml 的 controller 段):
    //
    //   lookahead_time = 1.0 s     滞后稳定性:T_L ≥ 5·τ_eff(τ_eff ≈ τ + T_c/2 = 0.125 s)
    //   min_lookahead  = 0.10 m    切角预算:L_min ≤ m_eff / f(90°) ≈ 0.16 m
    //   R_min = 0.60 m, L_κ = 0.40 m   90° 转角处把速度压到 ≈ 0.10–0.14 m/s
    //
    // 三个 use_* 开关用于消融实验:全部关掉(固定 L_d、不限速、不原地转)就是经典
    // Pure Pursuit。
    // ---------------------------------------------------------------------------
    struct PurePursuitConfig
    {
        double desired_linear_vel{0.30}; // m/s,巡航速度

        // look-ahead 距离:速度自适应时 L_d = clamp(T_L·|v̂|, L_min, L_max),
        // 让滞后比 τ/T_L 与速度无关;关闭时用固定的 lookahead_dist。
        bool use_velocity_scaled_lookahead{true};
        double lookahead_time{1.0};  // s,T_L
        double min_lookahead{0.10};  // m
        double max_lookahead{0.60};  // m
        double lookahead_dist{0.30}; // m,固定 L_d

        // 曲率限速:用固定距离 L_κ 的 look-ahead 点求调节曲率,半径 R < R_min 时
        // v ← v·R/R_min。固定 L_κ 切断"降速 → L_d 变短 → 曲率变大 → 再降速"的正反馈。
        bool use_curvature_regulation{true};
        double curvature_lookahead_dist{0.40}; // m,L_κ
        double regulated_min_radius{0.60};     // m,R_min

        // 接近目标:剩余路程 d < approach_dist 时 v ≤ max(min_approach_vel, v_des·d/approach_dist)。
        double approach_dist{0.40};    // m
        double min_approach_vel{0.05}; // m/s

        // 原地转向:look-ahead 点相对车头的夹角超过阈值时 v = 0、ω = ±rotate_vel;
        // 位置到达且要求朝向时,同样原地转到目标朝向。
        bool use_rotate_to_heading{true};
        double rotate_to_heading_angle{0.785}; // rad
        double rotate_vel{1.0};                // rad/s

        // 进度投影的搜索范围(沿路径,从当前段起):防止回折路径上跳段。
        double max_projection_search_dist{1.0}; // m
    };

    // 参数合法性检查;不合法抛 std::invalid_argument。
    void validate(const PurePursuitConfig& cfg);

    // 本拍的控制工况。
    enum class PursuitRegime
    {
        Stopped,      // 无路径,或已在路径终点
        RotateToPath, // look-ahead 点夹角过大,原地转向
        Track,        // 正常跟踪
        Approach,     // 进入接近目标区,减速
        RotateToGoal, // 位置已到,原地转到目标朝向
    };

    [[nodiscard]] std::string_view to_string(PursuitRegime regime) noexcept;

    // 最近一次 compute 的中间量,供 CSV 与 Rerun(纯数据)。
    struct PursuitDebug
    {
        PursuitRegime regime{PursuitRegime::Stopped};
        Eigen::Vector2d lookahead_point{Eigen::Vector2d::Zero()};
        double lookahead_dist{0.0};    // 本拍的 L_d
        double curvature{0.0};         // 转向曲率 κ = 2·y_g / d²
        double cross_track_error{0.0}; // 控制器看到的横向误差 e_ctrl(输入位姿到路径)
        double arclength{0.0};         // 投影点弧长(跟踪进度)
    };

    // ---------------------------------------------------------------------------
    // PurePursuitController: 几何路径跟踪。
    //
    // 车体系下取路径上距车 L 的点 G = (x_g, y_g),求过车、与车头相切且过 G 的圆弧:
    // κ = 2·y_g / (x_g² + y_g²) = 2·sin α / L,ω = v·κ(推导见 docs/math/pure_pursuit.md)。
    // 离终点不足 L 时 G 取终点,分母用实际距离 d² 而不是 L²。
    //
    // 输入位姿可以是 EKF 估计或真值(oracle 实验),控制器不区分:它只对"输入位姿到
    // 路径"的误差负责。输出不含加速度限幅,由 VelocitySmoother 处理。
    // ---------------------------------------------------------------------------
    class PurePursuitController final : public Controller
    {
    public:
        // limits:执行器饱和,用于角速度约束与参数校验(desired_linear_vel ≤ max_linear_vel,
        // rotate_vel ≤ max_angular_vel)。参数不合法抛 std::invalid_argument。
        PurePursuitController(const PurePursuitConfig& cfg, const RobotLimits& limits);

        void set_plan(const Path& path) override;
        [[nodiscard]] Twist2D compute_velocity_commands(const Pose2D& pose, const Twist2D& velocity,
                                                        const GoalChecker* goal_checker) override;
        void set_speed_limit(double limit, bool percentage) override;
        void reset() override;

        [[nodiscard]] const PursuitDebug& debug() const noexcept { return debug_; }
        [[nodiscard]] const PurePursuitConfig& config() const noexcept { return cfg_; }

    private:
        [[nodiscard]] double lookahead_distance(double speed) const noexcept;
        [[nodiscard]] std::size_t search_window() const noexcept;
        [[nodiscard]] double rotate_command(double angle_error) const noexcept;
        [[nodiscard]] double regulated_velocity(const Pose2D& pose, const PathProjection& proj,
                                                double kappa);

        PurePursuitConfig cfg_;
        RobotLimits limits_;
        Path plan_;
        std::vector<double> segment_starts_; // 各线段起点的弧长
        double plan_length_{0.0};
        std::size_t segment_{0};  // 当前进度所在线段,单调不减
        double progress_{0.0};    // 上一拍投影点的弧长
        double speed_cap_{0.0};   // 限速后的期望线速度
        PursuitDebug debug_{};
    };
}
