module;

#include <Eigen/Core>

export module mininav.core.types;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // Pose2D: 二维平面上的机器人位姿
    // ---------------------------------------------------------------------------
    //
    // 内部表示:
    //   - position_: Eigen::Vector2d,为未来与协方差矩阵 / EKF 打通做准备
    //   - yaw_:      弧度制,约定范围 (-pi, pi],由 wrap_angle 保证
    //
    // 设计约定:
    //   - 所有写入操作(构造、set_*)不主动 wrap yaw,由调用方负责;
    //     这样可以避免在高频路径里反复 wrap,也让 RobotModel 这类组件
    //     拥有对 wrap 时机的明确控制权。
    //   - 和 EKF 状态向量的约定: [x, y, yaw]^T
    // ---------------------------------------------------------------------------
    class Pose2D
    {
    public:
        Pose2D() noexcept = default;

        Pose2D(const double x, const double y, const double yaw) noexcept
            : position_{x, y}, yaw_{yaw}
        {
        }

        Pose2D(Eigen::Vector2d position, const double yaw) noexcept
            : position_{std::move(position)}, yaw_{yaw}
        {
        }

        [[nodiscard]] double x() const noexcept { return position_.x(); }
        [[nodiscard]] double y() const noexcept { return position_.y(); }
        [[nodiscard]] double yaw() const noexcept { return yaw_; }

        [[nodiscard]] const Eigen::Vector2d& position() const noexcept
        {
            return position_;
        }

        void set_x(const double x) noexcept { position_.x() = x; }
        void set_y(const double y) noexcept { position_.y() = y; }
        void set_yaw(const double yaw) noexcept { yaw_ = yaw; }

        void set_position(const Eigen::Vector2d& position) noexcept
        {
            position_ = position;
        }

        [[nodiscard]] Eigen::Vector3d to_vector() const noexcept;
        [[nodiscard]] static Pose2D from_vector(const Eigen::Vector3d& v) noexcept;

    private:
        Eigen::Vector2d position_{Eigen::Vector2d::Zero()};
        double yaw_{0.0};
    };

    // ---------------------------------------------------------------------------
    // Twist2D: 差分驱动机器人的控制量 / 速度
    //   v: 线速度 [m/s]
    //   w: 角速度 [rad/s]
    // ---------------------------------------------------------------------------
    class Twist2D
    {
    public:
        Twist2D() noexcept = default;

        Twist2D(const double v, const double w) noexcept : v_{v}, w_{w}
        {
        }

        [[nodiscard]] double v() const noexcept { return v_; }
        [[nodiscard]] double w() const noexcept { return w_; }

        void set_v(const double v) noexcept { v_ = v; }
        void set_w(const double w) noexcept { w_ = w; }

    private:
        double v_{0.0};
        double w_{0.0};
    };

    // ---------------------------------------------------------------------------
    // SimStateV0: V0 仿真器每一步的状态快照
    //
    // V0 阶段记录三件事:时间戳、当前真实位姿、当前施加的控制量。
    // 这是"理想运动仿真"——没有噪声、没有估计、没有规划。
    //
    // 设计原则:
    //   - SimState 是按版本演化的纯数据结构,不强求版本间向后兼容。
    //   - 每个 SimStateVN 对应那个版本的"我们关心什么"。
    //     V1 会新增 odom_pose 等字段,V2 会新增 ekf_pose、协方差等。
    // ---------------------------------------------------------------------------
    struct SimStateV0
    {
        double t{0.0};
        Pose2D pose{};
        Twist2D cmd{};
    };
}
