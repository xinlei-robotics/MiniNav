export module mininav.control.controller;

import mininav.core.types;
import mininav.core.path;

export namespace mininav::control
{
    // ---------------------------------------------------------------------------
    // GoalTolerance: 到达判定的容差,由 GoalChecker 提供给控制器(Nav2 的
    // GoalChecker::getTolerances)。check_yaw = false 时只看位置。
    // ---------------------------------------------------------------------------
    struct GoalTolerance
    {
        double xy{0.0};  // m
        double yaw{0.0}; // rad
        bool check_yaw{false};
    };

    // ---------------------------------------------------------------------------
    // GoalChecker: 对齐 nav2_core::GoalChecker。有状态:位置一旦到达就保持,之后
    // 只再看朝向(原地转向时的微小漂移不会让判定倒退)。
    // ---------------------------------------------------------------------------
    class GoalChecker
    {
    public:
        GoalChecker() = default;
        virtual ~GoalChecker() = default;
        GoalChecker(const GoalChecker&) = default;
        GoalChecker& operator=(const GoalChecker&) = default;
        GoalChecker(GoalChecker&&) = default;
        GoalChecker& operator=(GoalChecker&&) = default;

        virtual void reset() = 0;
        [[nodiscard]] virtual bool is_goal_reached(const Pose2D& pose, const Pose2D& goal,
                                                   const Twist2D& velocity) = 0;
        [[nodiscard]] virtual GoalTolerance tolerances() const = 0;
    };

    // ---------------------------------------------------------------------------
    // ProgressChecker: 对齐 nav2_core::ProgressChecker —— 时间窗内位移不足即判"卡住"。
    // 显式传入仿真时间 t(而不是读墙钟),保证闭环仿真逐位确定。
    // ---------------------------------------------------------------------------
    class ProgressChecker
    {
    public:
        ProgressChecker() = default;
        virtual ~ProgressChecker() = default;
        ProgressChecker(const ProgressChecker&) = default;
        ProgressChecker& operator=(const ProgressChecker&) = default;
        ProgressChecker(ProgressChecker&&) = default;
        ProgressChecker& operator=(ProgressChecker&&) = default;

        virtual void reset() = 0;
        // true = 仍在推进;false = 卡住。
        [[nodiscard]] virtual bool check(const Pose2D& pose, double t) = 0;
    };

    // ---------------------------------------------------------------------------
    // Controller: 对齐 nav2_core::Controller(不含 ROS 类型)。
    //
    // 只认 Path / Pose2D / Twist2D —— 不认识栅格、EKF、传感器,所以控制库只依赖
    // core。compute_velocity_commands 会推进内部进度(路径上的当前段),故非 const。
    // goal_checker 可为空;非空时控制器用它的容差决定何时原地转到目标朝向。
    // ---------------------------------------------------------------------------
    class Controller
    {
    public:
        Controller() = default;
        virtual ~Controller() = default;
        Controller(const Controller&) = default;
        Controller& operator=(const Controller&) = default;
        Controller(Controller&&) = default;
        Controller& operator=(Controller&&) = default;

        virtual void set_plan(const Path& path) = 0;
        [[nodiscard]] virtual Twist2D compute_velocity_commands(const Pose2D& pose,
                                                                const Twist2D& velocity,
                                                                const GoalChecker* goal_checker) = 0;
        // Nav2 语义:limit ≤ 0 取消限速;percentage = true 时 limit 是期望速度的百分比,
        // 否则是绝对线速度上限 [m/s]。
        virtual void set_speed_limit(double limit, bool percentage) = 0;
        virtual void reset() = 0;
    };
}
