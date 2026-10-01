export module mininav.control.goal_checker;

import mininav.core.types;
import mininav.control.controller;

export namespace mininav::control
{
    struct GoalCheckerConfig
    {
        double xy_tolerance{0.05};  // m
        double yaw_tolerance{0.10}; // rad
    };

    // 参数合法性检查;不合法抛 std::invalid_argument。
    void validate(const GoalCheckerConfig& cfg);

    // ---------------------------------------------------------------------------
    // SimpleGoalChecker: 对齐 Nav2 的 SimpleGoalChecker(stateful = true)。
    //   1. 位置进入 xy_tolerance 后记住"已到位",之后不再检查位置;
    //   2. check_yaw 时再要求 |wrap(goal.yaw − yaw)| ≤ yaw_tolerance。
    // check_yaw 由调用方按"目标是否给了朝向"决定(sim nav 的 --goal x,y[,yaw]),
    // 不是配置项。速度不参与判定。
    // ---------------------------------------------------------------------------
    class SimpleGoalChecker final : public GoalChecker
    {
    public:
        SimpleGoalChecker(const GoalCheckerConfig& cfg, bool check_yaw);

        void reset() override { position_reached_ = false; }
        [[nodiscard]] bool is_goal_reached(const Pose2D& pose, const Pose2D& goal,
                                           const Twist2D& velocity) override;
        [[nodiscard]] GoalTolerance tolerances() const override;

    private:
        GoalCheckerConfig cfg_;
        bool check_yaw_;
        bool position_reached_{false};
    };
}
