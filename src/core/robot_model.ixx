export module mininav.core.robot_model;

import mininav.core.types;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // RobotModel: 二维差分驱动机器人的运动学积分器
    // ---------------------------------------------------------------------------
    class RobotModel
    {
    public:
        [[nodiscard]] static Pose2D step(const Pose2D& current,
                                         const Twist2D& control,
                                         double dt) noexcept;
    };
} // namespace mininav
