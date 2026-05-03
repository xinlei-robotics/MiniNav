export module mininav.core.robot_model;

import mininav.core.types;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // RobotModel: 仿真用的二维差分驱动机器人
    //
    // V0 阶段:无状态、纯运动学积分。step 直接委托给
    //          mininav.core.kinematics 中的 differential_drive_step。
    //
    // 未来扩展:
    //   - V1: 派生 NoisyRobotModel,在 step 内对控制量叠加噪声
    //   - V6: 派生 RealCarModel,step 转化为底盘驱动指令
    //
    // 当前用 class 包装而非自由函数,是为了未来这些扩展。
    // ---------------------------------------------------------------------------
    class RobotModel
    {
    public:
        RobotModel() = default;
        virtual ~RobotModel() = default;

        RobotModel(const RobotModel&) = delete;
        RobotModel& operator=(const RobotModel&) = delete;
        RobotModel(RobotModel&&) = delete;
        RobotModel& operator=(RobotModel&&) = delete;

        [[nodiscard]] virtual Pose2D step(const Pose2D& current, const Twist2D& control, double dt) const noexcept;
    };
}
