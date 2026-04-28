export module mininav.core.kinematics;

import mininav.core.types;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // differential_drive_step: 差分驱动机器人的一阶欧拉积分
    //
    // 方程:
    //   x_{k+1}   = x_k   + v * cos(yaw_k) * dt
    //   y_{k+1}   = y_k   + v * sin(yaw_k) * dt
    //   yaw_{k+1} = wrap_angle(yaw_k + w * dt)
    //
    // 这是一个纯函数(无副作用、无内部状态),作为运动学的"事实参考"。
    // 任何带状态的运动学组件(RobotModel、未来的 NoisyRobotModel、
    // 实车驱动的反向 odom 计算)都应当复用这个函数,以保证全项目的
    // 运动学方程在数值上完全一致。
    //
    // 前置条件:
    //   - dt > 0
    // ---------------------------------------------------------------------------
    [[nodiscard]] Pose2D differential_drive_step(const Pose2D& current, const Twist2D& control, double dt) noexcept;
}
