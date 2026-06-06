module;

#include <utility>

export module mininav.core.kinematics;

import mininav.core.types;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // differential_drive_step: 差分驱动机器人的 RK4 积分入口。
    //
    // Kept as the stable public kinematics API; internally delegates to rk4_step.
    // ---------------------------------------------------------------------------
    [[nodiscard]] Pose2D differential_drive_step(const Pose2D& current, const Twist2D& control, double dt) noexcept;

    [[nodiscard]] std::pair<double, double> inverse_kinematics(const Twist2D& body, double wheel_base) noexcept;

    [[nodiscard]] Twist2D forward_kinematics(double v_left, double v_right, double wheel_base) noexcept;
}
