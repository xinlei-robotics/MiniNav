module;

#include <cassert>
#include <cmath>

module mininav.core.kinematics;

import mininav.core.types;
import mininav.core.math;

namespace mininav
{
    Pose2D differential_drive_step(const Pose2D& current, const Twist2D& control, double dt) noexcept
    {
        assert(dt > 0.0 && "differential_drive_step requires a strictly positive dt");

        const double yaw = current.yaw();
        const double v = control.v();
        const double w = control.w();

        const double next_x = current.x() + v * std::cos(yaw) * dt;
        const double next_y = current.y() + v * std::sin(yaw) * dt;
        const double next_yaw = wrap_angle(yaw + w * dt);

        return Pose2D{next_x, next_y, next_yaw};
    }

    std::pair<double, double> inverse_kinematics(const Twist2D& body, const double wheel_base) noexcept
    {
        const double half_L_omega = 0.5 * body.w() * wheel_base;

        const double v_left = body.v() - half_L_omega;
        const double v_right = body.v() + half_L_omega;

        return {v_left, v_right};
    }

    Twist2D forward_kinematics(const double v_left, const double v_right, const double wheel_base) noexcept
    {
        const double v = 0.5 * (v_left + v_right);
        const double w = (v_right - v_left) / wheel_base;
        return Twist2D{v, w};
    }
}
