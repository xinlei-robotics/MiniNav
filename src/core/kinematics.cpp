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
}
