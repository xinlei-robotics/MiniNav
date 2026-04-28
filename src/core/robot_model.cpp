module;

#include <cmath>
#include <cassert>

module mininav.core.robot_model;

import mininav.core.types;
import mininav.core.math;

namespace mininav
{
    Pose2D RobotModel::step(const Pose2D& current, const Twist2D& control,
                            const double dt) noexcept
    {
        assert(dt > 0.0 && "RobotModel::step requires a strictly positive dt");
        const double yaw = current.yaw();
        const double v = control.v();
        const double w = control.w();

        const double next_x = current.x() + v * std::cos(yaw) * dt;
        const double next_y = current.y() + v * std::sin(yaw) * dt;
        const double next_yaw = wrap_angle(yaw + w * dt);

        return {next_x, next_y, next_yaw};
    }
} // namespace mininav
