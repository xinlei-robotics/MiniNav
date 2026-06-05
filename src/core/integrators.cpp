module;

#include <cassert>
#include <cmath>

module mininav.core.integrators;

import mininav.core.math;
import mininav.core.types;

namespace mininav
{
    namespace
    {
        struct PoseDerivative
        {
            double x_dot;
            double y_dot;
            double yaw_dot;
        };

        [[nodiscard]] PoseDerivative derivative_at(const Pose2D& pose, const Twist2D& control) noexcept
        {
            return PoseDerivative{
                .x_dot = control.v() * std::cos(pose.yaw()),
                .y_dot = control.v() * std::sin(pose.yaw()),
                .yaw_dot = control.w(),
            };
        }

        [[nodiscard]] Pose2D add_scaled(
            const Pose2D& pose,
            const PoseDerivative& d,
            const double scale) noexcept
        {
            return Pose2D{
                pose.x() + scale * d.x_dot,
                pose.y() + scale * d.y_dot,
                pose.yaw() + scale * d.yaw_dot,
            };
        }
    }

    Pose2D rk4_step(const Pose2D& current, const Twist2D& control, const double dt) noexcept
    {
        assert(dt > 0.0 && "rk4_step requires a strictly positive dt");

        const PoseDerivative k1 = derivative_at(current, control);
        const PoseDerivative k2 = derivative_at(add_scaled(current, k1, 0.5 * dt), control);
        const PoseDerivative k3 = derivative_at(add_scaled(current, k2, 0.5 * dt), control);
        const PoseDerivative k4 = derivative_at(add_scaled(current, k3, dt), control);

        const double next_x = current.x()
            + (dt / 6.0) * (k1.x_dot + 2.0 * k2.x_dot + 2.0 * k3.x_dot + k4.x_dot);
        const double next_y = current.y()
            + (dt / 6.0) * (k1.y_dot + 2.0 * k2.y_dot + 2.0 * k3.y_dot + k4.y_dot);
        const double next_yaw = wrap_angle(current.yaw()
            + (dt / 6.0) * (k1.yaw_dot + 2.0 * k2.yaw_dot + 2.0 * k3.yaw_dot + k4.yaw_dot));

        return Pose2D{next_x, next_y, next_yaw};
    }
}
