module;

#include <cmath>

module mininav.core.robot_model;

import mininav.core.types;
import mininav.core.math;

namespace mininav {
    Pose2D RobotModel::step(const Pose2D& current, const Twist2D& control, double dt) const noexcept {
        Pose2D next = current;
        next.x += control.v * std::cos(current.yaw) * dt;
        next.y += control.v * std::sin(current.yaw) * dt;
        next.yaw = wrap_angle(current.yaw + control.w * dt);
        return next;
    }

}