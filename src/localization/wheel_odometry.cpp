module;

module mininav.localization.wheel_odometry;

import mininav.core.types;
import mininav.core.kinematics;

namespace mininav
{
    WheelOdometry::WheelOdometry(WheelOdometryParams params, Pose2D initial_pose) noexcept
        : params_{params}, odom_pose_{initial_pose}
    {
    }

    Pose2D WheelOdometry::update(const EncoderTicks& dticks, double dt)
    {
        const Twist2D twist =
            twist_from_ticks(dticks, params_.distance_per_tick, params_.wheel_base, dt);
        odom_pose_ = differential_drive_step(odom_pose_, twist, dt);

        return odom_pose_;
    }
}
