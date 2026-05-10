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
        
        const double dist_l = static_cast<double>(dticks.left) * params_.distance_per_tick;
        const double dist_r = static_cast<double>(dticks.right) * params_.distance_per_tick;

        const double v_l = dist_l / dt;
        const double v_r = dist_r / dt;

        const Twist2D twist = forward_kinematics(v_l, v_r, params_.wheel_base);
        odom_pose_ = differential_drive_step(odom_pose_, twist, dt);

        return odom_pose_;
    }
}
