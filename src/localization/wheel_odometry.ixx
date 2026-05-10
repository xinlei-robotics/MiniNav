module;

export module mininav.localization.wheel_odometry;

import mininav.core.types;
import mininav.core.math;

export namespace mininav
{
    struct WheelOdometryParams
    {
        double wheel_base{0.150};
        double distance_per_tick{2.0 * kPi * 0.032 / 1024.0};
    };

    class WheelOdometry
    {
    public:
        explicit WheelOdometry(WheelOdometryParams params, Pose2D initial_pose = {}) noexcept;
        Pose2D update(const EncoderTicks& dticks, double dt);

        [[nodiscard]] const Pose2D& current_estimate() const noexcept
        {
            return odom_pose_;
        }

        [[nodiscard]] const WheelOdometryParams& params() const noexcept
        {
            return params_;
        }

    private:
        WheelOdometryParams params_;
        Pose2D odom_pose_;
    };
}
