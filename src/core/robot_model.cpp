module mininav.core.robot_model;

import mininav.core.types;
import mininav.core.kinematics;

namespace mininav
{
    Pose2D RobotModel::step(const Pose2D &current,
                        const Twist2D &control,
                        const double dt) const noexcept
    {
        return differential_drive_step(current, control, dt);
    }
}
