export module mininav.core.robot_model;

import mininav.core.types;

export namespace mininav {

class RobotModel {
public:
  [[nodiscard]] Pose2D step(const Pose2D &current, const Twist2D &control,
                            double dt) const noexcept;
};

} // namespace mininav