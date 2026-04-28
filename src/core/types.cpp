module;

#include <Eigen/Core>

module mininav.core.types;

namespace mininav
{
    Eigen::Vector3d Pose2D::to_vector() const noexcept
    {
        return Eigen::Vector3d{position_.x(), position_.y(), yaw_};
    }

    Pose2D Pose2D::from_vector(const Eigen::Vector3d& v) noexcept
    {
        return Pose2D{v.x(), v.y(), v.z()};
    }
}
