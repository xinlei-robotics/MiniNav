// ---------------------------------------------------------------------------
// StopController: 工具链检查用的 nav2_core::Controller 插件。
//
// 不跟踪路径,只把当前速度经 mininav::control::VelocitySmoother 按加速度上限刹向零——
// 足以证明 GCC 编译的 controller_server 能通过 pluginlib 加载 Clang 编译、import 了
// MiniNav 模块的插件,并真正调用静态库里的代码。控制周期取 controller_server 的
// controller_frequency。
// ---------------------------------------------------------------------------

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_core/controller.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <memory>
#include <stdexcept>
#include <string>

import mininav.core.types;
import mininav.control.velocity_smoother;

namespace mininav_toolchain_check
{
    class StopController : public nav2_core::Controller
    {
    public:
        void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                       std::shared_ptr<tf2_ros::Buffer> /*tf*/,
                       std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/) override
        {
            const auto node = parent.lock();
            if (!node)
            {
                throw std::runtime_error("StopController: parent node is gone");
            }
            name_ = std::move(name);

            const double frequency = node->get_parameter_or("controller_frequency", 20.0);
            if (!(frequency > 0.0))
            {
                throw std::invalid_argument("StopController: controller_frequency must be > 0");
            }
            dt_ = 1.0 / frequency;
        }

        void cleanup() override {}
        void activate() override {}
        void deactivate() override {}
        void setPlan(const nav_msgs::msg::Path& /*path*/) override {}

        geometry_msgs::msg::TwistStamped computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity,
            nav2_core::GoalChecker* /*goal_checker*/) override
        {
            smoother_.reset(mininav::Twist2D{velocity.linear.x, velocity.angular.z});
            const mininav::Twist2D cmd = smoother_.smooth(mininav::Twist2D{}, dt_);

            geometry_msgs::msg::TwistStamped out;
            out.header = pose.header;
            out.twist.linear.x = cmd.v();
            out.twist.angular.z = cmd.w();
            return out;
        }

        void setSpeedLimit(const double& /*speed_limit*/, const bool& /*percentage*/) override {}

    private:
        std::string name_;
        double dt_{0.05};
        mininav::control::VelocitySmoother smoother_{mininav::control::VelocitySmootherConfig{}};
    };
} // namespace mininav_toolchain_check

PLUGINLIB_EXPORT_CLASS(mininav_toolchain_check::StopController, nav2_core::Controller)
