// pluginlib(GCC 编译)加载 Clang 编译的 StopController,其指令与直接调用库逐位相同。

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_core/controller.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <memory>

import mininav.core.types;
import mininav.control.velocity_smoother;

namespace
{
    class StopControllerTest : public ::testing::Test
    {
    protected:
        static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
        static void TearDownTestSuite() { rclcpp::shutdown(); }
    };

    TEST_F(StopControllerTest, LoadsThroughPluginlibAndMatchesTheLibraryBitForBit)
    {
        pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
        const std::shared_ptr<nav2_core::Controller> controller =
            loader.createSharedInstance("mininav_toolchain_check::StopController");
        ASSERT_NE(controller, nullptr);

        const auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("stop_controller_test");
        node->declare_parameter("controller_frequency", 20.0);
        controller->configure(node, "FollowPath", nullptr, nullptr);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "odom";
        pose.header.stamp.sec = 42;
        geometry_msgs::msg::Twist velocity;
        velocity.linear.x = 0.3;
        velocity.angular.z = -0.7;

        const geometry_msgs::msg::TwistStamped cmd =
            controller->computeVelocityCommands(pose, velocity, nullptr);

        mininav::control::VelocitySmoother smoother{mininav::control::VelocitySmootherConfig{}};
        smoother.reset(mininav::Twist2D{0.3, -0.7});
        const mininav::Twist2D expected = smoother.smooth(mininav::Twist2D{}, 1.0 / 20.0);

        // 同一份库代码、同一组输入:逐位相同,不是近似相等。
        EXPECT_EQ(cmd.twist.linear.x, expected.v());
        EXPECT_EQ(cmd.twist.angular.z, expected.w());
        // 确实在刹车:速度变小但一拍之内到不了零。
        EXPECT_GT(cmd.twist.linear.x, 0.0);
        EXPECT_LT(cmd.twist.linear.x, 0.3);
        EXPECT_EQ(cmd.header.frame_id, "odom");
        EXPECT_EQ(cmd.header.stamp.sec, 42);
    }
} // namespace
