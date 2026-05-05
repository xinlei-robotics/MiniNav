import mininav.core.types;

#include <gtest/gtest.h>

#include <Eigen/Core>

namespace {

constexpr double kEps = 1e-12;

} // namespace

// ---------------------------------------------------------------------------
// Pose2D
// ---------------------------------------------------------------------------

TEST(Pose2D, DefaultConstructionIsZero) {
  const mininav::Pose2D pose;
  EXPECT_NEAR(pose.x(), 0.0, kEps);
  EXPECT_NEAR(pose.y(), 0.0, kEps);
  EXPECT_NEAR(pose.yaw(), 0.0, kEps);
}

TEST(Pose2D, ScalarConstructor) {
  const mininav::Pose2D pose{1.5, -2.5, 0.3};
  EXPECT_NEAR(pose.x(), 1.5, kEps);
  EXPECT_NEAR(pose.y(), -2.5, kEps);
  EXPECT_NEAR(pose.yaw(), 0.3, kEps);
}

TEST(Pose2D, VectorConstructor) {
  const Eigen::Vector2d position{3.0, 4.0};
  const mininav::Pose2D pose{position, 1.0};
  EXPECT_NEAR(pose.x(), 3.0, kEps);
  EXPECT_NEAR(pose.y(), 4.0, kEps);
  EXPECT_NEAR(pose.yaw(), 1.0, kEps);
  EXPECT_TRUE(pose.position().isApprox(position));
}

TEST(Pose2D, Setters) {
  mininav::Pose2D pose;
  pose.set_x(7.0);
  pose.set_y(-3.0);
  pose.set_yaw(0.5);
  EXPECT_NEAR(pose.x(), 7.0, kEps);
  EXPECT_NEAR(pose.y(), -3.0, kEps);
  EXPECT_NEAR(pose.yaw(), 0.5, kEps);

  pose.set_position(Eigen::Vector2d{10.0, 20.0});
  EXPECT_NEAR(pose.x(), 10.0, kEps);
  EXPECT_NEAR(pose.y(), 20.0, kEps);
}

TEST(Pose2D, ToVectorOrderIsXYYaw) {
  const mininav::Pose2D pose{1.5, -2.5, 0.3};
  const Eigen::Vector3d v = pose.to_vector();
  EXPECT_NEAR(v.x(), 1.5, kEps);
  EXPECT_NEAR(v.y(), -2.5, kEps);
  EXPECT_NEAR(v.z(), 0.3, kEps);
}

TEST(Pose2D, FromVectorRoundTrip) {
  const Eigen::Vector3d v{1.5, -2.5, 0.3};
  const auto pose = mininav::Pose2D::from_vector(v);
  EXPECT_TRUE(pose.to_vector().isApprox(v));
}

// ---------------------------------------------------------------------------
// Twist2D
// ---------------------------------------------------------------------------

TEST(Twist2D, DefaultConstructionIsZero) {
  const mininav::Twist2D twist;
  EXPECT_NEAR(twist.v(), 0.0, kEps);
  EXPECT_NEAR(twist.w(), 0.0, kEps);
}

TEST(Twist2D, ScalarConstructor) {
  const mininav::Twist2D twist{2.0, -1.5};
  EXPECT_NEAR(twist.v(), 2.0, kEps);
  EXPECT_NEAR(twist.w(), -1.5, kEps);
}

TEST(Twist2D, Setters) {
  mininav::Twist2D twist;
  twist.set_v(3.14);
  twist.set_w(-2.71);
  EXPECT_NEAR(twist.v(), 3.14, kEps);
  EXPECT_NEAR(twist.w(), -2.71, kEps);
}

// ---------------------------------------------------------------------------
// SimStateV0
// ---------------------------------------------------------------------------

TEST(SimStateV0, AggregateInitialization) {
  const mininav::SimStateV0 state{
      1.0,
      mininav::Pose2D{2.0, 3.0, 0.5},
      mininav::Twist2D{1.0, 0.1}};
  EXPECT_NEAR(state.t, 1.0, kEps);
  EXPECT_NEAR(state.pose.x(), 2.0, kEps);
  EXPECT_NEAR(state.pose.y(), 3.0, kEps);
  EXPECT_NEAR(state.pose.yaw(), 0.5, kEps);
  EXPECT_NEAR(state.cmd.v(), 1.0, kEps);
  EXPECT_NEAR(state.cmd.w(), 0.1, kEps);
}

TEST(SimStateV0, DefaultConstructionIsZero) {
  const mininav::SimStateV0 state{};
  EXPECT_NEAR(state.t, 0.0, kEps);
  EXPECT_NEAR(state.pose.x(), 0.0, kEps);
  EXPECT_NEAR(state.cmd.v(), 0.0, kEps);
}

// ---------------------------------------------------------------------------
// SimStateV1
// ---------------------------------------------------------------------------

TEST(EncoderTicks, DefaultConstructed) {
  mininav::EncoderTicks t;
  EXPECT_EQ(t.left,  0);
  EXPECT_EQ(t.right, 0);
}

TEST(EncoderTicks, AggregateInit) {
  mininav::EncoderTicks t{.left = 100, .right = -50};
  EXPECT_EQ(t.left,   100);
  EXPECT_EQ(t.right, -50);
}

TEST(SimStateV1, DefaultIsWellFormed) {
  mininav::SimStateV1 s{};
  EXPECT_DOUBLE_EQ(s.t, 0.0);
  EXPECT_DOUBLE_EQ(s.cmd.v(), 0.0);
  EXPECT_DOUBLE_EQ(s.cmd.w(), 0.0);
  EXPECT_EQ(s.enc_dticks.left, 0);
  EXPECT_EQ(s.enc_dticks.right, 0);
  EXPECT_DOUBLE_EQ(s.truth_pose.yaw(), 0.0);
  EXPECT_DOUBLE_EQ(s.odom_pose.yaw(),  0.0);
}