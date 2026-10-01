import mininav.control.velocity_smoother;
import mininav.core.types;

#include <gtest/gtest.h>

#include <stdexcept>

namespace {

using mininav::Twist2D;
using mininav::control::VelocitySmoother;
using mininav::control::VelocitySmootherConfig;

constexpr double kEps = 1e-12;
constexpr double kDt = 0.05;  // 20 Hz

}  // namespace

TEST(VelocitySmoother, SmallChangesPassThrough) {
  VelocitySmoother smoother{VelocitySmootherConfig{}};
  smoother.reset(Twist2D{0.2, 0.1});
  const Twist2D out = smoother.smooth(Twist2D{0.21, 0.2}, kDt);  // Δv = 0.01 ≤ 0.025, Δω = 0.1 ≤ 0.15
  EXPECT_NEAR(out.v(), 0.21, kEps);
  EXPECT_NEAR(out.w(), 0.2, kEps);
}

// 0 → 0.3 m/s,a = 0.5 m/s²:每拍 +0.025,12 拍到位。
TEST(VelocitySmoother, LimitsLinearAcceleration) {
  VelocitySmoother smoother{VelocitySmootherConfig{}};
  Twist2D out{};
  for (int k = 1; k <= 12; ++k) {
    out = smoother.smooth(Twist2D{0.3, 0.0}, kDt);
    EXPECT_NEAR(out.v(), 0.025 * k, 1e-12) << "tick " << k;
  }
  out = smoother.smooth(Twist2D{0.3, 0.0}, kDt);
  EXPECT_NEAR(out.v(), 0.3, kEps);
}

TEST(VelocitySmoother, LimitsDecelerationSymmetrically) {
  VelocitySmoother smoother{VelocitySmootherConfig{}};
  smoother.reset(Twist2D{0.3, 0.0});
  const Twist2D out = smoother.smooth(Twist2D{0.0, 0.0}, kDt);
  EXPECT_NEAR(out.v(), 0.275, kEps);
}

TEST(VelocitySmoother, LimitsAngularAcceleration) {
  VelocitySmoother smoother{VelocitySmootherConfig{}};
  const Twist2D out = smoother.smooth(Twist2D{0.0, 1.0}, kDt);  // α = 3 rad/s² → 0.15 每拍
  EXPECT_NEAR(out.v(), 0.0, kEps);
  EXPECT_NEAR(out.w(), 0.15, kEps);
}

// 两个分量按同一比例收缩:从静止起步时输出曲率 ω/v 与目标一致。
// 目标 (0.3, 0.6):线速度要 12 拍、角速度要 4 拍,取较紧的 1/12。
TEST(VelocitySmoother, ScalesBothComponentsTogetherPreservingCurvatureFromRest) {
  VelocitySmoother smoother{VelocitySmootherConfig{}};
  const Twist2D out = smoother.smooth(Twist2D{0.3, 0.6}, kDt);
  EXPECT_NEAR(out.v(), 0.025, kEps);
  EXPECT_NEAR(out.w(), 0.05, kEps);
  EXPECT_NEAR(out.w() / out.v(), 2.0, 1e-12);
}

TEST(VelocitySmoother, ResetSetsBaseline) {
  VelocitySmoother smoother{VelocitySmootherConfig{}};
  smoother.reset(Twist2D{0.25, -0.5});
  EXPECT_EQ(smoother.last().v(), 0.25);
  EXPECT_EQ(smoother.last().w(), -0.5);
}

TEST(VelocitySmoother, RejectsNonPositiveLimits) {
  EXPECT_THROW((VelocitySmoother{VelocitySmootherConfig{.max_accel = 0.0, .max_angular_accel = 3.0}}),
               std::invalid_argument);
  EXPECT_THROW((VelocitySmoother{VelocitySmootherConfig{.max_accel = 0.5, .max_angular_accel = -1.0}}),
               std::invalid_argument);
}
