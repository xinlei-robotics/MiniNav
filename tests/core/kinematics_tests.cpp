import mininav.core.kinematics;
import mininav.core.math;
import mininav.core.types;

#include <gtest/gtest.h>

#include <cmath>

namespace
{
    constexpr double kEps = 1e-9;
}

TEST(DifferentialDriveStep, StraightLineAlongX)
{
    const mininav::Pose2D start{0.0, 0.0, 0.0};
    const mininav::Twist2D cmd{1.0, 0.0};

    const auto next = mininav::differential_drive_step(start, cmd, 1.0);
    EXPECT_NEAR(next.x(), 1.0, kEps);
    EXPECT_NEAR(next.y(), 0.0, kEps);
    EXPECT_NEAR(next.yaw(), 0.0, kEps);
}

TEST(DifferentialDriveStep, StraightLineWithSmallDt)
{
    const mininav::Pose2D start{0.0, 0.0, 0.0};
    const mininav::Twist2D cmd{2.0, 0.0};

    const auto next = mininav::differential_drive_step(start, cmd, 0.01);
    EXPECT_NEAR(next.x(), 0.02, kEps);
    EXPECT_NEAR(next.y(), 0.0, kEps);
}

TEST(DifferentialDriveStep, PureRotationDoesNotMoveXY)
{
    const mininav::Pose2D start{1.0, 2.0, 0.0};
    const mininav::Twist2D cmd{0.0, mininav::kPi / 2.0};

    const auto next = mininav::differential_drive_step(start, cmd, 1.0);
    EXPECT_NEAR(next.x(), 1.0, kEps);
    EXPECT_NEAR(next.y(), 2.0, kEps);
    EXPECT_NEAR(next.yaw(), mininav::kPi / 2.0, kEps);
}

TEST(DifferentialDriveStep, MoveAlong45Degrees)
{
    const mininav::Pose2D start{0.0, 0.0, mininav::kPi / 4.0};
    const mininav::Twist2D cmd{std::sqrt(2.0), 0.0};

    const auto next = mininav::differential_drive_step(start, cmd, 1.0);
    EXPECT_NEAR(next.x(), 1.0, kEps);
    EXPECT_NEAR(next.y(), 1.0, kEps);
}

TEST(DifferentialDriveStep, YawWrapsAcrossPositivePi)
{
    const mininav::Pose2D start{0.0, 0.0, mininav::kPi - 0.01};
    const mininav::Twist2D cmd{0.0, 0.1};

    const auto next = mininav::differential_drive_step(start, cmd, 1.0);
    // 原始 yaw 累加为 pi + 0.09,应被 wrap 到 -pi + 0.09
    EXPECT_NEAR(next.yaw(), -mininav::kPi + 0.09, kEps);
}

TEST(DifferentialDriveStep, BackwardMotion)
{
    const mininav::Pose2D start{5.0, 0.0, 0.0};
    const mininav::Twist2D cmd{-1.0, 0.0};

    const auto next = mininav::differential_drive_step(start, cmd, 1.0);
    EXPECT_NEAR(next.x(), 4.0, kEps);
}

TEST(DifferentialDriveStep, ZeroCommandKeepsPose)
{
    const mininav::Pose2D start{1.0, 2.0, 0.5};
    const mininav::Twist2D cmd{0.0, 0.0};

    const auto next = mininav::differential_drive_step(start, cmd, 0.1);
    EXPECT_NEAR(next.x(), 1.0, kEps);
    EXPECT_NEAR(next.y(), 2.0, kEps);
    EXPECT_NEAR(next.yaw(), 0.5, kEps);
}

TEST(DifferentialDriveStep, MultiStepConsistency)
{
    // 100 步 dt=0.01 的直线运动应等价于 1 步 dt=1.0(因为是欧拉积分,
    // 直线场景下没有非线性误差)
    mininav::Pose2D pose{0.0, 0.0, 0.0};
    const mininav::Twist2D cmd{1.0, 0.0};
    for (int i = 0; i < 100; ++i)
    {
        pose = mininav::differential_drive_step(pose, cmd, 0.01);
    }
    EXPECT_NEAR(pose.x(), 1.0, kEps);
    EXPECT_NEAR(pose.y(), 0.0, kEps);
}

TEST(KinematicsRoundTrip, InverseThenForwardIsIdentity) {
    constexpr double L = 0.35;
    for (double v : {-2.0, -0.3, 0.0, 0.7, 1.5}) {
        for (double w : {-3.0, -0.5, 0.0, 0.4, 2.1}) {
            mininav::Twist2D in{v, w};
            auto [vl, vr] = mininav::inverse_kinematics(in, L);
            mininav::Twist2D out = mininav::forward_kinematics(vl, vr, L);
            EXPECT_NEAR(out.v(), in.v(), kEps);
            EXPECT_NEAR(out.w(), in.w(), kEps);
        }
    }
}

TEST(InverseKinematics, StraightDriveEqualWheels) {
    mininav::Twist2D body{1.0, 0.0};
    auto [vl, vr] = mininav::inverse_kinematics(body, 0.3);
    EXPECT_NEAR(vl, 1.0, kEps);
    EXPECT_NEAR(vr, 1.0, kEps);
}

TEST(InverseKinematics, InPlaceRotationOppositeWheels) {
    mininav::Twist2D body{0.0, mininav::kPi / 2.0};
    auto [vl, vr] = mininav::inverse_kinematics(body, 0.3);
    EXPECT_NEAR(vl, -0.075 * mininav::kPi, kEps);
    EXPECT_NEAR(vr,  0.075 * mininav::kPi, kEps);
    EXPECT_NEAR(vl, -vr, kEps);
}

TEST(ForwardKinematics, RecoversTwistFromWheelSpeeds) {
    auto t = mininav::forward_kinematics(0.5, 1.5, /*L=*/0.4);
    EXPECT_NEAR(t.v(), 1.0, kEps);
    EXPECT_NEAR(t.w(), 2.5, kEps);
}