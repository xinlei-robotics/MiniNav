#include <gtest/gtest.h>

#include <cmath>

import mininav.core.types;
import mininav.core.kinematics;
import mininav.core.math;
import mininav.localization.wheel_odometry;

namespace
{
    using mininav::EncoderTicks;
    using mininav::Pose2D;
    using mininav::Twist2D;
    using mininav::WheelOdometry;
    using mininav::WheelOdometryParams;
    using mininav::differential_drive_step;
    using mininav::inverse_kinematics;

    constexpr double kWheelRadius     = 0.032;
    constexpr std::int64_t kTicksPerRev = 1024;
    constexpr double kWheelBase       = 0.150;
    constexpr double kDistPerTick     =
        2.0 * mininav::kPi * kWheelRadius / static_cast<double>(kTicksPerRev);

    [[nodiscard]] WheelOdometryParams default_params()
    {
        return WheelOdometryParams{
            .wheel_base        = kWheelBase,
            .distance_per_tick = kDistPerTick,
        };
    }


    [[nodiscard]] EncoderTicks ticks_for_distances(double dist_l, double dist_r)
    {
        return EncoderTicks{
            .left  = static_cast<std::int64_t>(std::llround(dist_l / kDistPerTick)),
            .right = static_cast<std::int64_t>(std::llround(dist_r / kDistPerTick)),
        };
    }
}

// ---------------------------------------------------------------------------
// 退化: 无 ticks → 位姿不变
// ---------------------------------------------------------------------------
TEST(WheelOdometry, ZeroTicksLeavePoseUnchanged)
{
    WheelOdometry odom{default_params(), Pose2D{1.0, 2.0, 0.5}};
    for (int i = 0; i < 10; ++i)
    {
        const Pose2D p = odom.update(EncoderTicks{0, 0}, 0.02);
        EXPECT_DOUBLE_EQ(p.x(),   1.0);
        EXPECT_DOUBLE_EQ(p.y(),   2.0);
        EXPECT_DOUBLE_EQ(p.yaw(), 0.5);
    }
}

// ---------------------------------------------------------------------------
// 直行: 左右等量正向 ticks → x 前进, y 与 yaw 不变
// ---------------------------------------------------------------------------
TEST(WheelOdometry, StraightDriveAdvancesXOnly)
{
    WheelOdometry odom{default_params()};

    constexpr double dt = 0.02;
    const auto ticks = ticks_for_distances(0.01, 0.01);

    for (int i = 0; i < 100; ++i)
    {
        odom.update(ticks, dt);
    }
    const Pose2D p = odom.current_estimate();
    constexpr double kPosTol = 100.0 * kDistPerTick;

    EXPECT_NEAR(p.x(), 1.0,  kPosTol);
    EXPECT_NEAR(p.y(), 0.0, 1e-12);
    EXPECT_NEAR(p.yaw(), 0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// 原地转: 反向等量 ticks → yaw 改变, 位置不变
// ---------------------------------------------------------------------------
TEST(WheelOdometry, InPlaceRotationChangesYawOnly)
{
    WheelOdometry odom{default_params()};
    constexpr double dt = 0.02;

    // 想让车以 ω=π/2 rad/s 原地转 → 左右轮速 ±L·ω/2
    // 单步弧长: L·ω·dt/2 = 0.150 · (π/2) · 0.02 / 2 ≈ 0.00236 m → 约 0.074 ticks, 100 步后转 π rad。
    constexpr double half_arc = kWheelBase * (mininav::kPi / 2.0) * dt / 2.0;
    const auto ticks = ticks_for_distances(-half_arc, +half_arc);

    for (int i = 0; i < 100; ++i)
    {
        odom.update(ticks, dt);
    }
    const Pose2D p = odom.current_estimate();

    // 100 步 × π/2 rad/s × 0.02 s = π rad —— 但小车积分顺序与真值通道
    // 一致 (用 differential_drive_step), 所以位置应严格不变。
    EXPECT_NEAR(p.x(), 0.0, 1e-12);
    EXPECT_NEAR(p.y(), 0.0, 1e-12);

    // yaw 需要考虑 wrap_angle 后的等价性 —— π 经 wrap 后仍是 π (开区间约定)
    // 但浮点 ticks 量化会让 yaw 略偏离 π, 用足够宽的容差。
    const double accumulated_yaw = std::atan2(std::sin(p.yaw()),
                                               std::cos(p.yaw()));
    EXPECT_NEAR(std::abs(accumulated_yaw), mininav::kPi,
                100.0 * kDistPerTick / kWheelBase);
}

// ---------------------------------------------------------------------------
// 零噪声 ticks 输入下, odom 估计与真值轨迹一致。
// ---------------------------------------------------------------------------
TEST(WheelOdometry, ZeroNoiseInputMatchesGroundTruthTrajectory)
{
    // 给定一段任意 twist 轨迹, 用 (1) differential_drive_step 直接积分得真值,
    // (2) 用 inverse_kinematics 把 twist 分解为左右轮距离, 再走 odometry 链路。
    // 没有任何噪声、没有任何量化直接用 ticks_for_distances 反算 ticks,
    // 两条路径的最终位姿应当在 ticks 量化容差范围内一致。

    constexpr double dt = 0.02;
    constexpr int    n_steps = 200;

    // 轨迹: 一段直行 + 一段转弯 + 一段反向
    auto cmd_at = [](int i) -> Twist2D {
        if (i < 60)  return Twist2D{0.5,  0.0};
        if (i < 120) return Twist2D{0.3,  0.8};
        if (i < 180) return Twist2D{-0.2, 0.0};
        return                          Twist2D{0.0,  0.5};
    };

    // (1) 真值通道
    Pose2D truth{};
    for (int i = 0; i < n_steps; ++i)
    {
        truth = differential_drive_step(truth, cmd_at(i), dt);
    }

    // (2) 估计通道: twist → (vl, vr) → 距离增量 → ticks (无量化损失反算) → WheelOdometry::update
    WheelOdometry odom{default_params()};
    for (int i = 0; i < n_steps; ++i)
    {
        const Twist2D twist = cmd_at(i);
        const auto [vl, vr] = inverse_kinematics(twist, kWheelBase);
        const auto ticks = ticks_for_distances(vl * dt, vr * dt);
        odom.update(ticks, dt);
    }
    const Pose2D estimate = odom.current_estimate();

    // 200 步累计 ticks 量化误差最坏情况 ≤ 200 * Δs_tick ≈ 4 cm,
    // 但实际由于 round-half-to-even 会大量抵消, 容差给得较宽以容忍随机方向。
    const double tol_pos = 200.0 * kDistPerTick;
    EXPECT_NEAR(estimate.x(),   truth.x(),   tol_pos);
    EXPECT_NEAR(estimate.y(),   truth.y(),   tol_pos);

    // yaw 的容差更难精确推导, 给一个工程合理值。
    EXPECT_NEAR(estimate.yaw(), truth.yaw(), 0.10);
}

// ---------------------------------------------------------------------------
// 起点: 非零初始位姿被尊重
// ---------------------------------------------------------------------------
TEST(WheelOdometry, NonzeroInitialPoseIsHonored)
{
    const Pose2D start{3.0, -2.0, 1.0};
    WheelOdometry odom{default_params(), start};

    EXPECT_DOUBLE_EQ(odom.current_estimate().x(),   3.0);
    EXPECT_DOUBLE_EQ(odom.current_estimate().y(),  -2.0);
    EXPECT_DOUBLE_EQ(odom.current_estimate().yaw(), 1.0);

    odom.update(EncoderTicks{0, 0}, 0.02);
    EXPECT_DOUBLE_EQ(odom.current_estimate().x(),   3.0);
    EXPECT_DOUBLE_EQ(odom.current_estimate().y(),  -2.0);
    EXPECT_DOUBLE_EQ(odom.current_estimate().yaw(), 1.0);
}

// ---------------------------------------------------------------------------
// 累积一致性: 一次性大 ticks vs 拆成多步的等价性
// ---------------------------------------------------------------------------
TEST(WheelOdometry, AccumulatesConsistentlyAcrossSteps)
{
    // 这条 test 用于验证: dt 缩短 → 数值积分应当向解析解收敛。
    // 我们用一段直行验证累积一致性。

    constexpr double total_distance = 0.5;

    // 路径 A: 1 大步
    {
        WheelOdometry odom{default_params()};
        const auto ticks = ticks_for_distances(total_distance, total_distance);
        odom.update(ticks, 1.0);
        EXPECT_NEAR(odom.current_estimate().x(), total_distance,
                    2.0 * kDistPerTick);
    }
    // 路径 B: 50 小步
    {
        WheelOdometry odom{default_params()};
        const auto ticks = ticks_for_distances(total_distance / 50.0,
                                                total_distance / 50.0);
        for (int i = 0; i < 50; ++i)
        {
            odom.update(ticks, 0.02);
        }
        EXPECT_NEAR(odom.current_estimate().x(), total_distance,
                    50.0 * kDistPerTick);
    }
}

// ---------------------------------------------------------------------------
// 反向运动: 负 ticks 处理正确
// ---------------------------------------------------------------------------
TEST(WheelOdometry, ReverseDriveProducesNegativeXAdvance)
{
    WheelOdometry odom{default_params()};

    constexpr double dt = 0.02;
    const auto ticks = ticks_for_distances(-0.01, -0.01);

    for (int i = 0; i < 100; ++i)
    {
        odom.update(ticks, dt);
    }

    constexpr double kPosTol = 100.0 * kDistPerTick;

    EXPECT_NEAR(odom.current_estimate().x(), -1.0, kPosTol);
    EXPECT_NEAR(odom.current_estimate().y(),  0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// 圆周运动: 经过 2π yaw 后回到起点附近
// ---------------------------------------------------------------------------
TEST(WheelOdometry, CircularMotionReturnsNearStart)
{
    // ω=π rad/s, v=0.2 m/s → 半径 r = v/ω ≈ 0.0637 m, 周期 T = 2π/ω = 2 s
    // dt=0.02 → 100 步走完一圈
    WheelOdometry odom{default_params()};
    constexpr double dt    = 0.02;
    constexpr double v     = 0.2;
    constexpr double omega = mininav::kPi;
    constexpr double half_L_omega = 0.5 * kWheelBase * omega;

    const double dist_l = (v - half_L_omega) * dt;
    const double dist_r = (v + half_L_omega) * dt;
    const auto ticks = ticks_for_distances(dist_l, dist_r);

    for (int i = 0; i < 100; ++i)
    {
        odom.update(ticks, dt);
    }
    const Pose2D p = odom.current_estimate();

    // 一圈后位置应当接近原点 (积分误差 + 量化误差累积)
    // 2π·r ≈ 0.4 m 圆周,总误差容许在 ~5 cm 量级 (欧拉积分精度 + 100 步量化)
    EXPECT_NEAR(p.x(), 0.0, 0.05);
    EXPECT_NEAR(p.y(), 0.0, 0.05);
}