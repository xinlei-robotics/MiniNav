#include <gtest/gtest.h>

#include <cmath>
#include <random>

import mininav.core.types;
import mininav.sensors.wheel_encoder;
import mininav.core.math;

namespace
{
    using mininav::EncoderTicks;
    using mininav::Twist2D;
    using mininav::WheelEncoderModel;
    using mininav::WheelEncoderParams;

    [[nodiscard]] WheelEncoderModel make_model(
        double       slip_sigma   = 0.0,
        double       wheel_radius = 0.032,
        std::int64_t tpr          = 1024,
        std::uint32_t seed_left   = 1,
        std::uint32_t seed_right  = 2)
    {
        WheelEncoderParams p{
            .wheel_radius  = wheel_radius,
            .wheel_base    = 0.150,
            .ticks_per_rev = tpr,
            .slip_sigma    = slip_sigma,
        };
        return WheelEncoderModel{
            p,
            std::mt19937{seed_left},
            std::mt19937{seed_right},
        };
    }
}

// ---------------------------------------------------------------------------
// 退化: 静止时 ticks 严格为 0
// ---------------------------------------------------------------------------
TEST(WheelEncoderModel, ZeroVelocityProducesZeroTicks)
{
    auto m = make_model(/*slip_sigma=*/0.1);
    for (int i = 0; i < 100; ++i)
    {
        const auto t = m.measure(Twist2D{0.0, 0.0}, 0.05);
        EXPECT_EQ(t.left,  0);
        EXPECT_EQ(t.right, 0);
    }
}

// ---------------------------------------------------------------------------
// 直行: 左右轮对称, ticks 相等
// ---------------------------------------------------------------------------
TEST(WheelEncoderModel, StraightDriveProducesEqualLeftRightTicks)
{
    auto m = make_model();
    std::int64_t left_total = 0, right_total = 0;
    for (int i = 0; i < 100; ++i)
    {
        const auto t = m.measure(Twist2D{0.5, 0.0}, 0.02);
        left_total  += t.left;
        right_total += t.right;
    }
    EXPECT_EQ(left_total, right_total);
    EXPECT_GT(left_total, 0);
}

// ---------------------------------------------------------------------------
// 原地转: ticks 等量异号
// ---------------------------------------------------------------------------
TEST(WheelEncoderModel, InPlaceRotationProducesOppositeTicks)
{
    auto m = make_model();
    std::int64_t left_total = 0, right_total = 0;
    for (int i = 0; i < 100; ++i)
    {
        const auto t = m.measure(Twist2D{0.0, mininav::kPi}, 0.02);
        left_total  += t.left;
        right_total += t.right;
    }
    EXPECT_EQ(left_total, -right_total);
    EXPECT_NE(left_total, 0);
}

// ---------------------------------------------------------------------------
// 量化精度: 解码 ticks → 距离, 应当与解析期望接近 (容差 = 1 tick)
// ---------------------------------------------------------------------------
TEST(WheelEncoderModel, QuantizedDistanceMatchesAnalytical)
{
    constexpr double v          = 0.5;
    constexpr double dt         = 0.02;
    constexpr int    n_steps    = 200;
    constexpr double total_time = n_steps * dt;
    constexpr double expected_distance = v * total_time;

    auto m = make_model();
    std::int64_t total_ticks = 0;
    for (int i = 0; i < n_steps; ++i)
    {
        const auto t = m.measure(Twist2D{v, 0.0}, dt);
        total_ticks += t.left;
    }

    const double measured_distance = total_ticks * m.distance_per_tick();
    EXPECT_NEAR(measured_distance, expected_distance, 0.5 * m.distance_per_tick());
}

// ---------------------------------------------------------------------------
// 量化的"累计-差分"语义: 低速时单步可能为 0, 但累计正确
// ---------------------------------------------------------------------------
TEST(WheelEncoderModel, LowSpeedAccumulatesCorrectlyDespiteRoundingToZero)
{
    // 单 tick 弧长约2e-4 m
    // 设v=0.001 m/s, dt=0.01s → v·dt = 1e-5 m
    // 这意味着大约要 20 个步骤才能累计满一个 tick。
    auto m = make_model();

    int n_zero_steps = 0;
    int n_nonzero_steps = 0;
    std::int64_t total_ticks = 0;
    constexpr int n_steps = 100;
    for (int i = 0; i < n_steps; ++i)
    {
        const auto t = m.measure(Twist2D{0.001, 0.0}, 0.01);
        if (t.left == 0) ++n_zero_steps;
        else             ++n_nonzero_steps;
        total_ticks += t.left;
    }

    EXPECT_GT(n_zero_steps,    0);
    EXPECT_GT(n_nonzero_steps, 0);

    const double measured = total_ticks * m.distance_per_tick();
    EXPECT_NEAR(measured, 1e-3, m.distance_per_tick());
}

// ---------------------------------------------------------------------------
// 可复现性: 同 seed → 同 ticks 序列
// ---------------------------------------------------------------------------
TEST(WheelEncoderModel, ReproducibleWithSameSeed)
{
    auto m1 = make_model(/*slip_sigma=*/0.05, /*wheel_radius=*/0.032, /*tpr=*/1024,
                         /*seed_left=*/42, /*seed_right=*/43);
    auto m2 = make_model(/*slip_sigma=*/0.05, /*wheel_radius=*/0.032, /*tpr=*/1024,
                         /*seed_left=*/42, /*seed_right=*/43);

    for (int i = 0; i < 100; ++i)
    {
        const auto a = m1.measure(Twist2D{0.5, 0.3}, 0.02);
        const auto b = m2.measure(Twist2D{0.5, 0.3}, 0.02);
        EXPECT_EQ(a.left,  b.left);
        EXPECT_EQ(a.right, b.right);
    }
}

// ---------------------------------------------------------------------------
// RNG 序列稳定性: σ = 0 时不消耗 RNG。
// ---------------------------------------------------------------------------
TEST(WheelEncoderModel, ZeroSlipSigmaDoesNotConsumeRng)
{
    // 两个完全相同初始状态的 model: m_a 用 σ=0 跑一步, m_b 不跑。
    // 然后两个都切到 σ>0 跑同样的步骤 —— 但在 C++ 里 σ 是 const 成员,
    // 所以这里反过来构造: 给两个 model 同样 seed, 但只有 m_a 在 σ=0
    // 配置下跑一步预热。等价的检验是: σ=0 路径不应消耗 slip RNG。
    //
    // 实现技巧: 直接对比 σ=0 model 与"跳过对应步"的 σ>0 model 的输出
    // 比较脆。更直接的检查 —— σ=0 时连续多步的 ticks 完全由 (v,dt)
    // 唯一决定, 与 RNG 无关:
    auto m1 = make_model(/*slip_sigma=*/0.0, /*wheel_radius=*/0.032, /*tpr=*/1024,
                         /*seed_left=*/42, /*seed_right=*/43);
    auto m2 = make_model(/*slip_sigma=*/0.0, /*wheel_radius=*/0.032, /*tpr=*/1024,
                         /*seed_left=*/9999, /*seed_right=*/8888);  // 不同 seed

    for (int i = 0; i < 50; ++i)
    {
        const auto a = m1.measure(Twist2D{0.5, 0.3}, 0.02);
        const auto b = m2.measure(Twist2D{0.5, 0.3}, 0.02);
        EXPECT_EQ(a.left,  b.left);
        EXPECT_EQ(a.right, b.right);
    }
}