#include <gtest/gtest.h>

#include <cmath>
#include <random>

import mininav.core.types;
import mininav.core.random;
import mininav.sensors.actuator_model;

namespace
{
    using mininav::ActuatorModel;
    using mininav::ActuatorNoiseParams;
    using mininav::RngFactory;
    using mininav::Twist2D;
}

// ---------------------------------------------------------------------------
// 退化测试: 零参数 == 理想执行器
// ---------------------------------------------------------------------------
TEST(ActuatorModel, ZeroParamsIsIdentity)
{
    ActuatorNoiseParams p{};  // 全 0
    ActuatorModel m{p, std::mt19937{42}};

    struct Case { double v, w; };
    for (const auto& c :
         {Case{1.0, 0.5}, Case{-2.0, 0.3}, Case{0.0, 0.0}, Case{0.7, -1.2}})
    {
        const Twist2D out = m.apply(Twist2D{c.v, c.w});
        EXPECT_DOUBLE_EQ(out.v(), c.v);
        EXPECT_DOUBLE_EQ(out.w(), c.w);
    }
}

// ---------------------------------------------------------------------------
// 物理边界: cmd = (0, 0) 静止时无噪声
// ---------------------------------------------------------------------------
TEST(ActuatorModel, ZeroCommandProducesZeroOutputEvenWithNoise)
{
    constexpr ActuatorNoiseParams p{0.1, 0.1, 0.1, 0.1};
    ActuatorModel m{p, std::mt19937{42}};

    const Twist2D out = m.apply(Twist2D{0.0, 0.0});
    EXPECT_DOUBLE_EQ(out.v(), 0.0);
    EXPECT_DOUBLE_EQ(out.w(), 0.0);
}

// ---------------------------------------------------------------------------
// 可复现性: 同 seed 同参数 == 同输出序列
// ---------------------------------------------------------------------------
TEST(ActuatorModel, ReproducibleWithSameSeed)
{
    constexpr ActuatorNoiseParams p{0.1, 0.05, 0.05, 0.1};
    ActuatorModel m1{p, std::mt19937{42}};
    ActuatorModel m2{p, std::mt19937{42}};

    for (int i = 0; i < 50; ++i)
    {
        const Twist2D in{1.0, 0.5};
        const auto a = m1.apply(in);
        const auto b = m2.apply(in);
        EXPECT_DOUBLE_EQ(a.v(), b.v());
        EXPECT_DOUBLE_EQ(a.w(), b.w());
    }
}

// ---------------------------------------------------------------------------
// 集成测试: 与 RngFactory 协作正常
// ---------------------------------------------------------------------------
TEST(ActuatorModel, IntegratesWithRngFactory)
{
    const RngFactory factory{42};
    constexpr ActuatorNoiseParams p{0.1, 0.0, 0.0, 0.1};
    ActuatorModel m{p, factory.make_engine("actuator")};

    const auto out = m.apply(Twist2D{1.0, 0.5});
    EXPECT_NE(out.v(), 1.0);
    EXPECT_NE(out.w(), 0.5);
}

// ---------------------------------------------------------------------------
// 统计性质: 样本均值 (5-sigma 容差)
// ---------------------------------------------------------------------------
TEST(ActuatorModel, SampleMeanIsUnbiased)
{
    constexpr int    N      = 10000;
    constexpr double v_cmd  = 1.0;
    constexpr double w_cmd  = 0.5;
    constexpr double alpha1 = 0.05;
    constexpr double alpha4 = 0.05;

    constexpr ActuatorNoiseParams p{.alpha1 = alpha1, .alpha4 = alpha4};
    ActuatorModel m{p, std::mt19937{42}};

    double v_sum = 0.0, w_sum = 0.0;
    for (int i = 0; i < N; ++i)
    {
        const auto out = m.apply(Twist2D{v_cmd, w_cmd});
        v_sum += out.v() - v_cmd;
        w_sum += out.w() - w_cmd;
    }
    const double v_mean = v_sum / N;
    const double w_mean = w_sum / N;

    // 期望 std: sigma_v = sqrt(alpha1 * v²) = sqrt(0.05) ≈ 0.224
    // 样本均值 std = sigma / sqrt(N), 5-sigma 容差几乎一定通过。
    const double v_sigma = std::sqrt(alpha1 * v_cmd * v_cmd);
    const double w_sigma = std::sqrt(alpha4 * w_cmd * w_cmd);
    EXPECT_NEAR(v_mean, 0.0, 5.0 * v_sigma / std::sqrt(static_cast<double>(N)));
    EXPECT_NEAR(w_mean, 0.0, 5.0 * w_sigma / std::sqrt(static_cast<double>(N)));
}

// ---------------------------------------------------------------------------
// 统计性质: 样本方差对应解析公式 sigma² = alpha1*v² + alpha2*w²
// ---------------------------------------------------------------------------
TEST(ActuatorModel, SampleVarianceMatchesAnalyticalFormula)
{
    constexpr int    N     = 50000;
    constexpr double v_cmd = 2.0;
    constexpr double w_cmd = 1.0;

    ActuatorNoiseParams p{.alpha1 = 0.10, .alpha2 = 0.05,
                          .alpha3 = 0.05, .alpha4 = 0.10};
    ActuatorModel m{p, std::mt19937{12345}};

    double v_sum = 0.0, v_sumsq = 0.0;
    double w_sum = 0.0, w_sumsq = 0.0;
    for (int i = 0; i < N; ++i)
    {
        const auto   out = m.apply(Twist2D{v_cmd, w_cmd});
        const double dv  = out.v() - v_cmd;
        const double dw  = out.w() - w_cmd;
        v_sum += dv; v_sumsq += dv * dv;
        w_sum += dw; w_sumsq += dw * dw;
    }
    const auto Nf = static_cast<double>(N);
    const double v_var_sample  = (v_sumsq - v_sum * v_sum / Nf) / (Nf - 1.0);
    const double w_var_sample  = (w_sumsq - w_sum * w_sum / Nf) / (Nf - 1.0);

    // 解析方差: alpha1*v² + alpha2*w²
    constexpr double v_var_expected = 0.10 * 4.0 + 0.05 * 1.0;  // 0.45
    constexpr double w_var_expected = 0.05 * 4.0 + 0.10 * 1.0;  // 0.30

    // 高斯样本方差的理论 std ≈ var * sqrt(2/N), 5-sigma 容差。
    const double v_tol = 5.0 * v_var_expected * std::sqrt(2.0 / Nf);
    const double w_tol = 5.0 * w_var_expected * std::sqrt(2.0 / Nf);

    EXPECT_NEAR(v_var_sample, v_var_expected, v_tol);
    EXPECT_NEAR(w_var_sample, w_var_expected, w_tol);
}

// ---------------------------------------------------------------------------
// RNG 序列稳定性: 调用 apply 时若 sigma=0 (因零命令), 不应消耗 RNG 状态。
// ---------------------------------------------------------------------------

TEST(ActuatorModel, ZeroCommandDoesNotConsumeRng)
{
    constexpr ActuatorNoiseParams p{0.1, 0.1, 0.1, 0.1};
    std::mt19937       rng_a{42};
    std::mt19937       rng_b{42};
    ActuatorModel      m_a{p, std::move(rng_a)};
    ActuatorModel      m_b{p, std::move(rng_b)};

    // m_a: 先 apply (0,0) 一次,  再 apply (1, 0.5) 一次
    // m_b: 直接 apply (1, 0.5) 一次
    // 如果 (0,0) 不消耗 RNG, 两者第二次 apply 的输出必须完全一致。
    [[maybe_unused]] const auto _ = m_a.apply(Twist2D{0.0, 0.0});

    const auto out_a = m_a.apply(Twist2D{1.0, 0.5});
    const auto out_b = m_b.apply(Twist2D{1.0, 0.5});

    EXPECT_DOUBLE_EQ(out_a.v(), out_b.v());
    EXPECT_DOUBLE_EQ(out_a.w(), out_b.w());
}