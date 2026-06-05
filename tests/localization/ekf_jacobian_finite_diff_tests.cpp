// ===========================================================================
// 解析 Jacobian G vs 中心差分。
// ===========================================================================
#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <random>

import mininav.localization.ekf;
import mininav.localization.ekf_state;

using namespace mininav;

namespace
{
    Vec6 make_random_state(std::mt19937& rng)
    {
        std::uniform_real_distribution<double> pos(-2.0, 2.0);
        std::uniform_real_distribution<double> ang(-1.0, 1.0);
        std::uniform_real_distribution<double> vel(-1.0, 1.0);
        std::uniform_real_distribution<double> bias(-0.1, 0.1); // gyro bias 量级很小

        Vec6 x;
        x << pos(rng), pos(rng), ang(rng), vel(rng), vel(rng), bias(rng);
        return x;
    }
}

TEST(EkfJacobian, MatchesFiniteDifference)
{
    std::mt19937 rng{42};
    constexpr double dt = 0.01;
    constexpr double eps = 1e-6;

    for (int trial = 0; trial < 200; ++trial)
    {
        const Vec6 x = make_random_state(rng);
        const Mat6 G_analytic = process_jacobian_G(x, dt);
        const Mat6 G_numeric = numeric_process_jacobian(x, dt, eps);

        EXPECT_TRUE(G_analytic.isApprox(G_numeric, 1e-6))
            << "trial " << trial << '\n'
            << "state = " << x.transpose() << '\n'
            << "analytic:\n" << G_analytic << '\n'
            << "numeric:\n" << G_numeric << '\n';
    }
}

TEST(EkfJacobian, ZeroVelocityDecouplesHeadingFromPosition)
{
    Vec6 x;
    x << 1.0, 2.0, 0.7, 0.0, 0.5, 0.0;
    constexpr double dt = 0.02;

    const Mat6 G = process_jacobian_G(x, dt);

    EXPECT_DOUBLE_EQ(G(kPx, kTheta), 0.0);
    EXPECT_DOUBLE_EQ(G(kPy, kTheta), 0.0);
    EXPECT_DOUBLE_EQ(G(kTheta, kOmega), dt);
}

// PR4: bias 维在过程模型中是纯随机游走(g 对它恒等映射), G 的 bias 行/列除对角
// 外全为 0; bias 不参与 θ/位置的预测。固化这一结构, 防止未来误把 bias 耦合进 g。
TEST(EkfJacobian, BiasIsDecoupledInProcessModel)
{
    Vec6 x;
    x << 1.0, 2.0, 0.7, 0.5, 0.3, 0.05;
    constexpr double dt = 0.02;

    const Mat6 G = process_jacobian_G(x, dt);

    // bias 列: 除 (b,b)=1 外全 0 —— 没有任何状态的预测依赖当前 bias。
    for (int i = 0; i < kStateDim; ++i)
    {
        const double expected = (i == kBiasOmega) ? 1.0 : 0.0;
        EXPECT_DOUBLE_EQ(G(i, kBiasOmega), expected) << "G 第 " << i << " 行的 bias 列";
    }
    // bias 行: 除 (b,b)=1 外全 0 —— bias 的预测不依赖任何其他状态。
    for (int j = 0; j < kStateDim; ++j)
    {
        const double expected = (j == kBiasOmega) ? 1.0 : 0.0;
        EXPECT_DOUBLE_EQ(G(kBiasOmega, j), expected) << "G bias 行的第 " << j << " 列";
    }
}