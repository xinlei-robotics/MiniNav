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
    Vec5 make_random_state(std::mt19937& rng)
    {
        std::uniform_real_distribution<double> pos(-2.0, 2.0);
        std::uniform_real_distribution<double> ang(-1.0, 1.0);
        std::uniform_real_distribution<double> vel(-1.0, 1.0);

        Vec5 x;
        x << pos(rng), pos(rng), ang(rng), vel(rng), vel(rng);
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
        const Vec5 x = make_random_state(rng);
        const Mat5 G_analytic = process_jacobian_G(x, dt);
        const Mat5 G_numeric = numeric_process_jacobian(x, dt, eps);

        EXPECT_TRUE(G_analytic.isApprox(G_numeric, 1e-6))
            << "trial " << trial << '\n'
            << "state = " << x.transpose() << '\n'
            << "analytic:\n" << G_analytic << '\n'
            << "numeric:\n" << G_numeric << '\n';
    }
}

TEST(EkfJacobian, ZeroVelocityDecouplesHeadingFromPosition)
{
    Vec5 x;
    x << 1.0, 2.0, 0.7, 0.0, 0.5;
    constexpr double dt = 0.02;

    const Mat5 G = process_jacobian_G(x, dt);

    EXPECT_DOUBLE_EQ(G(kPx, kTheta), 0.0);
    EXPECT_DOUBLE_EQ(G(kPy, kTheta), 0.0);
    EXPECT_DOUBLE_EQ(G(kTheta, kOmega), dt);
}