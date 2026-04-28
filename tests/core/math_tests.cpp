import mininav.core.math;

#include <gtest/gtest.h>

#include <cmath>

namespace
{
    constexpr double kEps = 1e-12;
} // namespace

TEST(WrapAngle, ZeroIsZero)
{
    EXPECT_NEAR(mininav::wrap_angle(0.0), 0.0, kEps);
}

TEST(WrapAngle, PiMapsToPi)
{
    // 约定区间为 (-pi, pi],所以 +pi 应保留为 +pi(右闭)
    EXPECT_NEAR(mininav::wrap_angle(mininav::kPi), mininav::kPi, kEps);
}

TEST(WrapAngle, NegativePiMapsToPi)
{
    // -pi 不在区间内,应被映射到 +pi(右端点)
    EXPECT_NEAR(mininav::wrap_angle(-mininav::kPi), mininav::kPi, kEps);
}

TEST(WrapAngle, JustAbovePiWrapsToNegative)
{
    const double a = mininav::kPi + 0.1;
    EXPECT_NEAR(mininav::wrap_angle(a), -mininav::kPi + 0.1, 1e-9);
}

TEST(WrapAngle, JustBelowNegativePiWrapsToPositive)
{
    const double a = -mininav::kPi - 0.1;
    EXPECT_NEAR(mininav::wrap_angle(a), mininav::kPi - 0.1, 1e-9);
}

TEST(WrapAngle, MultipleFullRotations)
{
    // 多圈情况:5pi 应等价于 +pi
    EXPECT_NEAR(mininav::wrap_angle(5.0 * mininav::kPi), mininav::kPi, 1e-9);
    // -5pi 也应等价于 +pi
    EXPECT_NEAR(mininav::wrap_angle(-5.0 * mininav::kPi), mininav::kPi, 1e-9);
}

TEST(WrapAngle, ResultAlwaysInRange)
{
    // 抽样检验: 任意取值 wrap 后应严格在 (-pi, pi]
    const double samples[] = {-100.0, -10.0, -1.0, 0.0, 1.0, 10.0, 100.0};
    for (const double a : samples)
    {
        const double w = mininav::wrap_angle(a);
        EXPECT_GT(w, -mininav::kPi);
        EXPECT_LE(w, mininav::kPi);
    }
}

TEST(MathConstants, TwoPiEqualsTwoTimesPi)
{
    EXPECT_NEAR(mininav::kTwoPi, 2.0 * mininav::kPi, kEps);
}
