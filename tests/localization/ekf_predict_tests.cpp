// ===========================================================================
// Ekf::predict 行为测试。
// ===========================================================================
#include <gtest/gtest.h>

#include <Eigen/Dense>

import mininav.localization.ekf;
import mininav.localization.ekf_state;
import mininav.core.math;

using namespace mininav;

namespace
{
    EkfState6 state_with_velocity(double v, double w)
    {
        EkfState6 s;
        s.mu << 0.0, 0.0, 0.0, v, w, 0.0; // 6D: 末位为 gyro bias b_ω₀ = 0
        s.Sigma = default_initial_covariance();
        return s;
    }

    ProcessNoiseParams default_preset_noise()
    {
        return ProcessNoiseParams{.alpha1 = 0.05, .alpha2 = 0.02, .alpha3 = 0.02, .alpha4 = 0.05};
    }
}

// --- 均值传播 -------------------------------------------------------------

TEST(EkfPredict, MeanIsStationaryWhenVelocityIsZero)
{
    Ekf ekf{make_initial_ekf_state(), default_preset_noise()};
    const Vec6 mu0 = ekf.mu();

    for (int i = 0; i < 2000; ++i)
    {
        ekf.predict(0.01);
    }

    EXPECT_LT((ekf.mu() - mu0).norm(), 1e-12);
}

TEST(EkfPredict, MeanFollowsStraightLineUnderConstantForwardVelocity)
{
    Ekf ekf{state_with_velocity(0.5, 0.0), ProcessNoiseParams{}};

    for (int i = 0; i < 10; ++i)
    {
        ekf.predict(0.01);
    }

    EXPECT_NEAR(ekf.mu()(kPx), 0.5 * 0.1, 1e-9);
    EXPECT_NEAR(ekf.mu()(kPy), 0.0, 1e-12);
    EXPECT_NEAR(ekf.mu()(kTheta), 0.0, 1e-12);
    EXPECT_NEAR(ekf.mu()(kV), 0.5, 1e-12);
}

TEST(EkfPredict, MeanRotatesInPlaceUnderConstantYawRate)
{
    Ekf ekf{state_with_velocity(0.0, 1.0), ProcessNoiseParams{}};

    for (int i = 0; i < 100; ++i)
    {
        ekf.predict(0.01);
    }

    EXPECT_NEAR(ekf.mu()(kTheta), 1.0, 1e-9);
    EXPECT_NEAR(ekf.mu()(kPx), 0.0, 1e-12);
    EXPECT_NEAR(ekf.mu()(kPy), 0.0, 1e-12);
}

TEST(EkfPredict, HeadingIsWrappedIntoCanonicalRange)
{
    Ekf ekf{state_with_velocity(0.0, 1.0), ProcessNoiseParams{}};
    for (int i = 0; i < 400; ++i)
    {
        ekf.predict(0.01);
    }
    EXPECT_LE(ekf.mu()(kTheta), kPi);
    EXPECT_GT(ekf.mu()(kTheta), -kPi);
    EXPECT_NEAR(ekf.mu()(kTheta), wrap_angle(4.0), 1e-9);
}

// --- 协方差行为 -----------------------------------------------------------

TEST(EkfPredict, CovarianceGrowsWithoutMeasurement)
{
    Ekf ekf{state_with_velocity(0.4, 0.3), default_preset_noise()};
    const double trace0 = ekf.Sigma().trace();
    const double sigma_xx0 = ekf.Sigma()(kPx, kPx);

    for (int i = 0; i < 500; ++i)
    {
        ekf.predict(0.01);
    }

    EXPECT_GT(ekf.Sigma().trace(), trace0);
    EXPECT_GT(ekf.Sigma()(kPx, kPx), sigma_xx0);
}

TEST(EkfPredict, CovarianceGrowsEvenWhenProcessNoiseIsZero)
{
    Ekf ekf{state_with_velocity(0.0, 0.0), ProcessNoiseParams{}};
    const double sigma_xx0 = ekf.Sigma()(kPx, kPx);

    for (int i = 0; i < 1000; ++i)
    {
        ekf.predict(0.01);
    }

    EXPECT_GT(ekf.Sigma()(kPx, kPx), sigma_xx0);
}

TEST(EkfPredict, CovarianceTraceIsMonotoneNonDecreasing)
{
    Ekf ekf{state_with_velocity(0.4, 0.3), default_preset_noise()};
    double previous = ekf.Sigma().trace();

    for (int i = 0; i < 500; ++i)
    {
        ekf.predict(0.01);
        const double current = ekf.Sigma().trace();
        EXPECT_GE(current, previous - 1e-12) << "predict 不应使不确定性变小 (step " << i << ')';
        previous = current;
    }
}

// --- 数值健康 -------------------------------------------------------------

TEST(EkfPredict, CovarianceRemainsSymmetricPositiveDefinite)
{
    Ekf ekf{state_with_velocity(0.4, 0.3), default_preset_noise()};

    for (int i = 0; i < 2000; ++i)
    {
        ekf.predict(0.01);
        ASSERT_TRUE(is_symmetric_positive_definite(ekf.Sigma(), 1e-9))
            << "Σ 在第 " << i << " 步丢失 symmetric-PSD 性质";
    }
}

TEST(EkfPredict, InitialCovarianceMatchesDesignedSigma0)
{
    const Mat6 Sigma0 = default_initial_covariance();
    EXPECT_DOUBLE_EQ(Sigma0(kPx, kPx), 1e-6);
    EXPECT_DOUBLE_EQ(Sigma0(kPy, kPy), 1e-6);
    EXPECT_DOUBLE_EQ(Sigma0(kTheta, kTheta), 1e-6);
    EXPECT_DOUBLE_EQ(Sigma0(kV, kV), 1e-2);
    EXPECT_DOUBLE_EQ(Sigma0(kOmega, kOmega), 1e-2);
    EXPECT_DOUBLE_EQ(Sigma0(kBiasOmega, kBiasOmega), 1e-2);
    EXPECT_TRUE(is_symmetric_positive_definite(Sigma0, 1e-12));
}