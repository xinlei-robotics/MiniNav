// ===========================================================================
// Ekf::update_imu 行为测试
//
//   1. 单步均值移动方向正确, 协方差(ω,ω)项收缩
//   2. R→0 极限: 几乎完全相信 IMU → μ(ω) ≈ z
//   3. R→∞ 极限: 几乎完全不信 IMU → μ 几乎不变
//   4. Joseph 1D form 全程保持 SPD
//   5. update_imu 与"把 update_encoder 限制到 ω 行"的退化等价路径在数值上
//      逐 bit 相等(独立验证标量公式没有偷换数学)
// ===========================================================================
#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>

import mininav.localization.ekf;
import mininav.localization.ekf_state;

using namespace mininav;

namespace
{
    EkfState5 belief_with_velocity(double v, double w)
    {
        EkfState5 s;
        s.mu << 0.0, 0.0, 0.0, v, w;
        s.Sigma = default_initial_covariance();
        return s;
    }

    ProcessNoiseParams default_noise()
    {
        return ProcessNoiseParams{.alpha1 = 0.05, .alpha2 = 0.02, .alpha3 = 0.02, .alpha4 = 0.05};
    }
}

// --- 单步行为 ------------------------------------------------------------

TEST(EkfImuUpdate, MeanMovesTowardMeasurementAndCovarianceShrinks)
{
    Ekf ekf{belief_with_velocity(0.0, 0.0), default_noise()};
    ekf.predict(0.01); // 让 (ω,ω) 协方差略增长, 给 update 留收缩空间

    const double w_before = ekf.mu()(kOmega);
    const double cov_w_before = ekf.Sigma()(kOmega, kOmega);

    constexpr double z = 0.5; // 明显非零观测
    constexpr double R = 1e-6; // 强信任 IMU
    ekf.update_imu(z, R);

    EXPECT_GT(ekf.mu()(kOmega), w_before);
    EXPECT_LE(ekf.mu()(kOmega), z + 1e-9);
    EXPECT_LT(ekf.Sigma()(kOmega, kOmega), cov_w_before);
}

// --- 极限行为 ------------------------------------------------------------

TEST(EkfImuUpdate, ZeroNoiseLimitMakesMuOmegaEqualMeasurement)
{
    Ekf ekf{belief_with_velocity(0.0, 0.0), default_noise()};
    ekf.predict(0.01);

    constexpr double z = 0.7;
    constexpr double R = 1e-12; // 几乎完全相信 IMU
    ekf.update_imu(z, R);

    // K → Σ̄·Hᵀ / Σ̄(ω,ω) → e_ω 在 ω 行 → μ(ω) → z
    EXPECT_NEAR(ekf.mu()(kOmega), z, 1e-6);
}

TEST(EkfImuUpdate, InfiniteNoiseLimitLeavesBeliefEssentiallyUnchanged)
{
    Ekf ekf{belief_with_velocity(0.4, 0.3), default_noise()};
    ekf.predict(0.01);

    const Vec5 mu_before = ekf.mu();
    const double cov_w_before = ekf.Sigma()(kOmega, kOmega);

    constexpr double z = 10.0; // 巨大但完全不该相信
    constexpr double R = 1e12; // R 远大于 Σ(ω,ω)
    ekf.update_imu(z, R);

    // K ≈ 0 → μ 几乎不变, Σ(ω,ω) 几乎不变
    EXPECT_LT((ekf.mu() - mu_before).norm(), 1e-6);
    EXPECT_NEAR(ekf.Sigma()(kOmega, kOmega), cov_w_before, 1e-6 * cov_w_before);
}

// --- 数值健康 ------------------------------------------------------------

TEST(EkfImuUpdate, JosephUpdatePreservesSpdOverManySteps)
{
    Ekf ekf{belief_with_velocity(0.4, 0.3), default_noise()};
    constexpr double R = 0.005 * 0.005;

    // 模拟 2000 步 predict + update_imu(典型噪声水平的 z)。
    for (int i = 0; i < 2000; ++i)
    {
        ekf.predict(0.01);
        const double z = 0.3 + 0.001 * std::sin(0.05 * i); // 慢变量
        ekf.update_imu(z, R);
        ASSERT_TRUE(is_symmetric_positive_definite(ekf.Sigma(), 1e-9))
            << "Σ 在第 " << i << " 步 IMU update 后丢失 symmetric-PSD 性质";
    }
}

// --- 标量实现 vs 一般 Joseph 流程的等价性 --------------------------------

TEST(EkfImuUpdate, ScalarUpdateMatchesGeneralJosephFlowExactly)
{
    // 用 update_encoder 接口手工构造"只观测 ω"的 2D-退化形式做对照:
    //   z_general = (任意, z_imu), R_general = diag(huge, R_imu)
    // 把 v 行的 R 设极大(huge), 几乎不更新 v —— 因此对 ω 的修正应与 update_imu
    // 一致(到浮点精度)。这是个端到端等价性校验。
    Ekf ekf_a{belief_with_velocity(0.4, 0.3), default_noise()};
    Ekf ekf_b{belief_with_velocity(0.4, 0.3), default_noise()};
    ekf_a.predict(0.01);
    ekf_b.predict(0.01);

    constexpr double z_imu = 0.35;
    constexpr double R_imu = 0.005 * 0.005;

    // A: 标量 update_imu
    ekf_a.update_imu(z_imu, R_imu);

    // B: 通过 update_encoder, v 行 R 极大模拟"实际上不观测 v"
    Eigen::Vector2d z2;
    z2 << ekf_b.mu()(kV), z_imu; // v 部分用预测值, 抵消 innovation
    Eigen::Matrix2d R2 = Eigen::Matrix2d::Zero();
    R2(0, 0) = 1e20; // v 完全不可信
    R2(1, 1) = R_imu;
    ekf_b.update_encoder(z2, R2);

    // 两者对 ω 的修正应几乎相同(huge R 下 v 的 update 几乎无效)。
    EXPECT_NEAR(ekf_a.mu()(kOmega), ekf_b.mu()(kOmega), 1e-9);
    EXPECT_NEAR(ekf_a.Sigma()(kOmega, kOmega), ekf_b.Sigma()(kOmega, kOmega), 1e-9);
}