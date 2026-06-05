// ===========================================================================
// Ekf::update_imu 行为测试 (PR4: 观测模型 z = ω + b_ω, H = [0,0,0,0,1,1])
//
//   1. 单步均值移动方向正确, 协方差(ω,ω)项收缩
//   2. R→0 极限: 几乎完全相信 IMU → 被观测的"和"μ(ω)+μ(b) ≈ z
//      (注意: ω 与 b 在单次观测里线性不可分, 故 μ(ω) 单独并不等于 z)
//   3. R→∞ 极限: 几乎完全不信 IMU → μ 几乎不变
//   4. Joseph 1D form 全程保持 SPD
//   5. update_imu 与手写的一般 1×6 矩阵 Joseph 流程 (同一 H=[0,0,0,0,1,1])
//      在数值上逐分量相等 —— 独立验证标量展开式没有偷换数学
// ===========================================================================
#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>

import mininav.localization.ekf;
import mininav.localization.ekf_state;
import mininav.core.math;

using namespace mininav;

namespace
{
    EkfState6 belief_with_velocity(double v, double w)
    {
        EkfState6 s;
        s.mu << 0.0, 0.0, 0.0, v, w, 0.0; // 6D: 末位为 gyro bias b_ω₀ = 0
        s.Sigma = default_initial_covariance();
        return s;
    }

    ProcessNoiseParams default_noise()
    {
        return ProcessNoiseParams{
            .alpha1 = 0.05, .alpha2 = 0.02, .alpha3 = 0.02, .alpha4 = 0.05,
            .q_bias_omega = 1e-8,
        };
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

    // 修正量在 ω 与 b 之间劈分, 故 ω 向 z 方向移动但不会越过 z。
    EXPECT_GT(ekf.mu()(kOmega), w_before);
    EXPECT_LE(ekf.mu()(kOmega), z + 1e-9);
    EXPECT_LT(ekf.Sigma()(kOmega, kOmega), cov_w_before);
}

// --- 极限行为 ------------------------------------------------------------

// R→0 时滤波器几乎完全相信 IMU, 把"被观测的和" ω+b 推到 z。
// 因 K(ω)+K(b) = S/S = 1, 这个和的修正是精确的: μ(ω)+μ(b) → z。
// 但 ω 与 b 各自如何分摊取决于先验协方差, 故不能断言 μ(ω) 单独等于 z。
TEST(EkfImuUpdate, ZeroNoiseLimitMakesObservedSumEqualMeasurement)
{
    Ekf ekf{belief_with_velocity(0.0, 0.0), default_noise()};
    ekf.predict(0.01);

    constexpr double z = 0.7;
    constexpr double R = 1e-12; // 几乎完全相信 IMU
    ekf.update_imu(z, R);

    EXPECT_NEAR(ekf.mu()(kOmega) + ekf.mu()(kBiasOmega), z, 1e-6);
}

TEST(EkfImuUpdate, InfiniteNoiseLimitLeavesBeliefEssentiallyUnchanged)
{
    Ekf ekf{belief_with_velocity(0.4, 0.3), default_noise()};
    ekf.predict(0.01);

    const Vec6 mu_before = ekf.mu();
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
    // 注意: 仅 IMU 时 (ω−b) 方向不可观测, 其方差不收缩, 但 Joseph form 仍结构性
    // 保证 Σ 全程 symmetric PSD。
    for (int i = 0; i < 2000; ++i)
    {
        ekf.predict(0.01);
        const double z = 0.3 + 0.001 * std::sin(0.05 * i); // 慢变量
        ekf.update_imu(z, R);
        ASSERT_TRUE(is_symmetric_positive_definite(ekf.Sigma(), 1e-9))
            << "Σ 在第 " << i << " 步 IMU update 后丢失 symmetric-PSD 性质";
    }
}

// --- 标量实现 vs 一般矩阵 Joseph 流程的等价性 ----------------------------

// 用手写的一般 1×6 矩阵 Joseph form (同一观测模型 H = [0,0,0,0,1,1]) 独立复算
// update_imu, 验证标量展开式 (SHt = Σ.col(ω)+Σ.col(b)、IKH 只改 ω/b 两列) 与
// 教科书矩阵式逐分量相等。这是端到端的"没偷换数学"校验。
TEST(EkfImuUpdate, ScalarUpdateMatchesGeneralMatrixJosephFlowExactly)
{
    Ekf ekf_a{belief_with_velocity(0.4, 0.3), default_noise()};
    ekf_a.predict(0.01);

    // 快照 A 预测后、更新前的置信, 作为手写矩阵流程的输入。
    const Vec6 mu_bar = ekf_a.mu();
    const Mat6 Sigma_bar = ekf_a.Sigma();

    constexpr double z_imu = 0.35;
    constexpr double R_imu = 0.005 * 0.005;

    // A: 标量 update_imu
    ekf_a.update_imu(z_imu, R_imu);

    // B: 一般 1×6 矩阵 Joseph form, H = e_ωᵀ + e_bᵀ
    Eigen::Matrix<double, 1, kStateDim> H = Eigen::Matrix<double, 1, kStateDim>::Zero();
    H(0, kOmega) = 1.0;
    H(0, kBiasOmega) = 1.0;

    const double y = z_imu - (H * mu_bar)(0);
    const double S = (H * Sigma_bar * H.transpose())(0) + R_imu;
    const Eigen::Matrix<double, kStateDim, 1> K = Sigma_bar * H.transpose() / S;

    Vec6 mu_b = mu_bar + K * y;
    mu_b(kTheta) = wrap_angle(mu_b(kTheta));

    const Mat6 IKH = Mat6::Identity() - K * H;
    Mat6 Sigma_b = IKH * Sigma_bar * IKH.transpose() + (K * K.transpose()) * R_imu;
    Sigma_b = 0.5 * (Sigma_b + Sigma_b.transpose());

    EXPECT_TRUE(ekf_a.mu().isApprox(mu_b, 1e-9))
        << "scalar μ:  " << ekf_a.mu().transpose() << '\n'
        << "matrix μ:  " << mu_b.transpose();
    EXPECT_TRUE(ekf_a.Sigma().isApprox(Sigma_b, 1e-9))
        << "scalar Σ:\n" << ekf_a.Sigma() << '\n'
        << "matrix Σ:\n" << Sigma_b;
}
