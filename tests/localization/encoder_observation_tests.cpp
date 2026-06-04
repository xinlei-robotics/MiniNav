// ===========================================================================
// encoder observation model 测试。
//   1. decode_encoder —— ticks → (v̂, ω̂) 与 forward_kinematics 一致。
//   2. encoder_noise_covariance —— R_enc 推导正确:
//        - 直行(v_l = v_r) 时 off-diagonal = 0(v 与 ω 观测不相关)
//        - 量化 floor 在 v=ω=0 时仍给出正定 R(防退化)
//        - 解析 R 与"对乘性打滑做协方差传播"的蒙特卡洛估计一致
// ===========================================================================
#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <random>

import mininav.localization.encoder_observation;
import mininav.core.types;
import mininav.core.kinematics;

using namespace mininav;

namespace
{
    constexpr double kWheelRadius = 0.032;
    constexpr double kWheelBase = 0.150;
    constexpr std::int64_t kTicksPerRev = 1024;
    constexpr double kPiLocal = 3.14159265358979323846;
    constexpr double kDistancePerTick =
        2.0 * kPiLocal * kWheelRadius / static_cast<double>(kTicksPerRev);

    EncoderNoiseParams make_params(double sigma_slip)
    {
        return EncoderNoiseParams{
            .sigma_slip = sigma_slip,
            .distance_per_tick = kDistancePerTick,
            .wheel_base = kWheelBase,
        };
    }
}

// --- decode --------------------------------------------------------------

TEST(EncoderObservation, DecodeMatchesForwardKinematics)
{
    const EncoderNoiseParams p = make_params(0.0);
    constexpr double dt = 0.01;

    // 左右各 100 ticks。
    const EncoderTicks dticks{.left = 100, .right = 100};
    const Eigen::Vector2d z = decode_encoder(dticks, p, dt);

    // 直接用 forward_kinematics 复算: v_l = v_r = 100·dpt/dt → v = that, ω = 0。
    const double v_wheel = 100.0 * kDistancePerTick / dt;
    const Twist2D expected = forward_kinematics(v_wheel, v_wheel, kWheelBase);

    EXPECT_NEAR(z(0), expected.v(), 1e-12);
    EXPECT_NEAR(z(1), expected.w(), 1e-12);
    EXPECT_NEAR(z(1), 0.0, 1e-12); // 左右相等 → 纯直行
}

TEST(EncoderObservation, DecodeRecoversPureRotation)
{
    const EncoderNoiseParams p = make_params(0.0);
    constexpr double dt = 0.01;

    // 左退右进, 等量 → 原地转(v=0, ω>0)。
    const EncoderTicks dticks{.left = -50, .right = 50};
    const Eigen::Vector2d z = decode_encoder(dticks, p, dt);

    EXPECT_NEAR(z(0), 0.0, 1e-12); // v = 0
    EXPECT_GT(z(1), 0.0); // ω > 0
}

// --- R_enc ---------------------------------------------------------------

TEST(EncoderObservation, NoiseCovarianceIsSymmetricPositiveDefinite)
{
    const EncoderNoiseParams p = make_params(0.02);
    constexpr double dt = 0.01;

    for (const auto [v, w] : {std::pair{0.5, 0.0}, std::pair{0.5, 0.4},
                              std::pair{0.0, 1.0}, std::pair{-0.3, -0.2}})
    {
        const Eigen::Matrix2d R = encoder_noise_covariance(v, w, p, dt);
        EXPECT_NEAR(R(0, 1), R(1, 0), 1e-15) << "R 必须对称";
        // 正定: 行列式 > 0 且对角 > 0。
        EXPECT_GT(R(0, 0), 0.0);
        EXPECT_GT(R(1, 1), 0.0);
        EXPECT_GT(R(0, 0) * R(1, 1) - R(0, 1) * R(1, 0), 0.0);
    }
}

TEST(EncoderObservation, StraightLineDecouplesVAndOmega)
{
    const EncoderNoiseParams p = make_params(0.02);
    constexpr double dt = 0.01;

    // 直行: v_l = v_r → σ_L² = σ_R² → off-diagonal = (σ_R²−σ_L²)/(2W) = 0。
    const Eigen::Matrix2d R = encoder_noise_covariance(0.5, 0.0, p, dt);
    EXPECT_NEAR(R(0, 1), 0.0, 1e-15);
}

TEST(EncoderObservation, QuantizationFloorKeepsRPositiveDefiniteAtRest)
{
    const EncoderNoiseParams p = make_params(0.02);
    constexpr double dt = 0.01;

    // v = ω = 0: 乘性项为 0, 仅量化 floor 支撑 R。R 仍须正定(否则 S 奇异)。
    const Eigen::Matrix2d R = encoder_noise_covariance(0.0, 0.0, p, dt);
    EXPECT_GT(R(0, 0), 0.0);
    EXPECT_GT(R(1, 1), 0.0);
    EXPECT_GT(R(0, 0) * R(1, 1) - R(0, 1) * R(1, 0), 0.0);
}

// 蒙特卡洛: 直接对乘性打滑采样 → forward_kinematics → 估计 (v̂, ω̂) 的样本协方差,
// 应与解析 R 的乘性部分(去掉量化 floor)一致。这独立验证 Jacobian 传播无误。
TEST(EncoderObservation, AnalyticCovarianceMatchesMonteCarlo)
{
    constexpr double sigma_slip = 0.03;
    constexpr double dt = 0.01;
    const EncoderNoiseParams p = make_params(sigma_slip);

    constexpr double v = 0.6;
    constexpr double w = 0.5; // 一般情形, v_l ≠ v_r → off-diagonal 非零

    // 解析 R(含量化 floor); 减去 floor 得纯乘性部分以与 MC 比。
    const Eigen::Matrix2d R_full = encoder_noise_covariance(v, w, p, dt);
    const Eigen::Matrix2d R_zero_slip =
        encoder_noise_covariance(v, w, make_params(0.0), dt); // 只剩 floor
    const Eigen::Matrix2d R_mult = R_full - R_zero_slip;

    // MC: 真值轮速 → 乘性扰动 → forward_kinematics → 收集 (v̂, ω̂)。
    const auto [v_l, v_r] = inverse_kinematics(Twist2D{v, w}, kWheelBase);
    std::mt19937 rng{12345};
    std::normal_distribution<double> slip{0.0, sigma_slip};

    constexpr int N = 400000;
    double sum_v = 0.0, sum_w = 0.0, sum_vv = 0.0, sum_ww = 0.0, sum_vw = 0.0;
    for (int i = 0; i < N; ++i)
    {
        const double vl_m = v_l * (1.0 + slip(rng));
        const double vr_m = v_r * (1.0 + slip(rng));
        const Twist2D t = forward_kinematics(vl_m, vr_m, kWheelBase);
        sum_v += t.v();
        sum_w += t.w();
        sum_vv += t.v() * t.v();
        sum_ww += t.w() * t.w();
        sum_vw += t.v() * t.w();
    }
    const double n = static_cast<double>(N);
    const double mean_v = sum_v / n;
    const double mean_w = sum_w / n;
    const double cov_vv = sum_vv / n - mean_v * mean_v;
    const double cov_ww = sum_ww / n - mean_w * mean_w;
    const double cov_vw = sum_vw / n - mean_v * mean_w;

    // MC 估计有 ~O(1/√N) 噪声; 用相对容差 5%。
    EXPECT_NEAR(cov_vv, R_mult(0, 0), 0.05 * R_mult(0, 0));
    EXPECT_NEAR(cov_ww, R_mult(1, 1), 0.05 * R_mult(1, 1));
    EXPECT_NEAR(cov_vw, R_mult(0, 1), 0.05 * std::abs(R_mult(0, 1)));
}