// ===========================================================================
// Ekf::update_encoder 行为测试 —— encoder 作为第一个观测。
//
//   1. EkfEncoderOnlyTracksWheelOdometry —— EKF-encoder-only ≈ wheel odometry
//      (encoder 是唯一信息源时, EKF 退化为该传感器的开环积分)。
//   2. DegeneratesToOdometryAsNoiseVanishes —— R→0、直行时退化到亚毫米级,
//      干净展示退化机制(残差是 predict-then-update 的一步速度滞后)。
//   3. VelocityCovarianceIsBounded —— 速度块协方差有界(被 encoder 直接观测)。
//   4. PositionCovarianceStillGrows —— 位置块协方差无界增长。encoder-only EKF
//      就是 dead reckoning: 位置/航向从未被直接观测, 只能从速度积分。
//   5. JosephUpdatePreservesSpd —— 每步 update 后 Σ 仍 symmetric PSD。
//
// 测试共用一个轻量仿真 harness: 给定一条 tick 流, 同时跑 wheel odometry 和
// EKF(predict + encoder update)。
// ===========================================================================
#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

import mininav.localization.ekf;
import mininav.localization.ekf_state;
import mininav.localization.encoder_observation;
import mininav.localization.wheel_odometry;
import mininav.core.types;
import mininav.core.kinematics;
import mininav.core.math;

using namespace mininav;

namespace
{
    constexpr double kWheelRadius = 0.032;
    constexpr double kWheelBase = 0.150;
    constexpr std::int64_t kTicksPerRev = 1024;
    constexpr double kDistancePerTick =
        2.0 * kPi * kWheelRadius / static_cast<double>(kTicksPerRev);
    constexpr double kDt = 0.01;

    constexpr double kAlpha1 = 0.05, kAlpha2 = 0.02, kAlpha3 = 0.02, kAlpha4 = 0.05;
    constexpr double kSigmaSlip = 0.02;

    EncoderNoiseParams enc_params()
    {
        return EncoderNoiseParams{
            .sigma_slip = kSigmaSlip,
            .distance_per_tick = kDistancePerTick,
            .wheel_base = kWheelBase,
        };
    }

    // 用乘性打滑模型生成一条 tick 流(镜像 WheelEncoderModel 的核心逻辑)。
    // 返回每步的 EncoderTicks delta。
    std::vector<EncoderTicks> generate_ticks(double v_cmd, double w_cmd,
                                             int steps, bool turn_after_half,
                                             std::uint32_t seed)
    {
        std::mt19937 rng{seed};
        std::normal_distribution<double> slip{0.0, kSigmaSlip};

        std::vector<EncoderTicks> out;
        out.reserve(static_cast<std::size_t>(steps));

        double s_left = 0.0, s_right = 0.0;
        std::int64_t prev_l = 0, prev_r = 0;

        for (int i = 0; i < steps; ++i)
        {
            const double t = static_cast<double>(i) * kDt;
            const double w = (turn_after_half && t >= static_cast<double>(steps) * kDt * 0.5)
                                 ? w_cmd
                                 : 0.0;
            const auto [vl_t, vr_t] = inverse_kinematics(Twist2D{v_cmd, w}, kWheelBase);
            const double vl_m = vl_t * (1.0 + slip(rng));
            const double vr_m = vr_t * (1.0 + slip(rng));

            s_left += vl_m * kDt;
            s_right += vr_m * kDt;

            const auto tl = static_cast<std::int64_t>(std::llround(s_left / kDistancePerTick));
            const auto tr = static_cast<std::int64_t>(std::llround(s_right / kDistancePerTick));

            out.push_back(EncoderTicks{.left = tl - prev_l, .right = tr - prev_r});
            prev_l = tl;
            prev_r = tr;
        }
        return out;
    }

    Ekf make_ekf()
    {
        return Ekf{
            make_initial_ekf_state(),
            ProcessNoiseParams{
                .alpha1 = kAlpha1, .alpha2 = kAlpha2, .alpha3 = kAlpha3, .alpha4 = kAlpha4,
            }
        };
    }
}

// --- 1. 退化为 wheel odometry --------------------------------------------
TEST(EkfEncoderUpdate, EkfEncoderOnlyTracksWheelOdometry)
{
    const auto ticks = generate_ticks(0.5, 0.4, 2000, /*turn=*/true, /*seed=*/7);
    const EncoderNoiseParams p = enc_params();

    Ekf ekf = make_ekf();
    WheelOdometry odom{
        WheelOdometryParams{.wheel_base = kWheelBase, .distance_per_tick = kDistancePerTick},
        Pose2D{0.0, 0.0, 0.0}
    };

    double max_pose_diff = 0.0;
    for (const auto& dticks : ticks)
    {
        const Pose2D odom_pose = odom.update(dticks, kDt);

        ekf.predict(kDt);
        const Eigen::Vector2d z = decode_encoder(dticks, p, kDt);
        const Eigen::Matrix2d R = encoder_noise_covariance(ekf.mu()(kV), ekf.mu()(kOmega), p, kDt);
        ekf.update_encoder(z, R);

        const double d = std::hypot(ekf.mu()(kPx) - odom_pose.x(),
                                    ekf.mu()(kPy) - odom_pose.y());
        max_pose_diff = std::max(max_pose_diff, d);
    }

    // 实测 ~1cm(残差 = 一步速度滞后)。2cm 阈值留足裕量。
    EXPECT_LT(max_pose_diff, 2e-2)
        << "EKF-encoder-only 应数值上贴合 wheel odometry; 偏差过大说明 update 实现有误";
}

// --- 2. R→0 极限下退化更紧 -----------------------------------------------
TEST(EkfEncoderUpdate, DegeneratesToOdometryAsNoiseVanishes)
{
    // 直行(无转弯), 避免转弯段的滞后放大, 突出"R→0 → 退化为 odom"的机制。
    const auto ticks = generate_ticks(0.5, 0.0, 1000, /*turn=*/false, /*seed=*/7);
    const EncoderNoiseParams p = enc_params();

    Ekf ekf = make_ekf();
    WheelOdometry odom{
        WheelOdometryParams{.wheel_base = kWheelBase, .distance_per_tick = kDistancePerTick},
        Pose2D{0.0, 0.0, 0.0}
    };

    double final_diff = 0.0;
    for (const auto& dticks : ticks)
    {
        const Pose2D odom_pose = odom.update(dticks, kDt);

        ekf.predict(kDt);
        const Eigen::Vector2d z = decode_encoder(dticks, p, kDt);
        // 人为把 R 缩到极小(×1e-4): 滤波器几乎完全相信 encoder。
        const Eigen::Matrix2d R =
            1e-4 * encoder_noise_covariance(ekf.mu()(kV), ekf.mu()(kOmega), p, kDt);
        ekf.update_encoder(z, R);

        final_diff = std::hypot(ekf.mu()(kPx) - odom_pose.x(),
                                ekf.mu()(kPy) - odom_pose.y());
    }

    // 实测 ~1e-4 m。亚毫米阈值 1e-3 m。
    EXPECT_LT(final_diff, 1e-3)
        << "R→0 时 EKF 应退化为 wheel odometry 的开环积分(残差仅一步速度滞后)";
}

// --- 3. 速度块协方差有界 -------------------------------------------------
TEST(EkfEncoderUpdate, VelocityCovarianceIsBounded)
{
    const auto ticks = generate_ticks(0.5, 0.0, 2000, /*turn=*/false, /*seed=*/7);
    const EncoderNoiseParams p = enc_params();
    Ekf ekf = make_ekf();

    double vel_trace_at_10s = 0.0, vel_trace_at_20s = 0.0;
    for (int i = 0; i < static_cast<int>(ticks.size()); ++i)
    {
        ekf.predict(kDt);
        const Eigen::Vector2d z = decode_encoder(ticks[static_cast<std::size_t>(i)], p, kDt);
        const Eigen::Matrix2d R = encoder_noise_covariance(ekf.mu()(kV), ekf.mu()(kOmega), p, kDt);
        ekf.update_encoder(z, R);

        const double vel_trace = ekf.Sigma()(kV, kV) + ekf.Sigma()(kOmega, kOmega);
        if (i == 999) vel_trace_at_10s = vel_trace;
        if (i == 1999) vel_trace_at_20s = vel_trace;
    }

    // encoder 直接观测 (v, ω) → 速度块达稳态。两个时刻应几乎相等。
    EXPECT_LT(vel_trace_at_20s, 1.5 * vel_trace_at_10s)
        << "速度块协方差应有界(稳态); 持续增长说明 update 没有正确约束速度";
}

// --- 4. 位置块协方差仍发散(observability 论点) ---------------------------
TEST(EkfEncoderUpdate, PositionCovarianceStillGrows)
{
    const auto ticks = generate_ticks(0.5, 0.0, 2000, /*turn=*/false, /*seed=*/7);
    const EncoderNoiseParams p = enc_params();
    Ekf ekf = make_ekf();

    double pos_trace_at_5s = 0.0, pos_trace_at_20s = 0.0;
    for (int i = 0; i < static_cast<int>(ticks.size()); ++i)
    {
        ekf.predict(kDt);
        const Eigen::Vector2d z = decode_encoder(ticks[static_cast<std::size_t>(i)], p, kDt);
        const Eigen::Matrix2d R = encoder_noise_covariance(ekf.mu()(kV), ekf.mu()(kOmega), p, kDt);
        ekf.update_encoder(z, R);

        const double pos_trace = ekf.Sigma()(kPx, kPx) + ekf.Sigma()(kPy, kPy);
        if (i == 499) pos_trace_at_5s = pos_trace;
        if (i == 1999) pos_trace_at_20s = pos_trace;
    }

    // 位置/航向从未被直接观测 → dead reckoning → 位置块无界增长。
    // 实测 ~65×。阈值 5× 留足裕量, 但仍清晰区别于"有界"。
    EXPECT_GT(pos_trace_at_20s, 5.0 * pos_trace_at_5s)
        << "位置块协方差应持续增长(encoder-only = dead reckoning, 位置不可观测)";
}

// --- 5. Joseph form 保持 SPD ---------------------------------------------
TEST(EkfEncoderUpdate, JosephUpdatePreservesSpd)
{
    const auto ticks = generate_ticks(0.5, 0.4, 2000, /*turn=*/true, /*seed=*/11);
    const EncoderNoiseParams p = enc_params();
    Ekf ekf = make_ekf();

    for (int i = 0; i < static_cast<int>(ticks.size()); ++i)
    {
        ekf.predict(kDt);
        const Eigen::Vector2d z = decode_encoder(ticks[static_cast<std::size_t>(i)], p, kDt);
        const Eigen::Matrix2d R = encoder_noise_covariance(ekf.mu()(kV), ekf.mu()(kOmega), p, kDt);
        ekf.update_encoder(z, R);

        ASSERT_TRUE(is_symmetric_positive_definite(ekf.Sigma(), 1e-9))
            << "Σ 在第 " << i << " 步 encoder update 后丢失 symmetric-PSD 性质";
    }
}

// --- 单步 update 的基本正确性 --------------------------------------------
TEST(EkfEncoderUpdate, SingleUpdateMovesVelocityTowardMeasurementAndShrinksCovariance)
{
    Ekf ekf = make_ekf();
    ekf.predict(kDt); // 让速度块协方差略微增长, 给 update 留收缩空间

    const double v_before = ekf.mu()(kV);
    const double cov_v_before = ekf.Sigma()(kV, kV);

    // 观测一个明显非零的速度。
    const Eigen::Vector2d z{0.5, 0.0};
    const Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 1e-4;
    ekf.update_encoder(z, R);

    // 均值向观测移动, 速度协方差被观测收缩。
    EXPECT_GT(ekf.mu()(kV), v_before);
    EXPECT_LT(ekf.mu()(kV), 0.5 + 1e-9);
    EXPECT_LT(ekf.Sigma()(kV, kV), cov_v_before);
}