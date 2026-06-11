// ===========================================================================
// 双传感器融合的 RMSE 必须显著优于单传感器。
//
// 端到端跑三个估计器在同一 truth 上的全程 RMSE:
//   - odom
//   - ekf-encoder-only
//   - ekf-encoder+imu
//
// 阈值(20s 直行+转弯轨迹):
//   - yaw RMSE : ≥ 60% 改善
//   - pos RMSE  : ≥ 50% 改善
//
// ===========================================================================
#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <random>

import mininav.core.types;
import mininav.core.kinematics;
import mininav.core.math;
import mininav.sensors.actuator_model;
import mininav.sensors.wheel_encoder;
import mininav.sensors.imu_model;
import mininav.localization.wheel_odometry;
import mininav.localization.ekf;
import mininav.localization.ekf_state;
import mininav.localization.encoder_observation;

using namespace mininav; using namespace mininav::ekf;

namespace
{
    constexpr double kWheelRadius = 0.032;
    constexpr double kWheelBase = 0.150;
    constexpr std::int64_t kTicksPerRev = 1024;
    constexpr double kDistancePerTick =
        2.0 * kPi * kWheelRadius / static_cast<double>(kTicksPerRev);
    constexpr double kDt = 0.01;
    constexpr double kDuration = 20.0;

    constexpr double kAlpha1 = 0.05, kAlpha2 = 0.02, kAlpha3 = 0.02, kAlpha4 = 0.05;
    constexpr double kSigmaSlip = 0.02;
    constexpr double kSigmaImu = 0.005;

    EncoderNoiseParams enc_params()
    {
        return EncoderNoiseParams{
            .sigma_slip = kSigmaSlip,
            .distance_per_tick = kDistancePerTick,
            .wheel_base = kWheelBase,
        };
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

    struct Rmse
    {
        double position;
        double yaw;
    };

    struct ThreeRmse
    {
        Rmse odom_only;
        Rmse ekf_encoder_only;
        Rmse ekf_fused;
    };

    // 端到端跑三个估计器在同一 truth 上, 用同一 master seed 派生独立子流。
    ThreeRmse run_three_estimators(std::uint32_t seed)
    {
        std::mt19937 slip_l_rng{seed ^ 0x12345u};
        std::mt19937 slip_r_rng{seed ^ 0x67890u};
        std::mt19937 imu_rng{seed ^ 0xABCDEu};

        ActuatorModel actuator{
            ActuatorNoiseParams{
                .alpha1 = kAlpha1, .alpha2 = kAlpha2, .alpha3 = kAlpha3, .alpha4 = kAlpha4,
            },
            std::mt19937{seed}
        };
        WheelEncoderModel encoder{
            WheelEncoderParams{
                .wheel_radius = kWheelRadius,
                .wheel_base = kWheelBase,
                .ticks_per_rev = kTicksPerRev,
                .slip_sigma = kSigmaSlip,
            },
            std::move(slip_l_rng), std::move(slip_r_rng)
        };
        ImuModel imu{ImuParams{.sigma_omega = kSigmaImu}, std::move(imu_rng)};

        WheelOdometry odom{
            WheelOdometryParams{.wheel_base = kWheelBase, .distance_per_tick = kDistancePerTick},
            Pose2D{0.0, 0.0, 0.0}
        };
        Ekf ekf_enc = make_ekf();
        Ekf ekf_fused = make_ekf();
        const EncoderNoiseParams enc_n = enc_params();

        Pose2D truth{0.0, 0.0, 0.0};
        constexpr int N = static_cast<int>(kDuration / kDt) + 1;

        double sum_p_odom = 0.0, sum_p_enc = 0.0, sum_p_fus = 0.0;
        double sum_y_odom = 0.0, sum_y_enc = 0.0, sum_y_fus = 0.0;

        for (int i = 0; i < N; ++i)
        {
            const double t = static_cast<double>(i) * kDt;
            // 10s 直行 + 10s 转弯, 与独立验证一致
            const double cv = 0.5;
            const double cw = (t < 10.0) ? 0.0 : 0.4;
            const Twist2D cmd{cv, cw};
            const Twist2D true_v = actuator.apply(cmd);
            const EncoderTicks dt_ticks = encoder.measure(true_v, kDt);
            const double imu_omega = imu.measure(true_v.w());

            // 三个估计器各自前进一步
            const Pose2D odom_pose = odom.update(dt_ticks, kDt);

            ekf_enc.predict(kDt);
            const Eigen::Vector2d z_e = decode_encoder(dt_ticks, enc_n, kDt);
            const Eigen::Matrix2d R_e =
                encoder_noise_covariance(ekf_enc.mu()(kV), ekf_enc.mu()(kOmega), enc_n, kDt);
            ekf_enc.update_encoder(z_e, R_e);

            ekf_fused.predict(kDt);
            const Eigen::Vector2d z_e2 = decode_encoder(dt_ticks, enc_n, kDt);
            const Eigen::Matrix2d R_e2 = encoder_noise_covariance(
                ekf_fused.mu()(kV), ekf_fused.mu()(kOmega), enc_n, kDt);
            ekf_fused.update_encoder(z_e2, R_e2);
            ekf_fused.update_imu(imu_omega, kSigmaImu * kSigmaImu);

            // 误差累计 (RMSE 用 truth 当前位姿)
            const double dx_o = odom_pose.x() - truth.x();
            const double dy_o = odom_pose.y() - truth.y();
            const double dy_o_yaw = wrap_angle(odom_pose.yaw() - truth.yaw());
            sum_p_odom += dx_o * dx_o + dy_o * dy_o;
            sum_y_odom += dy_o_yaw * dy_o_yaw;

            const double dx_e = ekf_enc.mu()(kPx) - truth.x();
            const double dy_e = ekf_enc.mu()(kPy) - truth.y();
            const double dy_e_yaw = wrap_angle(ekf_enc.mu()(kTheta) - truth.yaw());
            sum_p_enc += dx_e * dx_e + dy_e * dy_e;
            sum_y_enc += dy_e_yaw * dy_e_yaw;

            const double dx_f = ekf_fused.mu()(kPx) - truth.x();
            const double dy_f = ekf_fused.mu()(kPy) - truth.y();
            const double dy_f_yaw = wrap_angle(ekf_fused.mu()(kTheta) - truth.yaw());
            sum_p_fus += dx_f * dx_f + dy_f * dy_f;
            sum_y_fus += dy_f_yaw * dy_f_yaw;

            // 推进 truth
            truth = differential_drive_step(truth, true_v, kDt);
        }

        const auto n = static_cast<double>(N);
        return ThreeRmse{
            .odom_only = {std::sqrt(sum_p_odom / n), std::sqrt(sum_y_odom / n)},
            .ekf_encoder_only = {std::sqrt(sum_p_enc / n), std::sqrt(sum_y_enc / n)},
            .ekf_fused = {std::sqrt(sum_p_fus / n), std::sqrt(sum_y_fus / n)},
        };
    }
}

// --- 单 seed 的 RMSE 改善 -------------------------------------

TEST(EkfFusion, FusionImprovesRmseOverEncoderOnly_SingleSeed)
{
    const ThreeRmse r = run_three_estimators(7);

    // 阈值远松于实测(实测 ~85% 改善), 留足抗 seed 抖动裕量。
    EXPECT_LT(r.ekf_fused.yaw, r.ekf_encoder_only.yaw * 0.4)
        << "fused yaw RMSE 应 < 40% of encoder-only (实测 ~15%); 实际 ratio = "
        << (r.ekf_fused.yaw / r.ekf_encoder_only.yaw);

    EXPECT_LT(r.ekf_fused.position, r.ekf_encoder_only.position * 0.5)
        << "fused position RMSE 应 < 50% of encoder-only (实测 ~16%); 实际 ratio = "
        << (r.ekf_fused.position / r.ekf_encoder_only.position);
}

// --- 灵魂测试: 多 seed 平均后的 RMSE 改善 -------------------------------

TEST(EkfFusion, FusionImprovesRmseOverEncoderOnly_AveragedAcrossSeeds)
{
    constexpr int kSeeds = 8;
    double sum_ratio_yaw = 0.0, sum_ratio_pos = 0.0;

    for (std::uint32_t seed = 1; seed <= kSeeds; ++seed)
    {
        const ThreeRmse r = run_three_estimators(seed);
        sum_ratio_yaw += r.ekf_fused.yaw / r.ekf_encoder_only.yaw;
        sum_ratio_pos += r.ekf_fused.position / r.ekf_encoder_only.position;
    }
    const double avg_ratio_yaw = sum_ratio_yaw / kSeeds;
    const double avg_ratio_pos = sum_ratio_pos / kSeeds;

    // 实测平均 0.148 / 0.158, 阈值 0.3 留 2× 裕量。
    EXPECT_LT(avg_ratio_yaw, 0.3)
        << "averaged across " << kSeeds << " seeds, fused/enc yaw ratio = " << avg_ratio_yaw;
    EXPECT_LT(avg_ratio_pos, 0.3)
        << "averaged across " << kSeeds << " seeds, fused/enc pos ratio = " << avg_ratio_pos;
}

// --- 断言: fused 也优于 V1 odom -------------------------------------

TEST(EkfFusion, FusedEstimatorBeatsV1Odom)
{
    const ThreeRmse r = run_three_estimators(7);
    EXPECT_LT(r.ekf_fused.yaw, r.odom_only.yaw)
        << "fused yaw 应当也明显优于 V1 odom 基线";
    EXPECT_LT(r.ekf_fused.position, r.odom_only.position);
}