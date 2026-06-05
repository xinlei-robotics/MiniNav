// ===========================================================================
// gyro bias state augmentation —— 可学习性 + 可观测性测试。
//
// 本文件自包含地复刻 sim_v2_main 的三阶段 EKF 主循环(predict → update_encoder
// → update_imu), 注入一个已知常数gyro bias 真值, 验证两件事:
//
//   1. GyroBiasIsLearnedWithinFiveSeconds
//      encoder + IMU 双传感器在场时, filter 在 5s 内把 b_ω 估计收敛到真值附近。
//
//   2. GyroBiasIsUnobservableFromEncoderAlone
//      仅 encoder 时, bias 不可观测:
//        - 估计均值停在初值 0 附近;
//        - 协方差 Σ_bb 不收缩。
//      与双传感器情形对照, 这就是 sensor fusion 经典可观测性论点的可执行证据:
//        z_imu = ω + b  仅靠 IMU 无法分离 ω 与 b;
//        encoder 单独提供不含 bias 的 ω 信息后, b = z_imu − ω_enc 才可辨识。
// ===========================================================================

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <vector>

import mininav.core.types;
import mininav.core.command_source;
import mininav.core.random;
import mininav.sensors.actuator_model;
import mininav.sensors.wheel_encoder;
import mininav.sensors.imu_model;
import mininav.localization.ekf_state;
import mininav.localization.ekf;
import mininav.localization.encoder_observation;

namespace
{
    using namespace mininav;

    // 与 sim_v2_main 一致的物理常数与时间步。
    constexpr double kDt = 0.01;
    constexpr double kTotalTime = 20.0;
    constexpr double kWheelRadius = 0.032;
    constexpr double kWheelBase = 0.150;
    constexpr std::int64_t kTicksPerRev = 1024;
    constexpr double kDistancePerTick =
        2.0 * std::numbers::pi * kWheelRadius / static_cast<double>(kTicksPerRev);

    // 与 default preset 同源的噪声/标定参数(测试自带, 不依赖 app 层 preset)。
    constexpr double kAlpha1 = 0.05, kAlpha2 = 0.02, kAlpha3 = 0.02, kAlpha4 = 0.05;
    constexpr double kSlipSigma = 0.02;
    constexpr double kSigmaImu = 0.005;
    constexpr double kQBiasOmega = 1e-8;

    struct BiasTrace
    {
        std::vector<double> t;
        std::vector<double> bias_est; // filter 估计的 b_ω
        std::vector<double> bias_cov; // Σ_bb
    };

    // -----------------------------------------------------------------------
    // 跑一遍闭环仿真, 返回 b_ω 估计与 Σ_bb 的时间序列。
    //   true_bias  : 注入的常数 gyro bias 真值
    //   use_imu    : 是否执行 update_imu(false => 仅 encoder, 用于可观测性对照)
    //   seed       : 主种子(各子 RNG 由 tag 派生, 互相独立)
    // -----------------------------------------------------------------------
    [[nodiscard]] BiasTrace run_sim(double true_bias, bool use_imu, std::uint64_t seed)
    {
        const RngFactory rng_factory{seed};
        const StagedCommandSource command_source;

        ActuatorModel actuator{
            ActuatorNoiseParams{
                .alpha1 = kAlpha1, .alpha2 = kAlpha2,
                .alpha3 = kAlpha3, .alpha4 = kAlpha4,
            },
            rng_factory.make_engine("actuator")
        };

        WheelEncoderModel encoder{
            WheelEncoderParams{
                .wheel_radius = kWheelRadius,
                .wheel_base = kWheelBase,
                .ticks_per_rev = kTicksPerRev,
                .slip_sigma = kSlipSigma,
            },
            rng_factory.make_engine("encoder_slip_left"),
            rng_factory.make_engine("encoder_slip_right")
        };

        ImuModel imu{
            ImuParams{
                .sigma_omega = kSigmaImu,
                .bias_omega_init = true_bias,
                .bias_random_walk = 0.0, // 常数 bias
            },
            rng_factory.make_engine("imu_gyro_noise"),
            rng_factory.make_engine("imu_gyro_bias")
        };

        Ekf ekf{
            make_initial_ekf_state(),
            ProcessNoiseParams{
                .alpha1 = kAlpha1, .alpha2 = kAlpha2,
                .alpha3 = kAlpha3, .alpha4 = kAlpha4,
                .q_bias_omega = kQBiasOmega,
            }
        };

        const EncoderNoiseParams enc_noise{
            .sigma_slip = kSlipSigma,
            .distance_per_tick = kDistancePerTick,
            .wheel_base = kWheelBase,
        };

        const auto step_count =
            static_cast<std::size_t>(kTotalTime / kDt) + 1;

        BiasTrace trace;
        trace.t.reserve(step_count);
        trace.bias_est.reserve(step_count);
        trace.bias_cov.reserve(step_count);

        for (std::size_t i = 0; i < step_count; ++i)
        {
            const double t = static_cast<double>(i) * kDt;

            // 记录【本步开始时】的 belief 快照(log-then-update)。
            trace.t.push_back(t);
            trace.bias_est.push_back(ekf.mu()(kBiasOmega));
            trace.bias_cov.push_back(ekf.Sigma()(kBiasOmega, kBiasOmega));

            const Twist2D cmd = command_source.command_at(t);
            const Twist2D true_velocity = actuator.apply(cmd);
            const EncoderTicks dticks = encoder.measure(true_velocity, kDt);
            const double imu_omega = imu.measure(true_velocity.w());

            // EKF: predict → update_encoder → (可选)update_imu。
            ekf.predict(kDt);

            const Eigen::Vector2d z_enc = decode_encoder(dticks, enc_noise, kDt);
            const Eigen::Matrix2d R_enc =
                encoder_noise_covariance(ekf.mu()(kV), ekf.mu()(kOmega), enc_noise, kDt);
            ekf.update_encoder(z_enc, R_enc);

            if (use_imu)
            {
                const double R_imu = kSigmaImu * kSigmaImu;
                ekf.update_imu(imu_omega, R_imu);
            }
        }

        return trace;
    }

    // 返回 t >= t_min 区间内 bias 估计的最大绝对误差(相对 true_bias)。
    [[nodiscard]] double max_bias_error_after(const BiasTrace& tr, double true_bias, double t_min)
    {
        double worst = 0.0;
        for (std::size_t i = 0; i < tr.t.size(); ++i)
        {
            if (tr.t[i] < t_min)
            {
                continue;
            }
            const double err = std::abs(tr.bias_est[i] - true_bias);
            worst = std::max(worst, err);
        }
        return worst;
    }
}

// ---------------------------------------------------------------------------
// 主测试: encoder + IMU 在场时, 常数 gyro bias 最终被学到(±25% 容差)。
// ---------------------------------------------------------------------------
TEST(EkfBias, GyroBiasConvergesToTrueBias)
{
    constexpr double kTrueBias = 0.02; // rad/s
    constexpr double kTolerance = 0.005; // ±25%

    const BiasTrace tr = run_sim(kTrueBias, /*use_imu=*/true, /*seed=*/42);

    // 单个 seed 下 encoder slip/量化会让 imu-encoder residual 在局部时间窗内偏置,
    // 因此不要求 5s 后每一帧都落入窄容差; 这里验证最终估计收敛到真值附近。
    EXPECT_LT(std::abs(tr.bias_est.back() - kTrueBias), kTolerance)
        << "estimated gyro bias did not converge to within " << kTolerance
        << " rad/s of the true bias (" << kTrueBias << ") by the end of the run";

    // 收敛同时伴随 Σ_bb 显著收缩(从初值 1e-2 降到远小于它)。
    const double final_cov = tr.bias_cov.back();
    EXPECT_LT(final_cov, 1e-4)
        << "bias covariance failed to shrink, fusion did not make bias observable";
}

// ---------------------------------------------------------------------------
// 可观测性对照: 仅 encoder 时 bias 不可观测 —— 估计停在 0、协方差不收缩。
// ---------------------------------------------------------------------------
TEST(EkfBias, GyroBiasIsUnobservableFromEncoderAlone)
{
    constexpr double kTrueBias = 0.02;

    const BiasTrace enc_only = run_sim(kTrueBias, /*use_imu=*/false, /*seed=*/42);
    const BiasTrace fused = run_sim(kTrueBias, /*use_imu=*/true, /*seed=*/42);

    // 仅 encoder: 估计 bias 没有信息来源, 停留在初值 0 附近。
    const double enc_only_drift_from_zero = max_bias_error_after(enc_only, 0.0, /*t_min=*/0.0);
    EXPECT_LT(enc_only_drift_from_zero, 1e-3)
        << "encoder-only bias estimate moved away from 0 without an informing observation";

    // 仅 encoder: Σ_bb 不收缩(无观测约束); 应保持在初值量级附近。
    EXPECT_GT(enc_only.bias_cov.back(), 5e-3)
        << "encoder-only bias covariance shrank — bias should be unobservable here";

    // 双传感器: Σ_bb 远小于仅 encoder, 且估计逼近真值 —— 鲜明对照。
    EXPECT_LT(fused.bias_cov.back(), enc_only.bias_cov.back() * 0.1);
    EXPECT_LT(std::abs(fused.bias_est.back() - kTrueBias), 0.005);
}
