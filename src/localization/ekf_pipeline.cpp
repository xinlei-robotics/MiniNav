module;

#include <Eigen/Core>

module mininav.localization.ekf_pipeline;

import mininav.core.types;
import mininav.localization.ekf_state;
import mininav.localization.ekf;
import mininav.localization.encoder_observation;

namespace mininav::ekf
{
    namespace
    {
        [[nodiscard]] EkfState6 initial_state_at(const Pose2D& pose) noexcept
        {
            EkfState6 state = make_initial_ekf_state();
            state.mu(kPx) = pose.x();
            state.mu(kPy) = pose.y();
            state.mu(kTheta) = pose.yaw();
            return state;
        }
    }

    EkfPipeline::EkfPipeline(const EkfPipelineConfig& cfg, const Pose2D& initial_pose)
        : cfg_{cfg}, ekf_{initial_state_at(initial_pose), cfg.process, cfg.integrator}
    {
    }

    EkfNis EkfPipeline::step(const EncoderTicks& dticks, const double imu_omega, const double dt)
    {
        ekf_.predict(dt);

        // encoder 观测: 解码 → 在预测速度处求 R → (r_scale 旋钮) → Joseph update。
        // R 用滤波器自己的预测速度求值,而不是真值:估计器只能用它拥有的信息。
        const Eigen::Vector2d z_enc = decode_encoder(dticks, cfg_.encoder, dt);
        const Eigen::Matrix2d R_enc =
            encoder_noise_covariance(ekf_.mu()(kV), ekf_.mu()(kOmega), cfg_.encoder, dt)
            * cfg_.r_scale;
        const double nis_encoder = ekf_.update_encoder(z_enc, R_enc);

        // IMU 观测: 标量 ω + b_ω, R = σ_imu²。
        const double R_imu = cfg_.sigma_imu * cfg_.sigma_imu * cfg_.r_scale;
        const double nis_imu = ekf_.update_imu(imu_omega, R_imu);

        return EkfNis{.encoder = nis_encoder, .imu = nis_imu};
    }

    Pose2D EkfPipeline::pose() const noexcept
    {
        const Vec6& mu = ekf_.mu();
        return Pose2D{mu(kPx), mu(kPy), mu(kTheta)};
    }

    Twist2D EkfPipeline::velocity() const noexcept
    {
        const Vec6& mu = ekf_.mu();
        return Twist2D{mu(kV), mu(kOmega)};
    }
}
