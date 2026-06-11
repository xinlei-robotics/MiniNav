module;

#include <Eigen/Core>

module mininav.localization.encoder_observation;

import mininav.core.types;
import mininav.core.kinematics;

namespace mininav
{
    Eigen::Vector2d decode_encoder(const EncoderTicks& dticks,
                                   const EncoderNoiseParams& params,
                                   const double dt) noexcept
    {
        const Twist2D twist =
            twist_from_ticks(dticks, params.distance_per_tick, params.wheel_base, dt);
        return Eigen::Vector2d{twist.v(), twist.w()};
    }

    Eigen::Matrix2d encoder_noise_covariance(const double v, const double omega,
                                             const EncoderNoiseParams& params,
                                             const double dt) noexcept
    {
        // 体速度 → 轮速。
        const auto [v_l, v_r] = inverse_kinematics(Twist2D{v, omega}, params.wheel_base);

        // 乘性打滑
        const double slip_var = params.sigma_slip * params.sigma_slip;
        // 量化floor
        const double quant_floor =
            (params.distance_per_tick * params.distance_per_tick / 12.0) / (dt * dt);

        // 单轮速度方差: σ² = v_wheel²·σ_slip² + 量化floor
        const double sigma_l2 = v_l * v_l * slip_var + quant_floor;
        const double sigma_r2 = v_r * v_r * slip_var + quant_floor;

        //  经 forward_kinematics Jacobian 传播: R = J diag(σ_L², σ_R²) Jᵀ。
        const double W = params.wheel_base;
        Eigen::Matrix2d R;
        R(0, 0) = (sigma_l2 + sigma_r2) / 4.0;
        R(0, 1) = (sigma_r2 - sigma_l2) / (2.0 * W);
        R(1, 0) = R(0, 1);
        R(1, 1) = (sigma_l2 + sigma_r2) / (W * W);
        return R;
    }
}