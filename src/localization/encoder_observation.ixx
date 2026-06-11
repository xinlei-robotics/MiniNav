module;

#include <Eigen/Core>

export module mininav.localization.encoder_observation;

import mininav.core.types;

export namespace mininav
{
    // ===========================================================================
    // Encoder observation model for the EKF.
    //
    // 该模块是将原始 EncoderTicks 解码成对隐状态 (v, ω) 的观测 z = (v̂, ω̂),
    // 并从 wheel encoder 的噪声参数推导出观测噪声协方差 R。
    //
    // WheelEncoderModel 的噪声是轮速乘性打滑：
    //   v_wheel_meas = v_wheel_true · (1 + N(0, σ_slip²))
    // 因此单轮速度方差 = v_wheel² · σ_slip²。
    // ===========================================================================

    // ---------------------------------------------------------------------------
    // EncoderNoiseParams: 推导 z 和 R 所需的全部物理量。
    //
    // 这些参数来自 V1 的 WheelEncoderParams:
    //   sigma_slip        : 乘性打滑标准差
    //   distance_per_tick : 2π·r / ticks_per_rev [m/tick]
    //   wheel_base        : 左右轮中心距 W [m]
    // ---------------------------------------------------------------------------
    struct EncoderNoiseParams
    {
        double sigma_slip{0.0};
        double distance_per_tick{0.0};
        double wheel_base{0.0};
    };

    // ---------------------------------------------------------------------------
    // decode_encoder: EncoderTicks → 观测 z = (v̂, ω̂)。
    //
    //   v_wheel = Δticks · distance_per_tick / dt
    //   (v̂, ω̂) = forward_kinematics(v_l, v_r, W)
    //
    // 返回 Eigen::Vector2d 可以直接供 Ekf::update_encoder 使用。
    // ---------------------------------------------------------------------------
    [[nodiscard]] Eigen::Vector2d decode_encoder(const EncoderTicks& dticks,
                                                 const EncoderNoiseParams& params,
                                                 double dt) noexcept;

    // ---------------------------------------------------------------------------
    // encoder_noise_covariance: 推导 2×2 观测噪声协方差 R_enc。
    //
    // 在体速度 (v, ω) 处求值(预测步后的滤波器估计; 滤波器自有信息, 不作弊):
    //   1. 反解到轮速: (v_l, v_r) = inverse_kinematics(v, ω, W)
    //   2. 单轮速度方差(乘性打滑 + 量化 floor):
    //        σ_L² = v_l²·σ_slip² + q,   σ_R² = v_r²·σ_slip² + q
    //      量化 floor q = (distance_per_tick² / 12) / dt²:
    //        encoder 用 llround 量化, ±0.5 tick 均匀量化方差 = Δ²/12 (弧长),
    //        除以 dt² 转成速度方差。它既物理真实, 又防止 v=ω=0 时 R=0 退化。
    //   3. 经 forward_kinematics Jacobian J 传播:
    //        R = J · diag(σ_L², σ_R²) · Jᵀ
    //          = [[ (σ_L²+σ_R²)/4 ,  (σ_R²−σ_L²)/(2W) ],
    //             [ (σ_R²−σ_L²)/(2W),  (σ_L²+σ_R²)/W²  ]]
    // ---------------------------------------------------------------------------
    [[nodiscard]] Eigen::Matrix2d encoder_noise_covariance(double v, double omega,
                                                           const EncoderNoiseParams& params,
                                                           double dt) noexcept;
}
