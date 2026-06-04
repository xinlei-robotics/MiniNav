module;

#include <Eigen/Core>

#include <utility>

export module mininav.localization.ekf;

import mininav.localization.ekf_state;

export namespace mininav
{
    // ===========================================================================
    // 符号约定：
    //
    //   均值         mu ↔ μ          采用 Thrun 约定
    //   协方差        Sigma  ↔ Σ      采用 Thrun 约定
    //   过程模型       g(μ)            采用 Thrun 约定
    //   过程Jacobian G = ∂g/∂x       采用 Thrun 约定
    //   过程噪声       Q_process         采用控制工程约定
    //   观测噪声       R_meas            采用控制工程约定
    //
    // 注意：Thrun原书中将过程噪声定义为R、观测噪声定义为Q, 在此处采用控制工程约定则相反。
    // ===========================================================================

    // ---------------------------------------------------------------------------
    // 过程模型 g: 5D 常速(constant-velocity)运动模型。
    //
    //   g(x) = [ px + v·cosθ·dt
    //            py + v·sinθ·dt
    //            θ  + ω·dt
    //            v
    //            ω            ]
    //
    // 注意: 此函数返回的θ不wrap 到 (-π, π]。
    // ---------------------------------------------------------------------------
    [[nodiscard]] Vec5 process_model_g(const Vec5& mu, double dt) noexcept;

    // ---------------------------------------------------------------------------
    // 解析 Jacobian G = ∂g/∂x
    //
    //   G = | 1  0  -v·sinθ·dt   cosθ·dt   0  |
    //       | 0  1   v·cosθ·dt   sinθ·dt   0  |
    //       | 0  0      1           0      dt |
    //       | 0  0      0           1      0  |
    //       | 0  0      0           0      1  |
    // ---------------------------------------------------------------------------
    [[nodiscard]] Mat5 process_jacobian_G(const Vec5& mu, double dt) noexcept;

    // ---------------------------------------------------------------------------
    // 数值 Jacobian: 对 process_model_g 做逐列中心差分。
    // ---------------------------------------------------------------------------
    [[nodiscard]] Mat5 numeric_process_jacobian(const Vec5& mu, double dt, double eps) noexcept;

    // ---------------------------------------------------------------------------
    // ProcessNoiseParams: 过程噪声 Q 的物理来源参数。
    //
    // V1 版本中 Velocity Motion Model 的四个标定参数 (α₁..α₄, Thrun §5.3),
    // 与 ActuatorModel 用的是同一组物理量。
    // ---------------------------------------------------------------------------
    struct ProcessNoiseParams
    {
        double alpha1{0.0};
        double alpha2{0.0};
        double alpha3{0.0};
        double alpha4{0.0};
    };

    // ---------------------------------------------------------------------------
    // 过程噪声协方差 Q_process, 在状态 mu 处求值。
    //
    //   Q_vv = α₁·v² + α₂·ω²
    //   Q_ωω = α₃·v² + α₄·ω²
    //   其余项 = 0
    // ---------------------------------------------------------------------------
    [[nodiscard]] Mat5 process_noise_Q(const Vec5& mu, const ProcessNoiseParams& params) noexcept;

    // ---------------------------------------------------------------------------
    // is_symmetric_positive_definite: Σ 数值健康检查。
    //
    // 判据:
    //   对称：max|Σ - Σᵀ| ≤ tol;
    //   正定：Cholesky(LLT) 分解成功。
    // ---------------------------------------------------------------------------
    [[nodiscard]] bool is_symmetric_positive_definite(const Mat5& M, double tol) noexcept;

    // ---------------------------------------------------------------------------
    // 初始协方差 Σ₀ = diag(1e-6, 1e-6, 1e-6, 1e-2, 1e-2)。
    // ---------------------------------------------------------------------------
    [[nodiscard]] Mat5 default_initial_covariance() noexcept;

    // ---------------------------------------------------------------------------
    // make_initial_ekf_state: μ₀ = 0 (无信息先验), Σ₀ = default_initial_covariance()。
    // ---------------------------------------------------------------------------
    [[nodiscard]] EkfState5 make_initial_ekf_state() noexcept;

    // ===========================================================================
    // Ekf: 5D 扩展卡尔曼滤波器。
    // ===========================================================================
    class Ekf
    {
    public:
        Ekf(EkfState5 initial_state, ProcessNoiseParams noise) noexcept
            : state_{std::move(initial_state)}, noise_{noise}
        {
        }

        // -----------------------------------------------------------------------
        // predict: 一步预测
        //   μ̄ = g(μ)               (θ 推进后 wrap 回 (-π, π])
        //   Σ̄ = G·Σ·Gᵀ + Q         (Thrun: Σ̄ₜ = Gₜ·Σₜ₋₁·Gₜᵀ + Rₜ, 此处 R↔Q)
        // -----------------------------------------------------------------------
        void predict(double dt);

        // -----------------------------------------------------------------------
        // update_encoder: encoder 观测的 Kalman update。
        //
        // encoder 观测隐状态 (v, ω) —— 观测模型 z = H·x + 噪声, 其中
        //   H = [[0, 0, 0, 1, 0],
        //        [0, 0, 0, 0, 1]]    (选出 v、ω 两行)
        // 标准 Kalman update:
        //   y = z − H·μ̄
        //   S = H·Σ̄·Hᵀ + R_meas
        //   K = Σ̄·Hᵀ·S⁻¹
        //   μ = μ̄ + K·y       (θ wrap 回 (-π, π])
        //   Σ = (I−K·H)·Σ̄·(I−K·H)ᵀ + K·R·Kᵀ
        //
        // 参数:
        //   z      = (v̂, ω̂), 由 decode_encoder 从 EncoderTicks 解码而来
        //   R_meas = 2×2 观测噪声, 由 encoder_noise_covariance 从物理参数推导
        // EKF 在此传感器无关: 它只接受 (z, R), 不关心 z 来自 encoder 还是别处。
        // -----------------------------------------------------------------------
        void update_encoder(const Eigen::Vector2d& z, const Eigen::Matrix2d& R_meas);

        // -----------------------------------------------------------------------
        // update_imu: IMU观测的 Kalman update。
        //
        //测量模型:
        //   z_imu = ω + N(0, R_imu)
        //   H_imu = [0, 0, 0, 0, 1]
        //
        // Kalman update:
        //   y = z − μ̄(ω)
        //   S = Σ̄(ω,ω) + R_imu
        //   K = Σ̄·Hᵀ·S⁻¹
        //   μ = μ̄ + K·y
        //   Σ = (I−K·H)·Σ̄·(I−K·H)ᵀ + K·R·Kᵀ
        // -----------------------------------------------------------------------
        void update_imu(double z, double R_imu);

        [[nodiscard]] const EkfState5& state() const noexcept { return state_; }
        [[nodiscard]] const Vec5& mu() const noexcept { return state_.mu; }
        [[nodiscard]] const Mat5& Sigma() const noexcept { return state_.Sigma; }

    private:
        EkfState5 state_;
        ProcessNoiseParams noise_;
    };
}
