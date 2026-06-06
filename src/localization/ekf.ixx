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
    // Integrator: 过程模型 g 的数值积分方法。
    //
    //   Rk4   —— 默认/生产路径; 与仿真真值 differential_drive_step 共用同一个
    //            rk4_step (见 ekf_integrator_consistency_tests)。
    //   Euler —— 一阶前向欧拉; 保留它纯粹是为了 RK4-vs-Euler 的归因实验
    //            (analyze_integrator.py), 让"积分器对估计精度的影响"可被
    //            单独量化。两条路径的解析 Jacobian 都由有限差分测试守护。
    // ---------------------------------------------------------------------------
    enum class Integrator
    {
        Euler,
        Rk4,
    };

    // ---------------------------------------------------------------------------
    // 过程模型 g: 6D 常速(constant-velocity)运动模型 + gyro bias 随机游走。
    // (v, ω, b_ω) 为恒等映射, θ 推进 ω·dt(精确, wrap 留给 predict())。
    // 位置 (px, py) 的推进取决于 integrator:
    //
    //   Rk4   : 一步内 (v,ω) 常量 ⇒ RK4 塌缩为 Simpson 求积
    //           px += (dt/6)·v·[cosθ + 4·cos(θ+½ωdt) + cos(θ+ωdt)]   (py 用 sin)
    //   Euler : px += v·cosθ·dt,  py += v·sinθ·dt
    // ---------------------------------------------------------------------------
    [[nodiscard]] Vec6 process_model_g(const Vec6& mu, double dt,
                                       Integrator integrator = Integrator::Rk4) noexcept;

    // ---------------------------------------------------------------------------
    // 解析 Jacobian G = ∂g/∂x。仅 px、py 两行随 integrator 变化; 其余为单位阵
    // (px,py 对自身=1; v,ω,b_ω 行恒等), 且恒有 G(θ,ω)=dt。
    //
    //   Rk4 (记 (c₀,cₘ,c₁)=cos(θ, θ+½ωdt, θ+ωdt), (s₀,sₘ,s₁) 同理):
    //     G(px,θ)=(dt/6)·v·(−s₀−4sₘ−s₁)  G(px,v)=(dt/6)·(c₀+4cₘ+c₁)
    //     G(px,ω)=−(dt²/6)·v·(2sₘ+s₁)     ← RK4 相对欧拉新增的 O(dt²) 耦合
    //     G(py,θ)=(dt/6)·v·(c₀+4cₘ+c₁)    G(py,v)=(dt/6)·(s₀+4sₘ+s₁)
    //     G(py,ω)=(dt²/6)·v·(2cₘ+c₁)
    //   Euler:
    //     G(px,θ)=−v·sinθ·dt  G(px,v)=cosθ·dt
    //     G(py,θ)= v·cosθ·dt  G(py,v)=sinθ·dt   (G(px,ω)=G(py,ω)=0)
    //
    // ω→0 极限下 Rk4 退回 Euler。
    // ---------------------------------------------------------------------------
    [[nodiscard]] Mat6 process_jacobian_G(const Vec6& mu, double dt,
                                          Integrator integrator = Integrator::Rk4) noexcept;

    // ---------------------------------------------------------------------------
    // 数值 Jacobian: 对 process_model_g(·, integrator) 做逐列中心差分。
    // ---------------------------------------------------------------------------
    [[nodiscard]] Mat6 numeric_process_jacobian(const Vec6& mu, double dt, double eps,
                                                Integrator integrator = Integrator::Rk4) noexcept;

    // ---------------------------------------------------------------------------
    // ProcessNoiseParams: 过程噪声 Q 的物理来源参数。
    //
    // (α₁..α₄) 来自 V1 Velocity Motion Model(Thrun §5.3), 与 ActuatorModel
    // 用的是同一组物理量。
    //
    // q_bias_omega 是 gyro bias 随机游走的方差 (rad/s)², 也是在线 bias 学习开关:
    //   - 连续时间随机游走 b(t) 的速率谱密度为 σ_rw² (单位 (rad/s)²/s) 时,
    //     离散一步(dt)的方差增量为 σ_rw²·dt, 故 q_bias_omega = σ_rw²·dt。
    //   - 取值应非常小(bias 慢漂)。
    //   - 缺省 0 → 不在线估计 bias, update_imu 退化为 z=ω 的 5D 兼容行为。
    //     这样没有显式启用 bias 的滤波器不会让 bias 状态吸收 gyro 白噪声。
    // ---------------------------------------------------------------------------
    struct ProcessNoiseParams
    {
        double alpha1{0.0};
        double alpha2{0.0};
        double alpha3{0.0};
        double alpha4{0.0};
        double q_bias_omega{0.0};
    };

    // ---------------------------------------------------------------------------
    // 过程噪声协方差 Q_process, 在状态 mu 处求值。
    //
    //   Q_vv = α₁·v² + α₂·ω²
    //   Q_ωω = α₃·v² + α₄·ω²
    //   Q_bb = q_bias_omega
    //   其余项 = 0
    // ---------------------------------------------------------------------------
    [[nodiscard]] Mat6 process_noise_Q(const Vec6& mu, const ProcessNoiseParams& params) noexcept;

    // ---------------------------------------------------------------------------
    // is_symmetric_positive_definite: Σ 数值健康检查。
    //
    // 判据:
    //   对称：max|Σ - Σᵀ| ≤ tol;
    //   正定：Cholesky(LLT) 分解成功。
    // ---------------------------------------------------------------------------
    [[nodiscard]] bool is_symmetric_positive_definite(const Mat6& M, double tol) noexcept;

    // ---------------------------------------------------------------------------
    // 初始协方差 Σ₀ = diag(1e-6, 1e-6, 1e-6, 1e-2, 1e-2, 1e-2)。
    //
    // bias 维度初值 (0.1)² = 1e-2: 表示"启动时对 gyro bias 几乎一无所知"(先验
    // 1σ ≈ 0.1 rad/s, 远大于典型 bias 量级), 让 filter 有充分空间在观测中学到它。
    // ---------------------------------------------------------------------------
    [[nodiscard]] Mat6 default_initial_covariance() noexcept;

    // ---------------------------------------------------------------------------
    // make_initial_ekf_state: μ₀ = 0 (无信息先验, 含 b_ω₀ = 0),
    //                         Σ₀ = default_initial_covariance()。
    // ---------------------------------------------------------------------------
    [[nodiscard]] EkfState6 make_initial_ekf_state() noexcept;

    // ===========================================================================
    // Ekf: 6D 扩展卡尔曼滤波器
    // ===========================================================================
    class Ekf
    {
    public:
        // integrator 缺省 Rk4(生产路径); 传 Euler 仅用于归因实验。
        Ekf(EkfState6 initial_state, ProcessNoiseParams noise,
            Integrator integrator = Integrator::Rk4) noexcept
            : state_{std::move(initial_state)}, noise_{noise}, integrator_{integrator}
        {
        }

        [[nodiscard]] Integrator integrator() const noexcept { return integrator_; }

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
        //   H = [[0, 0, 0, 1, 0, 0],
        //        [0, 0, 0, 0, 1, 0]]
        //
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
        //
        // 返回值: 本次更新的 NIS(Normalized Innovation Squared) = yᵀ·S⁻¹·y。
        // 用于 filter consistency 诊断 —— 理论上服从自由度 2 的 χ² 分布。
        // -----------------------------------------------------------------------
        double update_encoder(const Eigen::Vector2d& z, const Eigen::Matrix2d& R_meas);

        // -----------------------------------------------------------------------
        // update_imu: IMU观测的 Kalman update(含 bias)。
        //
        // 测量模型(PR4):
        //   z_imu = ω + b_ω + N(0, R_imu)
        //   H_imu = [0, 0, 0, 0, 1, 1]
        //
        // Kalman update (利用 Hᵀ = e_ω + e_b):
        //   ẑ = μ̄(ω) + μ̄(b)
        //   Σ̄·Hᵀ = Σ̄.col(ω) + Σ̄.col(b)
        //   S = H·(Σ̄·Hᵀ) + R_imu = (Σ̄·Hᵀ)(ω) + (Σ̄·Hᵀ)(b) + R_imu
        //   K = (Σ̄·Hᵀ) / S
        //   μ = μ̄ + K·(z − ẑ)
        //   Σ = (I−K·H)·Σ̄·(I−K·H)ᵀ + K·R·Kᵀ
        //
        // 返回值: 本次更新的 NIS = y²/S(标量观测)。理论上服从自由度 1 的 χ² 分布。
        // -----------------------------------------------------------------------
        double update_imu(double z, double R_imu);

        [[nodiscard]] const EkfState6& state() const noexcept { return state_; }
        [[nodiscard]] const Vec6& mu() const noexcept { return state_.mu; }
        [[nodiscard]] const Mat6& Sigma() const noexcept { return state_.Sigma; }

    private:
        EkfState6 state_;
        ProcessNoiseParams noise_;
        Integrator integrator_{Integrator::Rk4};
    };
}
