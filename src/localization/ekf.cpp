module;

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>

#include <cassert>
#include <cmath>

module mininav.localization.ekf;

import mininav.localization.ekf_state;
import mininav.core.math;
import mininav.core.integrators;
import mininav.core.types;

namespace mininav::ekf
{
    Vec6 process_model_g(const Vec6& mu, const double dt, const Integrator integrator) noexcept
    {
        const double th = mu(kTheta);
        const double v = mu(kV);
        const double w = mu(kOmega);

        Vec6 out;
        if (integrator == Integrator::Rk4)
        {
            // 位置/航向与仿真真值共用同一个 RK4 积分器(单一积分器来源)。
            const Pose2D next = rk4_step(Pose2D{mu(kPx), mu(kPy), th}, Twist2D{v, w}, dt);
            out(kPx) = next.x();
            out(kPy) = next.y();
        }
        else // Integrator::Euler
        {
            out(kPx) = mu(kPx) + v * std::cos(th) * dt;
            out(kPy) = mu(kPy) + v * std::sin(th) * dt;
        }
        // (v, ω) 常量 ⇒ yaw_dot ≡ ω, θ 解精确等于 θ + ω·dt(RK4/Euler 一致)。
        out(kTheta) = th + w * dt;
        out(kV) = v;
        out(kOmega) = w;
        out(kBiasOmega) = mu(kBiasOmega);
        return out;
    }

    Mat6 process_jacobian_G(const Vec6& mu, const double dt, const Integrator integrator) noexcept
    {
        const double th = mu(kTheta);
        const double v = mu(kV);
        const double w = mu(kOmega);

        Mat6 G = Mat6::Identity();
        G(kTheta, kOmega) = dt; // θ 解精确, 与积分器无关。

        if (integrator == Integrator::Rk4)
        {
            // (v, ω) 在一步内为常量, 故 RK4 的 stage 航向只有三个取值:
            //   θ、θ+½ω·dt、θ+ω·dt。RK4 在此塌缩为对 ∫v·cos/sin(θ+ωτ)dτ 的 Simpson 求积:
            //   Δx = (dt/6)·v·[c₀ + 4·cₘ + c₁],  Δy = (dt/6)·v·[s₀ + 4·sₘ + s₁]。
            // 下面是它对状态的解析偏导(由 MatchesFiniteDifference 测试守护)。
            const double thm = th + 0.5 * w * dt;
            const double th1 = th + w * dt;

            const double c0 = std::cos(th), s0 = std::sin(th);
            const double cm = std::cos(thm), sm = std::sin(thm);
            const double c1 = std::cos(th1), s1 = std::sin(th1);

            const double sixth = dt / 6.0;
            const double dt2_sixth = dt * dt / 6.0;

            // px 行: ∂gₓ/∂{θ, v, ω}。∂gₓ/∂ω 是 RK4 相对欧拉新增的 O(dt²) 耦合。
            G(kPx, kTheta) = sixth * v * (-s0 - 4.0 * sm - s1);
            G(kPx, kV) = sixth * (c0 + 4.0 * cm + c1);
            G(kPx, kOmega) = -dt2_sixth * v * (2.0 * sm + s1);

            // py 行。
            G(kPy, kTheta) = sixth * v * (c0 + 4.0 * cm + c1);
            G(kPy, kV) = sixth * (s0 + 4.0 * sm + s1);
            G(kPy, kOmega) = dt2_sixth * v * (2.0 * cm + c1);
        }
        else // Integrator::Euler — px/py 仅依赖步首航向, 无 ∂/∂ω 耦合。
        {
            G(kPx, kTheta) = -v * std::sin(th) * dt;
            G(kPx, kV) = std::cos(th) * dt;
            G(kPy, kTheta) = v * std::cos(th) * dt;
            G(kPy, kV) = std::sin(th) * dt;
        }
        return G;
    }

    Mat6 numeric_process_jacobian(const Vec6& mu, const double dt, const double eps,
                                  const Integrator integrator) noexcept
    {
        Mat6 G_num = Mat6::Zero();
        for (int j = 0; j < kStateDim; ++j)
        {
            Vec6 plus = mu;
            Vec6 minus = mu;
            plus(j) += eps;
            minus(j) -= eps;
            const Vec6 col =
                (process_model_g(plus, dt, integrator) - process_model_g(minus, dt, integrator))
                / (2.0 * eps);
            G_num.col(j) = col;
        }
        return G_num;
    }

    Mat6 process_noise_Q(const Vec6& mu, const ProcessNoiseParams& params) noexcept
    {
        const double v = mu(kV);
        const double w = mu(kOmega);

        const double q_vv = params.alpha1 * v * v + params.alpha2 * w * w;
        const double q_ww = params.alpha3 * v * v + params.alpha4 * w * w;

        Mat6 Q = Mat6::Zero();
        Q(kV, kV) = q_vv;
        Q(kOmega, kOmega) = q_ww;
        Q(kBiasOmega, kBiasOmega) = params.q_bias_omega;
        return Q;
    }

    bool is_symmetric_positive_definite(const Mat6& M, const double tol) noexcept
    {
        const double asymmetry = (M - M.transpose()).cwiseAbs().maxCoeff();
        if (asymmetry > tol)
        {
            return false;
        }
        const Eigen::LLT<Mat6> llt(M);
        return llt.info() == Eigen::Success;
    }

    Mat6 default_initial_covariance() noexcept
    {
        Mat6 Sigma0 = Mat6::Zero();
        Sigma0.diagonal() << 1e-6, 1e-6, 1e-6, 1e-2, 1e-2, 1e-2;
        return Sigma0;
    }

    EkfState6 make_initial_ekf_state() noexcept
    {
        return EkfState6{
            .mu = Vec6::Zero(),
            .Sigma = default_initial_covariance(),
        };
    }

    void Ekf::predict(const double dt)
    {
        assert(dt > 0.0 && "Ekf::predict requires a strictly positive dt");

        // 预测前的均值处求 Jacobian 与过程噪声(g 与 G 用同一个 integrator)
        const Mat6 G = process_jacobian_G(state_.mu, dt, integrator_);
        const Mat6 Q = process_noise_Q(state_.mu, noise_);

        // 均值预测
        state_.mu = process_model_g(state_.mu, dt, integrator_);
        state_.mu(kTheta) = wrap_angle(state_.mu(kTheta));

        // 协方差预测: Σ̄ = G·Σ·Gᵀ + Q。
        const Mat6 Sigma_pred = G * state_.Sigma * G.transpose() + Q;

        // 强制对称: 抵消 G·Σ·Gᵀ 在浮点下累积的微小非对称, 是廉价且标准的
        state_.Sigma = 0.5 * (Sigma_pred + Sigma_pred.transpose());

        // 运行期保险丝: debug 构建下每步校验 Σ 仍 symmetric PSD;
        assert(is_symmetric_positive_definite(state_.Sigma, 1e-9) &&
            "covariance lost symmetric-PSD property after predict");
    }

    double Ekf::update_encoder(const Eigen::Vector2d& z, const Eigen::Matrix2d& R_meas)
    {
        // 观测矩阵 H (2×6): 选出 (v, ω) 两行, bias 列为 0。显式构造完整 H
        // (而非手工切片)让下面的 Joseph form 与教科书逐字对应、可读性最佳; 6D 下
        // 这点乘法开销可忽略。
        Eigen::Matrix<double, 2, kStateDim> H = Eigen::Matrix<double, 2, kStateDim>::Zero();
        H(0, kV) = 1.0;
        H(1, kOmega) = 1.0;

        // innovation y = z − H·μ̄。
        const Eigen::Vector2d y = z - H * state_.mu;

        // innovation covariance S = H·Σ̄·Hᵀ + R (2×2)。
        const Eigen::Matrix2d S = H * state_.Sigma * H.transpose() + R_meas;
        const Eigen::Matrix2d S_inv = S.inverse(); // S 是 2×2, 直接求逆数值稳妥。

        // NIS = yᵀ·S⁻¹·y(consistency 诊断, 在更新均值前用预测期量算)。
        const double nis = y.dot(S_inv * y);

        // Kalman gain K = Σ̄·Hᵀ·S⁻¹ (6×2)。
        const Eigen::Matrix<double, kStateDim, 2> K =
            state_.Sigma * H.transpose() * S_inv;

        // 均值更新。
        state_.mu += K * y;
        state_.mu(kTheta) = wrap_angle(state_.mu(kTheta));

        // 协方差更新 —— Joseph form: Σ = (I−K·H)·Σ̄·(I−K·H)ᵀ + K·R·Kᵀ。
        // 两个 M·X·Mᵀ 形式各自对称 PSD, 结构上保证输出对称半正定, 即便 K 含
        // 浮点误差。优于简化式 (I−K·H)·Σ̄(后者会累积非对称、可能丢正定性)。
        const Mat6 I = Mat6::Identity();
        const Mat6 IKH = I - K * H;
        const Mat6 Sigma_new =
            IKH * state_.Sigma * IKH.transpose() + K * R_meas * K.transpose();

        // 再对称化以清除最后的浮点不对称(廉价数值卫生)。
        state_.Sigma = 0.5 * (Sigma_new + Sigma_new.transpose());

        assert(is_symmetric_positive_definite(state_.Sigma, 1e-9) &&
            "covariance lost symmetric-PSD property after encoder update");

        return nis;
    }

    double Ekf::update_imu(const double z, const double R_imu)
    {
        assert(R_imu > 0.0 && "Ekf::update_imu requires strictly positive R_imu "
            "(use the encoder quantization-floor trick if a zero-floor R is needed)");

        if (noise_.q_bias_omega <= 0.0)
        {
            // q_bias_omega == 0 是兼容模式: 不在线估计 gyro bias, IMU 退化为
            // 原始观测 z = omega + noise。这样默认构造的 EKF 不会让一个
            // 未启用的 bias 状态吸收 gyro 白噪声。
            const double y = z - state_.mu(kOmega);
            const double S = state_.Sigma(kOmega, kOmega) + R_imu;
            const double nis = y * y / S; // NIS = y²/S(1-DOF)。
            const Vec6 K = state_.Sigma.col(kOmega) / S;

            state_.mu += K * y;
            state_.mu(kTheta) = wrap_angle(state_.mu(kTheta));

            Mat6 IKH = Mat6::Identity();
            IKH.col(kOmega) -= K;
            const Mat6 Sigma_new =
                IKH * state_.Sigma * IKH.transpose() + (K * K.transpose()) * R_imu;
            state_.Sigma = 0.5 * (Sigma_new + Sigma_new.transpose());

            assert(is_symmetric_positive_definite(state_.Sigma, 1e-9) &&
                "covariance lost symmetric-PSD property after IMU update");
            return nis;
        }

        // 1D 观测的 Kalman update — H_imu = [0,0,0,0,1,1], 即 Hᵀ = e_ω + e_b。
        // 把矩阵运算展开成"ω 列 + bias 列"的形式, 避免引入 Eigen 的 1×N 矩阵,
        // 同时让 bias 通过 H 的 bias 列参与修正(这是 PR4 相对 PR3 的唯一数学变化)。
        // Σ̄·Hᵀ = Σ̄.col(ω) + Σ̄.col(b)  (6-vector)。
        const Vec6 SHt = state_.Sigma.col(kOmega) + state_.Sigma.col(kBiasOmega);

        // ẑ = H·μ̄ = μ̄(ω) + μ̄(b)。
        const double z_hat = state_.mu(kOmega) + state_.mu(kBiasOmega);

        // innovation。
        const double y = z - z_hat;

        // S = H·(Σ̄·Hᵀ) + R = SHt(ω) + SHt(b) + R_imu (标量 innovation 方差)。
        const double S = SHt(kOmega) + SHt(kBiasOmega) + R_imu;

        // NIS = y²/S(1-DOF), 在更新均值前用预测期量算。
        const double nis = y * y / S;

        // K = (Σ̄·Hᵀ) / S (6-vector)。
        const Vec6 K = SHt / S;

        // 均值更新, 随后单独 wrap θ。
        state_.mu += K * y;
        state_.mu(kTheta) = wrap_angle(state_.mu(kTheta));

        // 协方差 Joseph form: Σ = (I − K·H)·Σ̄·(I − K·H)ᵀ + K·R·Kᵀ。
        // K·H 是 6×6, 只在 ω、bias 两列非零(各等于 K)。故 (I − K·H) 在这两列
        // 各减去 K, 其余列不变。
        Mat6 IKH = Mat6::Identity();
        IKH.col(kOmega) -= K;
        IKH.col(kBiasOmega) -= K;

        const Mat6 Sigma_new =
            IKH * state_.Sigma * IKH.transpose() + (K * K.transpose()) * R_imu;

        state_.Sigma = 0.5 * (Sigma_new + Sigma_new.transpose());

        assert(is_symmetric_positive_definite(state_.Sigma, 1e-9) &&
            "covariance lost symmetric-PSD property after IMU update");

        return nis;
    }
}
