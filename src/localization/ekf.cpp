module;

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>

#include <cassert>
#include <cmath>

module mininav.localization.ekf;

import mininav.localization.ekf_state;
import mininav.core.math;

namespace mininav
{
    Vec5 process_model_g(const Vec5& mu, const double dt) noexcept
    {
        const double px = mu(kPx);
        const double py = mu(kPy);
        const double th = mu(kTheta);
        const double v = mu(kV);
        const double w = mu(kOmega);

        Vec5 out;
        out(kPx) = px + v * std::cos(th) * dt;
        out(kPy) = py + v * std::sin(th) * dt;
        out(kTheta) = th + w * dt;
        out(kV) = v;
        out(kOmega) = w;
        return out;
    }

    Mat5 process_jacobian_G(const Vec5& mu, const double dt) noexcept
    {
        const double th = mu(kTheta);
        const double v = mu(kV);

        Mat5 G = Mat5::Identity();
        G(kPx, kTheta) = -v * std::sin(th) * dt;
        G(kPx, kV) = std::cos(th) * dt;
        G(kPy, kTheta) = v * std::cos(th) * dt;
        G(kPy, kV) = std::sin(th) * dt;
        G(kTheta, kOmega) = dt;
        return G;
    }

    Mat5 numeric_process_jacobian(const Vec5& mu, const double dt, const double eps) noexcept
    {
        Mat5 G_num = Mat5::Zero();
        for (int j = 0; j < kStateDim; ++j)
        {
            Vec5 plus = mu;
            Vec5 minus = mu;
            plus(j) += eps;
            minus(j) -= eps;
            const Vec5 col = (process_model_g(plus, dt) - process_model_g(minus, dt)) / (2.0 * eps);
            G_num.col(j) = col;
        }
        return G_num;
    }

    Mat5 process_noise_Q(const Vec5& mu, const ProcessNoiseParams& params) noexcept
    {
        const double v = mu(kV);
        const double w = mu(kOmega);

        const double q_vv = params.alpha1 * v * v + params.alpha2 * w * w;
        const double q_ww = params.alpha3 * v * v + params.alpha4 * w * w;

        Mat5 Q = Mat5::Zero();
        Q(kV, kV) = q_vv;
        Q(kOmega, kOmega) = q_ww;
        return Q;
    }

    bool is_symmetric_positive_definite(const Mat5& M, const double tol) noexcept
    {
        const double asymmetry = (M - M.transpose()).cwiseAbs().maxCoeff();
        if (asymmetry > tol)
        {
            return false;
        }
        const Eigen::LLT<Mat5> llt(M);
        return llt.info() == Eigen::Success;
    }

    Mat5 default_initial_covariance() noexcept
    {
        Mat5 Sigma0 = Mat5::Zero();
        Sigma0.diagonal() << 1e-6, 1e-6, 1e-6, 1e-2, 1e-2;
        return Sigma0;
    }

    EkfState5 make_initial_ekf_state() noexcept
    {
        return EkfState5{
            .mu = Vec5::Zero(),
            .Sigma = default_initial_covariance(),
        };
    }

    void Ekf::predict(const double dt)
    {
        assert(dt > 0.0 && "Ekf::predict requires a strictly positive dt");

        // 预测前的均值处求 Jacobian 与过程噪声
        const Mat5 G = process_jacobian_G(state_.mu, dt);
        const Mat5 Q = process_noise_Q(state_.mu, noise_);

        // 均值预测
        state_.mu = process_model_g(state_.mu, dt);
        state_.mu(kTheta) = wrap_angle(state_.mu(kTheta));

        // 协方差预测: Σ̄ = G·Σ·Gᵀ + Q。
        const Mat5 Sigma_pred = G * state_.Sigma * G.transpose() + Q;

        // 强制对称: 抵消 G·Σ·Gᵀ 在浮点下累积的微小非对称, 是廉价且标准的
        state_.Sigma = 0.5 * (Sigma_pred + Sigma_pred.transpose());

        // 运行期保险丝: debug 构建下每步校验 Σ 仍 symmetric PSD;
        assert(is_symmetric_positive_definite(state_.Sigma, 1e-9) &&
            "covariance lost symmetric-PSD property after predict");
    }

    void Ekf::update_encoder(const Eigen::Vector2d& z, const Eigen::Matrix2d& R_meas)
    {
        // 观测矩阵 H (2×5): 选出 (v, ω) 两行。显式构造完整 H(而非手工切片)
        // 让下面的 Joseph form 与教科书逐字对应、可读性最佳; 5D 下这点乘法
        // 开销可忽略。
        Eigen::Matrix<double, 2, kStateDim> H = Eigen::Matrix<double, 2, kStateDim>::Zero();
        H(0, kV) = 1.0;
        H(1, kOmega) = 1.0;

        // innovation y = z − H·μ̄。
        const Eigen::Vector2d y = z - H * state_.mu;

        // innovation covariance S = H·Σ̄·Hᵀ + R (2×2)。
        const Eigen::Matrix2d S = H * state_.Sigma * H.transpose() + R_meas;

        // Kalman gain K = Σ̄·Hᵀ·S⁻¹ (5×2)。S 是 2×2, 直接求逆数值稳妥。
        const Eigen::Matrix<double, kStateDim, 2> K =
            state_.Sigma * H.transpose() * S.inverse();

        // 均值更新。
        state_.mu += K * y;
        state_.mu(kTheta) = wrap_angle(state_.mu(kTheta));

        // 协方差更新 —— Joseph form: Σ = (I−K·H)·Σ̄·(I−K·H)ᵀ + K·R·Kᵀ。
        // 两个 M·X·Mᵀ 形式各自对称 PSD, 结构上保证输出对称半正定, 即便 K 含
        // 浮点误差。优于简化式 (I−K·H)·Σ̄(后者会累积非对称、可能丢正定性)。
        const Mat5 I = Mat5::Identity();
        const Mat5 IKH = I - K * H;
        const Mat5 Sigma_new =
            IKH * state_.Sigma * IKH.transpose() + K * R_meas * K.transpose();

        // 再对称化以清除最后的浮点不对称(廉价数值卫生)。
        state_.Sigma = 0.5 * (Sigma_new + Sigma_new.transpose());

        assert(is_symmetric_positive_definite(state_.Sigma, 1e-9) &&
            "covariance lost symmetric-PSD property after encoder update");
    }

    void Ekf::update_imu(const double z, const double R_imu)
    {
        assert(R_imu > 0.0 && "Ekf::update_imu requires strictly positive R_imu "
            "(use the encoder quantization-floor trick if a zero-floor R is needed)");

        // 1D 观测的 Kalman update — 把 5D 的 H = [0,0,0,0,1] 展开成标量算式,
        // 避免引入 Eigen 的 1×1 矩阵 (Matrix<double,1,1>) 与 Vector<double,1>。
        //
        // 数学等价于把 update_encoder 限制到第二行: H·μ̄ = μ̄(ω),
        // H·Σ̄·Hᵀ = Σ̄(ω,ω), Σ̄·Hᵀ = Σ̄ 的第 kOmega 列。

        // y = z − μ̄(ω)。
        const double y = z - state_.mu(kOmega);

        // S = Σ̄(ω,ω) + R_imu (标量 innovation 方差)。
        const double S = state_.Sigma(kOmega, kOmega) + R_imu;

        // K = Σ̄·Hᵀ / S = Σ̄ 的 ω 列 / S (5-vector)。
        const Vec5 K = state_.Sigma.col(kOmega) / S;

        // 均值更新, 随后单独 wrap θ。
        state_.mu += K * y;
        state_.mu(kTheta) = wrap_angle(state_.mu(kTheta));

        // 协方差 Joseph form: Σ = (I − K·H)·Σ̄·(I − K·H)ᵀ + K·R·Kᵀ。
        // H 只在 kOmega 列为 1, 故 K·H 只影响 Σ 的 kOmega 列: (I − K·H)
        // 的 kOmega 列变成 e_ω − K, 其余列不变。下面直接构造 (I − K·H)
        // 矩阵以与 update_encoder 同形地走 Joseph 流程。
        Mat5 IKH = Mat5::Identity();
        IKH.col(kOmega) -= K;

        const Mat5 Sigma_new =
            IKH * state_.Sigma * IKH.transpose() + (K * K.transpose()) * R_imu;

        state_.Sigma = 0.5 * (Sigma_new + Sigma_new.transpose());

        assert(is_symmetric_positive_definite(state_.Sigma, 1e-9) &&
            "covariance lost symmetric-PSD property after IMU update");
    }
}