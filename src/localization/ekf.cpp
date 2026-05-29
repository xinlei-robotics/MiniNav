module;

#include <Eigen/Cholesky>
#include <Eigen/Core>

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
}