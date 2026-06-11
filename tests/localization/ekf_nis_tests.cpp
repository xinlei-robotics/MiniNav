// ===========================================================================
// NIS (Normalized Innovation Squared) 诊断的数值正确性。
//
// update_encoder / update_imu 返回 NIS = yᵀ·S⁻¹·y(分别 2-DOF / 1-DOF)。
// 这里用预测期(predict 后、update 前)的 (μ̄, Σ̄) 独立地手算同一二次型,
// 与方法返回值逐位比对 —— 把"NIS 计算正确"钉成回归测试。consistency 分析
// (analyze_ekf.py 的 χ² 区间检验)完全依赖这个返回值的正确性。
// ===========================================================================
#include <gtest/gtest.h>

#include <Eigen/Dense>

import mininav.localization.ekf;
import mininav.localization.ekf_state;

using namespace mininav; using namespace mininav::ekf;

namespace
{
    ProcessNoiseParams noise_with_bias()
    {
        return ProcessNoiseParams{
            .alpha1 = 0.05, .alpha2 = 0.02, .alpha3 = 0.02, .alpha4 = 0.05,
            .q_bias_omega = 1e-8, // >0 → 启用 bias 估计路径(H_imu = [..,1,1])
        };
    }
}

TEST(EkfNis, EncoderNisMatchesQuadraticForm)
{
    Ekf ekf{make_initial_ekf_state(), noise_with_bias()};
    ekf.predict(0.01); // 让 Σ 离开初值, 给 NIS 一个非平凡的 S

    // 预测期量(update 会就地修改 μ/Σ, 故先取快照算期望)。
    const Vec6 mu = ekf.mu();
    const Mat6 Sigma = ekf.Sigma();

    const Eigen::Vector2d z{0.42, -0.17};
    Eigen::Matrix2d R;
    R << 4e-3, 0.0, 0.0, 9e-3;

    // 期望: H 选 (v, ω) 两行。
    const Eigen::Vector2d h{mu(kV), mu(kOmega)};
    const Eigen::Vector2d y = z - h;
    Eigen::Matrix2d S;
    S << Sigma(kV, kV), Sigma(kV, kOmega),
        Sigma(kOmega, kV), Sigma(kOmega, kOmega);
    S += R;
    const double expected = y.dot(S.inverse() * y);

    const double nis = ekf.update_encoder(z, R);

    EXPECT_NEAR(nis, expected, 1e-12);
    EXPECT_GT(nis, 0.0); // S 正定 ⇒ NIS ≥ 0(y≠0 时 >0)
}

TEST(EkfNis, ImuNisMatchesScalarFormWithBias)
{
    Ekf ekf{make_initial_ekf_state(), noise_with_bias()};
    ekf.predict(0.01);

    const Vec6 mu = ekf.mu();
    const Mat6 Sigma = ekf.Sigma();

    const double z = 0.3;
    const double R_imu = 2.5e-5;

    // 带 bias 的测量: ẑ = μ(ω)+μ(b), S = Σωω + 2Σωb + Σbb + R。
    const double z_hat = mu(kOmega) + mu(kBiasOmega);
    const double y = z - z_hat;
    const double S = Sigma(kOmega, kOmega) + 2.0 * Sigma(kOmega, kBiasOmega)
        + Sigma(kBiasOmega, kBiasOmega) + R_imu;
    const double expected = y * y / S;

    const double nis = ekf.update_imu(z, R_imu);

    EXPECT_NEAR(nis, expected, 1e-12);
    EXPECT_GT(nis, 0.0);
}

TEST(EkfNis, ImuNisMatchesScalarFormCompatMode)
{
    // q_bias_omega == 0 → 兼容模式: ẑ = μ(ω), S = Σωω + R(不含 bias 列)。
    Ekf ekf{
        make_initial_ekf_state(),
        ProcessNoiseParams{.alpha1 = 0.05, .alpha2 = 0.02, .alpha3 = 0.02, .alpha4 = 0.05}
    };
    ekf.predict(0.01);

    const Vec6 mu = ekf.mu();
    const Mat6 Sigma = ekf.Sigma();

    const double z = 0.25;
    const double R_imu = 2.5e-5;

    const double y = z - mu(kOmega);
    const double S = Sigma(kOmega, kOmega) + R_imu;
    const double expected = y * y / S;

    const double nis = ekf.update_imu(z, R_imu);

    EXPECT_NEAR(nis, expected, 1e-12);
}
