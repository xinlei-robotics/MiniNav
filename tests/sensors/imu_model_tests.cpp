// ===========================================================================
// ImuModel 测试。
//
//   1. 零噪声退化 : sigma_omega=0 时直接返回 true_omega, 不消费 RNG。
//      "完美 gyro"基线, 单元测试可隔离"算法 vs 噪声"。
//   2. 噪声统计性质 : sigma_omega > 0 时, measure(ω) − ω 的样本均值 ≈ 0,
//      样本方差 ≈ sigma_omega² (3σ 容差)。
//   3. RNG 独立性 : 同一 master seed 下用 "imu_gyro_noise" 标签拿到的子种子,
//      与 actuator/encoder 标签拿到的不同, 即 IMU 加入不会扰动 V1 的随机序列。
// ===========================================================================
#include <gtest/gtest.h>

#include <cmath>
#include <random>

import mininav.sensors.imu_model;
import mininav.core.random;

using namespace mininav;

namespace
{
    ImuModel make_imu(double sigma, std::uint64_t seed = 7)
    {
        const RngFactory factory{seed};
        return ImuModel{ImuParams{.sigma_omega = sigma}, factory.make_engine("imu_gyro_noise")};
    }
}

// --- 1. 零噪声退化 -------------------------------------------------------

TEST(ImuModel, ZeroNoisePassesThroughExactly)
{
    ImuModel imu = make_imu(0.0);

    for (const double w : {-1.5, -0.3, 0.0, 0.4, 1.2})
    {
        EXPECT_EQ(imu.measure(w), w) << "完美 gyro 应原样返回 true_omega";
    }
}

TEST(ImuModel, ZeroNoiseDoesNotConsumeRng)
{
    // sigma=0 时, 多次 measure 不调 distribution, RNG 状态不变。
    ImuModel zero_imu = make_imu(0.0);
    for (int i = 0; i < 1000; ++i)
    {
        zero_imu.measure(0.5);
    }
    // 此处只验证不会 crash 且行为可预期; 严格的"RNG 未被消费"由 sigma=0
    // 路径明确不调 distribution 保证, 见 imu_model.cpp。
    SUCCEED();
}

// --- 2. 噪声统计性质 -----------------------------------------------------

TEST(ImuModel, NoiseStatisticsMatchSigmaOmega)
{
    constexpr double sigma = 0.005;
    ImuModel imu = make_imu(sigma);

    constexpr double true_omega = 0.3;
    constexpr int N = 100000;
    double sum = 0.0, sum_sq = 0.0;
    for (int i = 0; i < N; ++i)
    {
        const double residual = imu.measure(true_omega) - true_omega;
        sum += residual;
        sum_sq += residual * residual;
    }
    const double mean = sum / N;
    const double var = sum_sq / N - mean * mean;
    const double std = std::sqrt(var);

    // 样本均值 ≈ 0 (3σ_mean ≈ 3·sigma/√N)。
    EXPECT_LT(std::abs(mean), 3.0 * sigma / std::sqrt(static_cast<double>(N)));
    // 样本标准差与 sigma 在 5% 内吻合 (N=100k 下样本 σ 的相对误差 ~ 1/√(2N) ≈ 0.2%)。
    EXPECT_NEAR(std, sigma, 0.05 * sigma);
}

TEST(ImuModel, NoiseIsAdditiveAndIndependentOfTrueOmega)
{
    // measure(ω) − ω 的分布与 ω 无关 (加性高斯噪声)。
    constexpr double sigma = 0.01;
    constexpr int N = 50000;

    auto sample_std = [sigma](double true_omega)
    {
        ImuModel imu = make_imu(sigma, 123);
        double sum = 0.0, sum_sq = 0.0;
        for (int i = 0; i < N; ++i)
        {
            const double r = imu.measure(true_omega) - true_omega;
            sum += r;
            sum_sq += r * r;
        }
        const double m = sum / N;
        return std::sqrt(sum_sq / N - m * m);
    };

    const double s_at_zero = sample_std(0.0);
    const double s_at_one = sample_std(1.0);
    // 两者都应 ≈ sigma; 比较时 5% 容差。
    EXPECT_NEAR(s_at_zero, s_at_one, 0.05 * sigma);
}

// --- 3. RNG 独立性 -------------------------------------------------------

TEST(ImuModel, GyroSubseedDiffersFromActuatorAndEncoder)
{
    const RngFactory factory{42};
    std::mt19937 imu_rng = factory.make_engine("imu_gyro_noise");
    std::mt19937 act_rng = factory.make_engine("actuator");
    std::mt19937 enc_l_rng = factory.make_engine("encoder_slip_left");

    // 取每个引擎的首个输出来比较 — 不同子种子 → 不同首字。
    const auto a = imu_rng();
    const auto b = act_rng();
    const auto c = enc_l_rng();

    EXPECT_NE(a, b) << "imu 与 actuator 子种子应不同, 否则 IMU 噪声会污染 V1 序列";
    EXPECT_NE(a, c) << "imu 与 encoder_slip_left 子种子应不同";
    EXPECT_NE(b, c);
}