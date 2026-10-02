import mininav.localization.ekf_pipeline;
import mininav.localization.ekf;
import mininav.localization.ekf_state;
import mininav.localization.encoder_observation;
import mininav.core.types;

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>

namespace {

using mininav::EncoderTicks;
using mininav::Pose2D;
using mininav::ekf::Ekf;
using mininav::ekf::EkfNis;
using mininav::ekf::EkfPipeline;
using mininav::ekf::EkfPipelineConfig;

constexpr double kDt = 0.01;

// default 档位的参数;r_scale ≠ 1,确保缩放也按原顺序作用在 R 上。
[[nodiscard]] EkfPipelineConfig test_config() {
  EkfPipelineConfig cfg{};
  cfg.encoder = {.sigma_slip = 0.02,
                 .distance_per_tick = 2.0 * std::numbers::pi * 0.032 / 1024.0,
                 .wheel_base = 0.150};
  cfg.sigma_imu = 0.005;
  cfg.r_scale = 1.7;
  cfg.process = {.alpha1 = 0.05, .alpha2 = 0.02, .alpha3 = 0.02, .alpha4 = 0.05,
                 .q_bias_omega = 1e-8};
  cfg.integrator = mininav::ekf::Integrator::Rk4;
  return cfg;
}

// 确定性的合成读数:左右轮 tick 缓慢变化(含转弯),陀螺带 0.02 rad/s 偏置。
[[nodiscard]] EncoderTicks ticks_at(const std::size_t k) {
  const auto base = static_cast<std::int64_t>(15 + (k / 40) % 5);
  const auto turn = static_cast<std::int64_t>((k / 100) % 3) - 1;
  return EncoderTicks{.left = base - turn, .right = base + turn};
}

[[nodiscard]] double gyro_at(const std::size_t k) {
  return 0.3 * std::sin(0.01 * static_cast<double>(k)) + 0.02;
}

} // namespace

// EkfPipeline 只是把 V2 app 里的编排搬进 localization:与手写的
// predict → decode → R_enc·r_scale → update_encoder → σ²·r_scale → update_imu
// 必须逐位相同(EKF 模式 golden 不变的前提)。
TEST(EkfPipeline, MatchesManualUpdateSequenceBitForBit) {
  const EkfPipelineConfig cfg = test_config();
  EkfPipeline pipeline{cfg, Pose2D{0.0, 0.0, 0.0}};
  Ekf manual{mininav::ekf::make_initial_ekf_state(), cfg.process, cfg.integrator};

  using mininav::ekf::kOmega;
  using mininav::ekf::kV;
  for (std::size_t k = 0; k < 500; ++k) {
    const EncoderTicks dticks = ticks_at(k);
    const double omega = gyro_at(k);

    const EkfNis nis = pipeline.step(dticks, omega, kDt);

    manual.predict(kDt);
    const Eigen::Vector2d z = mininav::decode_encoder(dticks, cfg.encoder, kDt);
    const Eigen::Matrix2d R =
        mininav::encoder_noise_covariance(manual.mu()(kV), manual.mu()(kOmega), cfg.encoder, kDt)
        * cfg.r_scale;
    const double nis_enc = manual.update_encoder(z, R);
    const double nis_imu = manual.update_imu(omega, cfg.sigma_imu * cfg.sigma_imu * cfg.r_scale);

    ASSERT_EQ(nis.encoder, nis_enc) << "step " << k;
    ASSERT_EQ(nis.imu, nis_imu) << "step " << k;
    for (int i = 0; i < mininav::ekf::kStateDim; ++i) {
      ASSERT_EQ(pipeline.filter().mu()(i), manual.mu()(i)) << "step " << k << " mu " << i;
      for (int j = 0; j < mininav::ekf::kStateDim; ++j) {
        ASSERT_EQ(pipeline.filter().Sigma()(i, j), manual.Sigma()(i, j))
            << "step " << k << " Sigma " << i << ',' << j;
      }
    }
  }
}

TEST(EkfPipeline, InitialPoseIsInjectedWithDefaultCovariance) {
  const EkfPipeline pipeline{test_config(), Pose2D{1.5, -0.5, 0.75}};
  EXPECT_EQ(pipeline.pose().x(), 1.5);
  EXPECT_EQ(pipeline.pose().y(), -0.5);
  EXPECT_EQ(pipeline.pose().yaw(), 0.75);
  EXPECT_EQ(pipeline.velocity().v(), 0.0);
  EXPECT_EQ(pipeline.velocity().w(), 0.0);
  EXPECT_EQ(pipeline.filter().mu()(mininav::ekf::kBiasOmega), 0.0);
  EXPECT_TRUE(pipeline.filter().Sigma().isApprox(mininav::ekf::default_initial_covariance()));
}

TEST(EkfPipeline, PoseAndVelocityReadTheFilterMean) {
  EkfPipeline pipeline{test_config(), Pose2D{}};
  for (std::size_t k = 0; k < 200; ++k) {
    (void)pipeline.step(ticks_at(k), gyro_at(k), kDt);
  }
  using namespace mininav::ekf;
  const Vec6& mu = pipeline.filter().mu();
  EXPECT_EQ(pipeline.pose().x(), mu(kPx));
  EXPECT_EQ(pipeline.pose().y(), mu(kPy));
  EXPECT_EQ(pipeline.pose().yaw(), mu(kTheta));
  EXPECT_EQ(pipeline.velocity().v(), mu(kV));
  EXPECT_EQ(pipeline.velocity().w(), mu(kOmega));
  EXPECT_GT(pipeline.velocity().v(), 0.0);  // 合成读数一直在前进
}

TEST(EkfPipeline, ForwardsIntegratorChoice) {
  EkfPipelineConfig cfg = test_config();
  cfg.integrator = mininav::ekf::Integrator::Euler;
  const EkfPipeline pipeline{cfg, Pose2D{}};
  EXPECT_EQ(pipeline.filter().integrator(), mininav::ekf::Integrator::Euler);
}
