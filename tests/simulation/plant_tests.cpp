import mininav.simulation.plant;
import mininav.simulation.noise_presets;
import mininav.core.types;
import mininav.core.kinematics;
import mininav.core.random;
import mininav.core.robot_description;
import mininav.sensors.actuator_model;
import mininav.sensors.wheel_encoder;
import mininav.sensors.imu_model;

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace {

using mininav::EncoderTicks;
using mininav::Pose2D;
using mininav::RngFactory;
using mininav::RobotDescription;
using mininav::Twist2D;
using mininav::simulation::NoisePreset;
using mininav::simulation::Plant;
using mininav::simulation::SensorReadings;

constexpr double kDt = 0.01;

[[nodiscard]] RobotDescription test_robot() {
  RobotDescription r{};
  r.wheel_radius = 0.032;
  r.wheel_base = 0.150;
  r.ticks_per_rev = 1024;
  r.footprint = {.length = 0.22, .width = 0.16};
  r.limits = {.max_linear_vel = 0.5, .max_angular_vel = 2.0};
  r.actuator_time_constant = 0.1;
  return r;
}

constexpr NoisePreset kNoiseFree{
    .name = "noise-free",
    .alpha1 = 0.0, .alpha2 = 0.0, .alpha3 = 0.0, .alpha4 = 0.0,
    .slip_sigma = 0.0,
    .sigma_imu = 0.0,
    .imu_bias_init = 0.0,
    .imu_bias_rw = 0.0,
    .q_bias_omega = 0.0,
};

// 变化的指令序列:直行、转弯、原地转、静止(静止时执行器不采样)。
[[nodiscard]] Twist2D command_at(const std::size_t k) {
  switch ((k / 50) % 4) {
    case 0: return Twist2D{0.3, 0.0};
    case 1: return Twist2D{0.25, 0.8};
    case 2: return Twist2D{0.0, -1.0};
    default: return Twist2D{0.0, 0.0};
  }
}

void expect_same_ticks(const EncoderTicks& a, const EncoderTicks& b) {
  EXPECT_EQ(a.left, b.left);
  EXPECT_EQ(a.right, b.right);
}

} // namespace

// Plant 必须与"直接按 V2 的方式组合三个传感器模型"逐位相同:相同的 RNG 标签、
// 相同的消耗顺序、测量先于真值推进。这条测试把 EKF 模式 golden 的前提锁在单测层。
TEST(Plant, MatchesDirectSensorCompositionBitForBit) {
  const RobotDescription robot = test_robot();
  const NoisePreset& noise = mininav::simulation::kPresetDefault;
  const RngFactory rng{42};

  Plant plant{robot, noise, rng, Pose2D{0.0, 0.0, 0.0}};

  mininav::ActuatorModel actuator{
      mininav::ActuatorNoiseParams{.alpha1 = noise.alpha1, .alpha2 = noise.alpha2,
                                   .alpha3 = noise.alpha3, .alpha4 = noise.alpha4},
      rng.make_engine("actuator")};
  mininav::WheelEncoderModel encoder{
      mininav::WheelEncoderParams{.wheel_radius = robot.wheel_radius, .wheel_base = robot.wheel_base,
                                  .ticks_per_rev = robot.ticks_per_rev, .slip_sigma = noise.slip_sigma},
      rng.make_engine("encoder_slip_left"), rng.make_engine("encoder_slip_right")};
  mininav::ImuModel imu{
      mininav::ImuParams{.sigma_omega = noise.sigma_imu, .bias_omega_init = noise.imu_bias_init,
                         .bias_random_walk = noise.imu_bias_rw},
      rng.make_engine("imu_gyro_noise"), rng.make_engine("imu_gyro_bias")};
  Pose2D truth{0.0, 0.0, 0.0};

  for (std::size_t k = 0; k < 400; ++k) {
    const Twist2D cmd = command_at(k);
    const SensorReadings r = plant.step(cmd, kDt);

    const Twist2D v = actuator.apply(cmd);
    const EncoderTicks ticks = encoder.measure(v, kDt);
    const double omega = imu.measure(v.w());
    truth = mininav::differential_drive_step(truth, v, kDt);

    ASSERT_EQ(r.true_velocity.v(), v.v()) << "step " << k;
    ASSERT_EQ(r.true_velocity.w(), v.w()) << "step " << k;
    expect_same_ticks(r.dticks, ticks);
    ASSERT_EQ(r.imu_omega, omega) << "step " << k;
    ASSERT_EQ(plant.truth().x(), truth.x()) << "step " << k;
    ASSERT_EQ(plant.truth().y(), truth.y()) << "step " << k;
    ASSERT_EQ(plant.truth().yaw(), truth.yaw()) << "step " << k;
  }
  EXPECT_EQ(plant.true_bias_omega(), noise.imu_bias_init);
}

// 无噪声时 Plant 退化为纯运动学:真实速度 = 指令,陀螺 = ω,真值 = 逐步积分。
TEST(Plant, NoiseFreePlantFollowsKinematicsExactly) {
  Plant plant{test_robot(), kNoiseFree, RngFactory{1}, Pose2D{1.0, -2.0, 0.5}};
  Pose2D expected{1.0, -2.0, 0.5};
  for (std::size_t k = 0; k < 200; ++k) {
    const Twist2D cmd = command_at(k);
    const SensorReadings r = plant.step(cmd, kDt);
    expected = mininav::differential_drive_step(expected, cmd, kDt);
    EXPECT_EQ(r.true_velocity.v(), cmd.v());
    EXPECT_EQ(r.true_velocity.w(), cmd.w());
    EXPECT_EQ(r.imu_omega, cmd.w());
  }
  EXPECT_EQ(plant.truth().x(), expected.x());
  EXPECT_EQ(plant.truth().y(), expected.y());
  EXPECT_EQ(plant.truth().yaw(), expected.yaw());
}

// 编码器读数与走过的弧长一致:直行 1 s @ 0.3 m/s,两轮各走 0.3 m。
TEST(Plant, NoiseFreeEncoderCountsMatchDistance) {
  const RobotDescription robot = test_robot();
  Plant plant{robot, kNoiseFree, RngFactory{1}, Pose2D{}};
  std::int64_t left = 0;
  std::int64_t right = 0;
  for (std::size_t k = 0; k < 100; ++k) {
    const SensorReadings r = plant.step(Twist2D{0.3, 0.0}, kDt);
    left += r.dticks.left;
    right += r.dticks.right;
  }
  const double expected_ticks = 0.3 / robot.distance_per_tick();
  EXPECT_NEAR(static_cast<double>(left), expected_ticks, 1.0);
  EXPECT_NEAR(static_cast<double>(right), expected_ticks, 1.0);
}

TEST(Plant, TruthStartsAtInitialPose) {
  const Plant plant{test_robot(), mininav::simulation::kPresetDefault, RngFactory{7},
                    Pose2D{0.5, 0.25, -1.0}};
  EXPECT_EQ(plant.truth().x(), 0.5);
  EXPECT_EQ(plant.truth().y(), 0.25);
  EXPECT_EQ(plant.truth().yaw(), -1.0);
}

TEST(Plant, SameSeedReproducesAndDifferentSeedDiffers) {
  const RobotDescription robot = test_robot();
  const NoisePreset& noise = mininav::simulation::kPresetHighNoise;
  Plant a{robot, noise, RngFactory{3}, Pose2D{}};
  Plant b{robot, noise, RngFactory{3}, Pose2D{}};
  Plant c{robot, noise, RngFactory{4}, Pose2D{}};
  for (std::size_t k = 0; k < 100; ++k) {
    const Twist2D cmd{0.3, 0.2};
    (void)a.step(cmd, kDt);
    (void)b.step(cmd, kDt);
    (void)c.step(cmd, kDt);
  }
  EXPECT_EQ(a.truth().x(), b.truth().x());
  EXPECT_EQ(a.truth().y(), b.truth().y());
  EXPECT_NE(a.truth().x(), c.truth().x());
}

// ---------------------------------------------------------------------------
// 噪声档位表
// ---------------------------------------------------------------------------

TEST(NoisePreset, LookupByName) {
  using mininav::simulation::noise_preset;
  EXPECT_EQ(noise_preset("low-noise").slip_sigma, 0.005);
  EXPECT_EQ(noise_preset("default").slip_sigma, 0.02);
  EXPECT_EQ(noise_preset("high-noise").slip_sigma, 0.05);
  EXPECT_THROW((void)noise_preset("medium"), std::invalid_argument);
}

// 三档噪声单调递增:low ≤ default ≤ high(逐项)。
TEST(NoisePreset, LevelsAreOrdered) {
  using mininav::simulation::kNoisePresets;
  for (std::size_t i = 1; i < kNoisePresets.size(); ++i) {
    const NoisePreset& lo = kNoisePresets[i - 1];
    const NoisePreset& hi = kNoisePresets[i];
    EXPECT_LE(lo.alpha1, hi.alpha1);
    EXPECT_LE(lo.alpha4, hi.alpha4);
    EXPECT_LE(lo.slip_sigma, hi.slip_sigma);
    EXPECT_LE(lo.sigma_imu, hi.sigma_imu);
  }
}
