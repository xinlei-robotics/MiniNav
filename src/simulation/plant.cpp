module mininav.simulation.plant;

import mininav.core.types;
import mininav.core.kinematics;
import mininav.core.random;
import mininav.core.robot_description;
import mininav.sensors.actuator_model;
import mininav.sensors.wheel_encoder;
import mininav.sensors.imu_model;
import mininav.simulation.noise_presets;

namespace mininav::simulation
{
    Plant::Plant(const RobotDescription& robot, const NoisePreset& noise, const RngFactory& rng,
                 const Pose2D& initial_pose)
        : actuator_{
              ActuatorNoiseParams{
                  .alpha1 = noise.alpha1, .alpha2 = noise.alpha2,
                  .alpha3 = noise.alpha3, .alpha4 = noise.alpha4,
              },
              rng.make_engine("actuator")
          },
          encoder_{
              WheelEncoderParams{
                  .wheel_radius = robot.wheel_radius,
                  .wheel_base = robot.wheel_base,
                  .ticks_per_rev = robot.ticks_per_rev,
                  .slip_sigma = noise.slip_sigma,
              },
              rng.make_engine("encoder_slip_left"),
              rng.make_engine("encoder_slip_right")
          },
          imu_{
              ImuParams{
                  .sigma_omega = noise.sigma_imu,
                  .bias_omega_init = noise.imu_bias_init,
                  .bias_random_walk = noise.imu_bias_rw,
              },
              rng.make_engine("imu_gyro_noise"),
              rng.make_engine("imu_gyro_bias")
          },
          truth_{initial_pose}
    {
    }

    SensorReadings Plant::step(const Twist2D& cmd, const double dt)
    {
        // 顺序即 RNG 消耗顺序:actuator → encoder → imu。测量取自本步的真实速度,
        // 真值在测量之后才推进。
        const Twist2D true_velocity = actuator_.apply(cmd);
        const EncoderTicks dticks = encoder_.measure(true_velocity, dt);
        const double imu_omega = imu_.measure(true_velocity.w());
        truth_ = differential_drive_step(truth_, true_velocity, dt);
        return SensorReadings{
            .true_velocity = true_velocity,
            .dticks = dticks,
            .imu_omega = imu_omega,
        };
    }
}
