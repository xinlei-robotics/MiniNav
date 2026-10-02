export module mininav.simulation.plant;

import mininav.core.types;
import mininav.core.random;
import mininav.core.robot_description;
import mininav.sensors.actuator_model;
import mininav.sensors.wheel_encoder;
import mininav.sensors.imu_model;
import mininav.simulation.noise_presets;

export namespace mininav::simulation
{
    // ---------------------------------------------------------------------------
    // SensorReadings: 被控对象一步的产出。
    //   true_velocity  执行噪声之后的真实速度 —— 真值,只供记录与评估
    //   dticks / imu_omega  传感器读数 —— 估计器只能看到这两项
    // ---------------------------------------------------------------------------
    struct SensorReadings
    {
        Twist2D true_velocity;
        EncoderTicks dticks;
        double imu_omega{0.0};
    };

    // ---------------------------------------------------------------------------
    // Plant: 被仿真的机器人(真实世界的一侧)。
    //
    // 每步:指令 → 执行噪声(Velocity Motion Model)→ 编码器 + IMU 测量 → 真值积分。
    // 与估计器之间只流过 EncoderTicks 与陀螺标量(V1 的依赖倒置):V5 的仿真节点
    // 包装 Plant、EKF 节点包装 EkfPipeline,正好在这里一刀切开。
    //
    // 随机性:每个噪声源一个独立引擎,由 RngFactory 按标签派生("actuator"、
    // "encoder_slip_left/right"、"imu_gyro_noise/bias");每步消耗顺序固定为
    // actuator → encoder → imu。同 seed ⇒ 逐位相同的读数与真值。
    // ---------------------------------------------------------------------------
    class Plant
    {
    public:
        Plant(const RobotDescription& robot, const NoisePreset& noise, const RngFactory& rng,
              const Pose2D& initial_pose);

        Plant(const Plant&) = delete;
        Plant& operator=(const Plant&) = delete;
        Plant(Plant&&) noexcept = default;
        Plant& operator=(Plant&&) noexcept = default;
        ~Plant() = default;

        // 推进一步(dt 内指令保持不变),返回本步的传感器读数;truth() 随之更新。
        [[nodiscard]] SensorReadings step(const Twist2D& cmd, double dt);

        [[nodiscard]] const Pose2D& truth() const noexcept { return truth_; }

        // gyro bias 真值,供 bias 估计曲线对照。
        [[nodiscard]] double true_bias_omega() const noexcept { return imu_.bias_omega(); }

    private:
        ActuatorModel actuator_;
        WheelEncoderModel encoder_;
        ImuModel imu_;
        Pose2D truth_;
    };
}
