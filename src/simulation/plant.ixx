module;

#include <limits>

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
    // ActuatorDynamics: 指令到电机实际输出之间的确定性动力学(robot.yaml 的 limits 与
    // actuator_time_constant)。
    //
    //   1. 饱和:v、ω 各自钳到 ±max(差速底盘真实的限制在单轮转速上,这里按 robot.yaml
    //      给出的 (v, ω) 上限近似);
    //   2. 一阶滞后 τ·u̇ = sat(cmd) − u,按精确离散化推进一步:
    //        u ← u + (1 − e^(−dt/τ))·(sat(cmd) − u)
    //      前向欧拉的系数是 dt/τ,τ 接近 dt 时会过冲;精确系数恒在 (0, 1) 内。
    //
    // 默认值(上限 ∞、τ = 0)即"直通":不做任何浮点运算,EKF 模式的输出逐位不变。
    // ---------------------------------------------------------------------------
    struct ActuatorDynamics
    {
        double max_linear_vel{std::numeric_limits<double>::infinity()};  // m/s
        double max_angular_vel{std::numeric_limits<double>::infinity()}; // rad/s
        double time_constant{0.0};                                       // s,0 = 无滞后
    };

    // 由机器人描述得到完整的执行器动力学(闭环导航用)。
    [[nodiscard]] ActuatorDynamics actuator_dynamics_of(const RobotDescription& robot) noexcept;

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
    // 每步:指令 → 执行器动力学(饱和 + 一阶滞后,确定性)→ 执行噪声(Velocity Motion
    // Model)→ 编码器 + IMU 测量 → 真值积分。
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
        // dynamics 缺省为直通(EKF 模式);闭环导航传 actuator_dynamics_of(robot)。
        Plant(const RobotDescription& robot, const NoisePreset& noise, const RngFactory& rng,
              const Pose2D& initial_pose, const ActuatorDynamics& dynamics = {});

        Plant(const Plant&) = delete;
        Plant& operator=(const Plant&) = delete;
        Plant(Plant&&) noexcept = default;
        Plant& operator=(Plant&&) noexcept = default;
        ~Plant() = default;

        // 推进一步(dt 内指令保持不变),返回本步的传感器读数;truth() 随之更新。
        [[nodiscard]] SensorReadings step(const Twist2D& cmd, double dt);

        [[nodiscard]] const Pose2D& truth() const noexcept { return truth_; }

        // 最近一步经饱和与滞后之后、执行噪声之前的电机输出 u。
        [[nodiscard]] const Twist2D& actuator_output() const noexcept { return actuator_output_; }

        // gyro bias 真值,供 bias 估计曲线对照。
        [[nodiscard]] double true_bias_omega() const noexcept { return imu_.bias_omega(); }

    private:
        [[nodiscard]] Twist2D apply_dynamics(const Twist2D& cmd, double dt) noexcept;

        ActuatorDynamics dynamics_;
        bool passthrough_;
        Twist2D actuator_output_{};
        ActuatorModel actuator_;
        WheelEncoderModel encoder_;
        ImuModel imu_;
        Pose2D truth_;
    };
}
