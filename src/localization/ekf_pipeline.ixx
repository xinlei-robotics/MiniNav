export module mininav.localization.ekf_pipeline;

import mininav.core.types;
import mininav.localization.ekf;
import mininav.localization.encoder_observation;

export namespace mininav::ekf
{
    // ---------------------------------------------------------------------------
    // EkfPipelineConfig: 从原始读数到 EKF 更新所需的全部参数。
    //
    //   encoder     编码器观测模型:EncoderTicks 解码为 (v̂, ω̂),并由打滑 / 量化推导 R_enc
    //   sigma_imu   gyro 白噪声标准差 → R_imu = σ²
    //   r_scale     观测噪声 R(编码器 + IMU)的整体缩放;敏感性分析旋钮,1 = 物理值
    //   process     过程噪声 Q 的物理参数(Velocity Motion Model + bias 随机游走)
    //   integrator  过程模型 g 的积分器
    // ---------------------------------------------------------------------------
    struct EkfPipelineConfig
    {
        EncoderNoiseParams encoder{};
        double sigma_imu{0.0};
        double r_scale{1.0};
        ProcessNoiseParams process{};
        Integrator integrator{Integrator::Rk4};
    };

    // 一步更新的 NIS:编码器(自由度 2)与 IMU(自由度 1)。
    struct EkfNis
    {
        double encoder{0.0};
        double imu{0.0};
    };

    // ---------------------------------------------------------------------------
    // EkfPipeline: 估计管线 —— 原始传感器读数进,位姿估计出。
    //
    // 每步:predict → 编码器解码、在预测速度处求 R_enc → update_encoder
    //      → R_imu = σ² → update_imu。
    // 编码器与 IMU 都作为观测,不作为控制输入。输入只有 EncoderTicks 与陀螺标量,
    // 所以仿真的 Plant 与 V6 实车驱动可以喂同一条管线;V5 的 EKF 节点包装它。
    //
    // 初始位姿即"已知起点"(相当于 RViz 的 2D Pose Estimate):μ₀ 的 (px, py, θ)
    // 取 initial_pose,其余为 0;Σ₀ = default_initial_covariance()。
    // ---------------------------------------------------------------------------
    class EkfPipeline
    {
    public:
        EkfPipeline(const EkfPipelineConfig& cfg, const Pose2D& initial_pose);

        // 用 [t, t + dt] 内的编码器增量与该时刻的陀螺读数推进一步。
        EkfNis step(const EncoderTicks& dticks, double imu_omega, double dt);

        [[nodiscard]] Pose2D pose() const noexcept;
        [[nodiscard]] Twist2D velocity() const noexcept; // 估计的真实 (v, ω),不含 gyro bias
        [[nodiscard]] const Ekf& filter() const noexcept { return ekf_; }

    private:
        EkfPipelineConfig cfg_;
        Ekf ekf_;
    };
}
