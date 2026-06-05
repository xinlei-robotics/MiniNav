module;

#include <random>

export module mininav.sensors.imu_model;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // ImuParams: IMU 噪声 + bias 参数。
    //
    // IMU 测量模型(PR4):
    //   ω_imu = ω_true + b_ω + N(0, sigma_omega²)
    //
    // sigma_omega: gyro 白噪声标准差(rad/s)。0.005 是消费级 MEMS gyro 的典型量级
    //   (如 BNO055 gyro 噪声密度 0.014 °/s/√Hz, 100 Hz 采样后约 0.0024 rad/s —
    //   0.005 是保守取值)。
    //
    // bias_omega_init: gyro bias 的初始(真)值(rad/s)。真实 MEMS gyro 上电后存在
    //   ~0.01–0.05 rad/s 量级的零偏。
    //
    // bias_random_walk: bias 每步随机游走的标准差(rad/s)。> 0 时 bias 缓慢漂移
    //   (移动靶), filter 需持续跟踪; = 0 时 bias 为常数。
    // ---------------------------------------------------------------------------
    struct ImuParams
    {
        double sigma_omega{0.005}; // rad/s, gyro 白噪声标准差
        double bias_omega_init{0.0}; // rad/s, gyro bias 初始(真)值
        double bias_random_walk{0.0}; // rad/s, bias 每步随机游走标准差(0 => 常数 bias)
    };

    // ---------------------------------------------------------------------------
    // ImuModel: gyro 仿真器(白噪声 + 可漂移 bias)。
    //
    // 设计:
    //   - 持有两路独立 RNG: 一路用于白噪声, 一路用于 bias 随机游走。两路均应由
    //     RngFactory::make_engine 以不同 tag("imu_gyro_noise" / "imu_gyro_bias")
    //     产出独立子种子, 互不干扰、也不扰动 V1 已有随机序列。
    //   - measure(true_omega): (可选)推进 bias 随机游走 → 返回 ω_true + b_ω + 白噪声,
    //     每步最多各消费一次对应 RNG。
    //   - bias_omega(): 返回当前(真)bias, 供日志/对照(filter 学到的估计应趋近它)。
    //   - copy-disabled, move-enabled: 与 WheelEncoderModel 一致。
    //
    // 退化行为(保持确定性与 RNG 独立性):
    //   sigma_omega   == 0 → 不消费白噪声 RNG;
    //   bias_random_walk == 0 → 不消费 bias RNG, bias 恒为 bias_omega_init。
    // ---------------------------------------------------------------------------
    class ImuModel
    {
    public:
        // 单 RNG 构造: 仅提供白噪声 RNG; bias 随机游走 RNG 缺省构造。
        ImuModel(ImuParams params, std::mt19937 noise_rng) noexcept;

        // 双 RNG 构造: 同时提供白噪声与 bias 随机游走两路独立 RNG。
        ImuModel(ImuParams params, std::mt19937 noise_rng, std::mt19937 bias_rng) noexcept;

        ImuModel(const ImuModel&) = delete;
        ImuModel& operator=(const ImuModel&) = delete;
        ImuModel(ImuModel&&) noexcept = default;
        ImuModel& operator=(ImuModel&&) noexcept = default;
        ~ImuModel() = default;

        [[nodiscard]] double measure(double true_omega);

        [[nodiscard]] double bias_omega() const noexcept { return bias_omega_; }
        [[nodiscard]] const ImuParams& params() const noexcept { return params_; }

    private:
        ImuParams params_;
        double bias_omega_;
        std::mt19937 noise_rng_;
        std::mt19937 bias_rng_;
    };
}
