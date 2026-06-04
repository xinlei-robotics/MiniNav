module;

#include <random>

export module mininav.sensors.imu_model;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // ImuParams: IMU噪声参数.
    //
    // IMU 测量模型:
    //   ω_imu = ω_true + N(0, sigma_omega²)
    //
    // sigma_omega 是 gyro 白噪声标准差(rad/s)。0.005 是消费级 MEMS gyro 的
    // 典型量级(如 BNO055 的 gyro 噪声密度 0.014 °/s/√Hz, 100 Hz 采样后约
    // 0.0024 rad/s — 0.005 是保守取值。
    // ---------------------------------------------------------------------------
    struct ImuParams
    {
        double sigma_omega{0.005}; // rad/s, gyro 白噪声标准差
    };

    // ---------------------------------------------------------------------------
    // ImuModel: gyro 仿真器。
    //
    // 设计:
    //   - 构造接收 ImuParams + 一个 std::mt19937(由 RngFactory::make_engine
    //     ("imu_gyro_noise") 产出独立子种子的 RNG)
    //   - measure(true_omega) 返回带噪 ω_imu, 每次调用消费一次 RNG
    //   - copy-disabled, move-enabled : 与 WheelEncoderModel 一致(底层 mt19937
    //     是 move-friendly 的值类型)
    //
    // sigma_omega == 0 时退化为完美 gyro: measure() 直接返回 true_omega,不消费 RNG。
    // ---------------------------------------------------------------------------
    class ImuModel
    {
    public:
        ImuModel(ImuParams params, std::mt19937 rng) noexcept;

        ImuModel(const ImuModel&) = delete;
        ImuModel& operator=(const ImuModel&) = delete;
        ImuModel(ImuModel&&) noexcept = default;
        ImuModel& operator=(ImuModel&&) noexcept = default;
        ~ImuModel() = default;

        [[nodiscard]] double measure(double true_omega);

        [[nodiscard]] const ImuParams& params() const noexcept { return params_; }

    private:
        ImuParams params_;
        std::mt19937 rng_;
    };
}
