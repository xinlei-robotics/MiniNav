module;

#include <random>

export module mininav.sensors.wheel_encoder;

import mininav.core.types;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // WheelEncoderParams: 物理常数 + 噪声参数
    //
    // 物理常数:
    //   wheel_radius:     轮半径 [m]
    //   wheel_base:       左右轮中心距 [m]
    //   ticks_per_rev:    每圈 tick 数
    //
    // 噪声参数:
    //   slip_sigma: 打滑乘性高斯噪声标准差 (无量纲, 典型 0.005~0.05)
    //               左右轮各自独立采样, 但共享此参数。
    //               物理含义: v_meas = v_true * (1 + N(0, slip_sigma²))
    // ---------------------------------------------------------------------------
    struct WheelEncoderParams
    {
        double wheel_radius{0.032}; // Adeept 4WD 小车的轮半径约为 32mm
        double wheel_base{0.150}; // Adeept 4WD 小车的左右轮中心距约为 150mm
        std::int64_t ticks_per_rev{1024}; // 典型增量式编码器每圈 512 或 1024 tick
        double slip_sigma{0.0}; // 默认 slip_sigma = 0 → 退化为只有量化误差的"完美编码器"。
    };

    class WheelEncoderModel
    {
    public:
        WheelEncoderModel(WheelEncoderParams params, std::mt19937 slip_rng_left, std::mt19937 slip_rng_right) noexcept;

        WheelEncoderModel(const WheelEncoderModel&) = delete;
        WheelEncoderModel& operator=(const WheelEncoderModel&) = delete;
        WheelEncoderModel(WheelEncoderModel&&) noexcept = default;
        WheelEncoderModel& operator=(WheelEncoderModel&&) noexcept = default;
        ~WheelEncoderModel() = default;

        [[nodiscard]] EncoderTicks measure(const Twist2D& true_velocity, double dt);

        [[nodiscard]] const WheelEncoderParams& params() const noexcept
        {
            return params_;
        }

        [[nodiscard]] double distance_per_tick() const noexcept
        {
            return distance_per_tick_;
        }

    private:
        WheelEncoderParams params_;
        double distance_per_tick_;

        std::mt19937 slip_rng_left_;
        std::mt19937 slip_rng_right_;

        double s_left_accum_{0.0};
        double s_right_accum_{0.0};

        std::int64_t ticks_left_prev_{0};
        std::int64_t ticks_right_prev_{0};
    };
}
