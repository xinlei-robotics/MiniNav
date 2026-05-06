module;

#include <random>

export module mininav.sensors.actuator_model;

import mininav.core.types;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // ActuatorNoiseParams: Velocity Motion Model 的四个标定参数 (Thrun §5.3)
    //
    //   sigma_v² = alpha1 * v²  +  alpha2 * w²
    //   sigma_w² = alpha3 * v²  +  alpha4 * w²
    // ---------------------------------------------------------------------------

    struct ActuatorNoiseParams
    {
        double alpha1{0.0};
        double alpha2{0.0};
        double alpha3{0.0};
        double alpha4{0.0};
    };

    class ActuatorModel
    {
    public:
        ActuatorModel(const ActuatorNoiseParams& params, const std::mt19937& rng) noexcept;

        ActuatorModel(const ActuatorModel&)            = delete;
        ActuatorModel& operator=(const ActuatorModel&) = delete;
        ActuatorModel(ActuatorModel&&) noexcept        = default;
        ActuatorModel& operator=(ActuatorModel&&) noexcept = default;
        ~ActuatorModel()                                = default;

        // 应用执行噪声: cmd → (v_true, w_true)。每次调用消耗 0~2 个 RNG 样本。
        [[nodiscard]] Twist2D apply(const Twist2D& cmd);

        [[nodiscard]] const ActuatorNoiseParams& params() const noexcept
        {
            return params_;
        }

    private:
        ActuatorNoiseParams params_;
        std::mt19937 rng_;
    };
}
