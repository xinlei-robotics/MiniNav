module;

#include <random>
#include <utility>

module mininav.sensors.imu_model;

namespace mininav
{
    ImuModel::ImuModel(ImuParams params, std::mt19937 noise_rng) noexcept
        : params_{params},
          bias_omega_{params.bias_omega_init},
          noise_rng_{std::move(noise_rng)},
          bias_rng_{} // 缺省构造; bias_random_walk == 0 时永不被消费
    {
    }

    ImuModel::ImuModel(ImuParams params, std::mt19937 noise_rng, std::mt19937 bias_rng) noexcept
        : params_{params},
          bias_omega_{params.bias_omega_init},
          noise_rng_{std::move(noise_rng)},
          bias_rng_{std::move(bias_rng)}
    {
    }

    double ImuModel::measure(const double true_omega)
    {
        // bias 随机游走推进。bias_random_walk == 0 时不消费 bias RNG。
        if (params_.bias_random_walk > 0.0)
        {
            std::normal_distribution<double> bias_step{0.0, params_.bias_random_walk};
            bias_omega_ += bias_step(bias_rng_);
        }

        // 叠加 bias。
        double reading = true_omega + bias_omega_;

        // 叠加白噪声。sigma_omega == 0 时不消费白噪声 RNG。
        if (params_.sigma_omega > 0.0)
        {
            std::normal_distribution<double> noise{0.0, params_.sigma_omega};
            reading += noise(noise_rng_);
        }

        return reading;
    }
}
