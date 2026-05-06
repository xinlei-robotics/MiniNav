module;

#include <cmath>
#include <random>

module mininav.sensors.actuator_model;

import mininav.core.types;

namespace mininav
{
    ActuatorModel::ActuatorModel(const ActuatorNoiseParams& params,
                                 const std::mt19937& rng) noexcept
        : params_{params}, rng_{rng}
    {
    }

    Twist2D ActuatorModel::apply(const Twist2D& cmd)
    {
        const double v = cmd.v();
        const double w = cmd.w();
        // Velocity Motion Model
        const double v_var = params_.alpha1 * v * v + params_.alpha2 * w * w;
        const double w_var = params_.alpha3 * v * v + params_.alpha4 * w * w;
        const double sigma_v = std::sqrt(v_var);
        const double sigma_w = std::sqrt(w_var);
        double v_noisy = v;
        double w_noisy = w;
        // 关键: sigma == 0 时不调用 RNG, 保持随机序列稳定 (见 .ixx 设计说明)。
        if (sigma_v > 0.0)
        {
            std::normal_distribution<double> dist(0.0, sigma_v);
            v_noisy += dist(rng_);
        }
        if (sigma_w > 0.0)
        {
            std::normal_distribution<double> dist(0.0, sigma_w);
            w_noisy += dist(rng_);
        }
        return Twist2D{v_noisy, w_noisy};
    }
}
