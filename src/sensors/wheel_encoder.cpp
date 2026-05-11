module;

#include <cmath>
#include <cstdint>
#include <random>
#include <utility>

module mininav.sensors.wheel_encoder;

import mininav.core.types;
import mininav.core.kinematics;
import mininav.core.math;

namespace mininav
{
    namespace
    {
        [[nodiscard]] std::int64_t arclength_to_ticks(const double s_accum,
                                                      const double dist_per_tick) noexcept
        {
            return std::llround(s_accum / dist_per_tick);
        }
    }

    WheelEncoderModel::WheelEncoderModel(WheelEncoderParams params, std::mt19937 slip_rng_left,
                                         std::mt19937 slip_rng_right) noexcept
        : params_{params},
          distance_per_tick_{
              2.0 * kPi * params.wheel_radius / static_cast<double>(params.ticks_per_rev)
          },
          slip_rng_left_{slip_rng_left},
          slip_rng_right_{slip_rng_right}
    {
    }

    EncoderTicks WheelEncoderModel::measure(const Twist2D& true_velocity, const double dt)
    {
        const auto [v_l_true, v_r_true] = inverse_kinematics(true_velocity, params_.wheel_base);

        double v_l_meas = v_l_true;
        double v_r_meas = v_r_true;

        if (params_.slip_sigma > 0.0)
        {
            std::normal_distribution dist(0.0, params_.slip_sigma);
            v_l_meas = v_l_true * (1.0 + dist(slip_rng_left_));
            v_r_meas = v_r_true * (1.0 + dist(slip_rng_right_));
        }

        s_left_accum_ += v_l_meas * dt;
        s_right_accum_ += v_r_meas * dt;

        const std::int64_t ticks_left_total = arclength_to_ticks(s_left_accum_, distance_per_tick_);
        const std::int64_t ticks_right_total = arclength_to_ticks(s_right_accum_, distance_per_tick_);

        const EncoderTicks delta{
            .left  = ticks_left_total  - ticks_left_prev_,
            .right = ticks_right_total - ticks_right_prev_,
        };

        ticks_left_prev_  = ticks_left_total;
        ticks_right_prev_ = ticks_right_total;

        return delta;
    }
}
