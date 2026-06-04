module;

#include <random>
#include <utility>

module mininav.sensors.imu_model;

namespace mininav
{
    ImuModel::ImuModel(ImuParams params, std::mt19937 rng) noexcept
        : params_{params}, rng_{std::move(rng)}
    {
    }

    double ImuModel::measure(const double true_omega)
    {
        // sigma_omega == 0: 完美 gyro。不调用 distribution, 不消费 RNG, 直接返回 true_omega。
        if (params_.sigma_omega <= 0.0)
        {
            return true_omega;
        }
        std::normal_distribution<double> dist{0.0, params_.sigma_omega};
        return true_omega + dist(rng_);
    }
}
