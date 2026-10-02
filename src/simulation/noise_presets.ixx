module;

#include <array>
#include <stdexcept>
#include <string>
#include <string_view>

export module mininav.simulation.noise_presets;

export namespace mininav::simulation
{
    // ---------------------------------------------------------------------------
    // NoisePreset: 三档噪声标定(工程合理值)。
    //
    // 前七个字段是被仿真世界的真实噪声(执行器 / 编码器 / IMU);q_bias_omega 是与
    // 该档位配对的 EKF 调参(bias 过程噪声,单步方差),不影响真值,由 app 交给
    // EkfPipeline。
    //
    // imu_bias_rw 默认置 0(常数 bias); 调成小正数(如 5e-5)即可观察 filter 跟踪
    // 漂移 bias 的能力。EKF 侧 q_bias_omega 保持小正值, 使 filter 即便面对常数 bias
    // 也维持一点自适应余量(标准工业做法)。
    // ---------------------------------------------------------------------------
    struct NoisePreset
    {
        std::string_view name;
        double alpha1, alpha2, alpha3, alpha4; // Velocity Motion Model
        double slip_sigma;                     // 编码器打滑标准差
        double sigma_imu;                      // IMU gyro 白噪声标准差 [rad/s]
        double imu_bias_init;                  // IMU gyro bias 真值(filter 待估计)[rad/s]
        double imu_bias_rw;                    // bias 每步随机游走标准差 [rad/s]; 0 => 常数 bias
        double q_bias_omega;                   // EKF 对 bias 的过程噪声(单步方差)(rad/s)²
    };

    // 无噪声:真值 = 指令经执行器动力学后的运动学积分,传感器只剩量化。用于把控制
    // 误差与噪声、定位漂移彻底拆开(闭环验收的"无噪声运行")。q_bias_omega = 0 时
    // EKF 走无 bias 兼容路径。
    inline constexpr NoisePreset kPresetNone{
        .name = "none",
        .alpha1 = 0.0, .alpha2 = 0.0, .alpha3 = 0.0, .alpha4 = 0.0,
        .slip_sigma = 0.0,
        .sigma_imu = 0.0,
        .imu_bias_init = 0.0,
        .imu_bias_rw = 0.0,
        .q_bias_omega = 0.0,
    };
    inline constexpr NoisePreset kPresetLowNoise{
        .name = "low-noise",
        .alpha1 = 0.01, .alpha2 = 0.005, .alpha3 = 0.005, .alpha4 = 0.01,
        .slip_sigma = 0.005,
        .sigma_imu = 0.002,
        .imu_bias_init = 0.01,
        .imu_bias_rw = 0.0,
        .q_bias_omega = 1e-8,
    };
    inline constexpr NoisePreset kPresetDefault{
        .name = "default",
        .alpha1 = 0.05, .alpha2 = 0.02, .alpha3 = 0.02, .alpha4 = 0.05,
        .slip_sigma = 0.02,
        .sigma_imu = 0.005,
        .imu_bias_init = 0.02,
        .imu_bias_rw = 0.0,
        .q_bias_omega = 1e-8,
    };
    inline constexpr NoisePreset kPresetHighNoise{
        .name = "high-noise",
        .alpha1 = 0.15, .alpha2 = 0.08, .alpha3 = 0.08, .alpha4 = 0.15,
        .slip_sigma = 0.05,
        .sigma_imu = 0.015,
        .imu_bias_init = 0.03,
        .imu_bias_rw = 0.0,
        .q_bias_omega = 4e-8,
    };

    inline constexpr std::array kNoisePresets{kPresetNone, kPresetLowNoise, kPresetDefault, kPresetHighNoise};

    // 按名字查档位;未知名字抛 std::invalid_argument(CLI 层另有 IsMember 校验)。
    [[nodiscard]] constexpr const NoisePreset& noise_preset(const std::string_view name)
    {
        for (const NoisePreset& preset : kNoisePresets)
        {
            if (preset.name == name)
            {
                return preset;
            }
        }
        throw std::invalid_argument{"unknown noise preset '" + std::string{name} +
                                    "' (expected none, low-noise, default, high-noise)"};
    }
}
