module;

#include <utility>

export module mininav.core.kinematics;

import mininav.core.types;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // differential_drive_step: 差分驱动机器人的积分入口。
    //
    // Kept as the stable public kinematics API; internally delegates to rk4_step.
    // ---------------------------------------------------------------------------
    [[nodiscard]] Pose2D differential_drive_step(const Pose2D& current, const Twist2D& control, double dt) noexcept;

    [[nodiscard]] std::pair<double, double> inverse_kinematics(const Twist2D& body, double wheel_base) noexcept;

    [[nodiscard]] Twist2D forward_kinematics(double v_left, double v_right, double wheel_base) noexcept;

    // ---------------------------------------------------------------------------
    // twist_from_ticks: 一段时间窗 [t, t+dt] 内的编码器增量 → 本体速度 (v, ω)。
    //
    //   v_wheel = Δticks · distance_per_tick / dt
    //   (v, ω)  = forward_kinematics(v_left, v_right, wheel_base)
    //
    // 单一解码路径, 同时供 WheelOdometry(估计基线)与 decode_encoder(EKF 观测)
    // 复用; V6 真实机器人上 GPIO 中断计数走的也是这同一条路径(dt > 0 由调用方保证)。
    // ---------------------------------------------------------------------------
    [[nodiscard]] Twist2D twist_from_ticks(const EncoderTicks& dticks,
                                           double distance_per_tick,
                                           double wheel_base, double dt) noexcept;
}
