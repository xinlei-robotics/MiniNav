module;

export module mininav.core.integrators;

import mininav.core.types;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // rk4_step: fourth-order Runge-Kutta integration for the planar differential
    // drive kinematic ODE:
    //
    //   x_dot     = v cos(theta)
    //   y_dot     = v sin(theta)
    //   theta_dot = omega
    //
    // The control is treated as constant over [t, t + dt].
    // ---------------------------------------------------------------------------
    [[nodiscard]] Pose2D rk4_step(const Pose2D& current, const Twist2D& control, double dt) noexcept;
}
