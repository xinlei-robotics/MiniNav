// ===========================================================================
// 积分器一致性: process_model_g 的位姿子向量 (px, py, θ) 必须与仿真真值所用的
// differential_drive_step 完全一致。
//
// 这条测试固化一个不变量: 滤波器对世界的运动模型与世界本身共用同一个 rk4_step
// —— g 把位置积分委托给 differential_drive_step / rk4_step, 而不是另写一份。
// 若未来有人在 g 里手写积分(哪怕只是欧拉), 就会悄悄重新引入"真值 RK4 / 预测欧拉"
// 的积分器失配, 这条测试会立刻报错。它与 ekf_jacobian_finite_diff_tests
// (校验 G 与 g 一致)互补: 一个钉住"g == 真值积分器", 一个钉住"G == ∂g/∂x"。
// ===========================================================================
#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <random>

import mininav.localization.ekf;
import mininav.localization.ekf_state;
import mininav.core.kinematics;
import mininav.core.types;
import mininav.core.math;

using namespace mininav;

namespace
{
    Vec6 state_of(double px, double py, double th, double v, double w, double b = 0.0)
    {
        Vec6 mu;
        mu << px, py, th, v, w, b;
        return mu;
    }

    // 比对 g(mu) 的位姿子向量与 differential_drive_step 的输出。
    //   位置: 逐位相等(g 直接委托 rk4_step, 同参数 ⇒ 同结果)。
    //   航向: g 刻意不 wrap、真值 wrap, 故比较二者的角度差(wrap 后 ≈ 0), 这样在
    //         ±π 边界附近也稳健。
    //   v / ω / b_ω: 不受积分器影响, 应恒等透传。
    void expect_pose_matches_integrator(const Vec6& mu, double dt)
    {
        const Vec6 g = process_model_g(mu, dt);
        const Pose2D truth = differential_drive_step(
            Pose2D{mu(kPx), mu(kPy), mu(kTheta)},
            Twist2D{mu(kV), mu(kOmega)},
            dt);

        EXPECT_DOUBLE_EQ(g(kPx), truth.x());
        EXPECT_DOUBLE_EQ(g(kPy), truth.y());
        EXPECT_NEAR(wrap_angle(g(kTheta) - truth.yaw()), 0.0, 1e-12);
        EXPECT_DOUBLE_EQ(g(kV), mu(kV));
        EXPECT_DOUBLE_EQ(g(kOmega), mu(kOmega));
        EXPECT_DOUBLE_EQ(g(kBiasOmega), mu(kBiasOmega));
    }
}

TEST(EkfIntegratorConsistency, PoseMatchesDifferentialDriveStep_RepresentativeStates)
{
    constexpr double dt = 0.01;
    expect_pose_matches_integrator(state_of(0.0, 0.0, 0.0, 0.5, 0.0), dt);          // 直线
    expect_pose_matches_integrator(state_of(0.0, 0.0, 0.0, 0.0, 1.0), dt);          // 原地旋转
    expect_pose_matches_integrator(state_of(1.0, -2.0, 0.7, 0.6, 0.4), dt);         // 一般曲线
    expect_pose_matches_integrator(state_of(0.0, 0.0, -1.2, -0.3, -0.8, 0.05), dt); // 倒车 + 负角速度
}

TEST(EkfIntegratorConsistency, PoseMatchesDifferentialDriveStep_AcrossWrapBoundary)
{
    // θ + ω·dt 越过 +π: g 返回未 wrap 的值, 真值已 wrap; 二者的角度差仍应 ≈ 0。
    constexpr double dt = 0.1;
    const Vec6 mu = state_of(0.0, 0.0, 3.10, 0.5, 1.0); // 3.10 + 0.1 = 3.20 > π
    const Vec6 g = process_model_g(mu, dt);

    EXPECT_GT(g(kTheta), kPi); // g 自身不 wrap, 这是被 predict() 依赖的约定
    expect_pose_matches_integrator(mu, dt);
}

TEST(EkfIntegratorConsistency, PoseMatchesDifferentialDriveStep_RandomSweep)
{
    std::mt19937 rng{1234};
    std::uniform_real_distribution<double> pos(-3.0, 3.0);
    std::uniform_real_distribution<double> ang(-kPi, kPi);
    std::uniform_real_distribution<double> vel(-1.5, 1.5);
    std::uniform_real_distribution<double> dt_dist(0.005, 0.05);

    for (int i = 0; i < 500; ++i)
    {
        const Vec6 mu = state_of(pos(rng), pos(rng), ang(rng), vel(rng), vel(rng));
        expect_pose_matches_integrator(mu, dt_dist(rng));
    }
}

// --- Euler 路径(归因实验用)-----------------------------------------------

TEST(EkfIntegrator, EulerMatchesClosedFormFirstOrder)
{
    constexpr double dt = 0.02;
    const Vec6 mu = state_of(1.0, -0.5, 0.7, 0.6, 0.4);
    const Vec6 g = process_model_g(mu, dt, Integrator::Euler);

    EXPECT_DOUBLE_EQ(g(kPx), mu(kPx) + mu(kV) * std::cos(mu(kTheta)) * dt);
    EXPECT_DOUBLE_EQ(g(kPy), mu(kPy) + mu(kV) * std::sin(mu(kTheta)) * dt);
    EXPECT_DOUBLE_EQ(g(kTheta), mu(kTheta) + mu(kOmega) * dt);
}

TEST(EkfIntegrator, EulerAndRk4DifferWhenTurning)
{
    // 转弯(ω≠0)时 Simpson 与一阶欧拉对位置的积分必然不同 —— 这保证 --integrator
    // 开关确实改变了滤波器行为, 归因实验才有意义。
    constexpr double dt = 0.05;
    const Vec6 mu = state_of(0.0, 0.0, 0.3, 1.0, 1.5);

    const Vec6 g_euler = process_model_g(mu, dt, Integrator::Euler);
    const Vec6 g_rk4 = process_model_g(mu, dt, Integrator::Rk4);

    EXPECT_GT(std::hypot(g_rk4(kPx) - g_euler(kPx), g_rk4(kPy) - g_euler(kPy)), 1e-6);
}

TEST(EkfIntegrator, EulerAndRk4CoincideWhenNotTurning)
{
    // ω=0 时步内航向不变, Simpson 三个采样点重合 ⇒ RK4 退化为欧拉, 二者逐位相等。
    constexpr double dt = 0.05;
    const Vec6 mu = state_of(0.2, 0.4, 1.1, 0.8, 0.0);

    const Vec6 g_euler = process_model_g(mu, dt, Integrator::Euler);
    const Vec6 g_rk4 = process_model_g(mu, dt, Integrator::Rk4);

    EXPECT_DOUBLE_EQ(g_rk4(kPx), g_euler(kPx));
    EXPECT_DOUBLE_EQ(g_rk4(kPy), g_euler(kPy));
}
