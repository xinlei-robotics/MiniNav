import mininav.core.robot_model;
import mininav.core.kinematics;
import mininav.core.types;
import mininav.core.math;

#include <gtest/gtest.h>

namespace {

    constexpr double kEps = 1e-12;

} // namespace

// ---------------------------------------------------------------------------
// RobotModel 在 V0 阶段是 differential_drive_step 的多态包装。
// 此处只验证"委托"的行为正确,详尽的运动学测试在 kinematics_tests.cpp 中。
// ---------------------------------------------------------------------------

TEST(RobotModel, StepDelegatesToKinematics) {
    const mininav::RobotModel model;
    const mininav::Pose2D start{1.0, 2.0, 0.3};
    const mininav::Twist2D cmd{0.5, 0.1};
    const double dt = 0.05;

    const auto via_model = model.step(start, cmd, dt);
    const auto via_free_function =
        mininav::differential_drive_step(start, cmd, dt);

    EXPECT_NEAR(via_model.x(), via_free_function.x(), kEps);
    EXPECT_NEAR(via_model.y(), via_free_function.y(), kEps);
    EXPECT_NEAR(via_model.yaw(), via_free_function.yaw(), kEps);
}

TEST(RobotModel, StepIsConst) {
    // 这个测试主要是编译期保证:如果 step 不是 const,下面这行就编不过。
    const mininav::RobotModel model;
    const mininav::Pose2D start{0.0, 0.0, 0.0};
    const mininav::Twist2D cmd{1.0, 0.0};
    const auto next = model.step(start, cmd, 1.0);
    EXPECT_NEAR(next.x(), 1.0, kEps);
}