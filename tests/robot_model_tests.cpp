import mininav.core.robot_model;
import mininav.core.types;
import mininav.core.math;

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool nearly_equal(double a, double b, double eps = 1e-8) {
  return std::abs(a - b) < eps;
}

void expect(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "[TEST FAILED] " << message << std::endl;
    std::exit(1);
  }
}

} // namespace

int main() {
  using namespace mininav;

  RobotModel model;

  {
    Pose2D pose{0.0, 0.0, 0.0};
    Twist2D twist{1.0, 0.0};
    double dt = 1.0;

    Pose2D next_pose = model.step(pose, twist, dt);
    expect(nearly_equal(next_pose.x, 1.0), "Expected x to be 1.0");
    expect(nearly_equal(next_pose.y, 0.0), "Expected y to be 0.0");
    expect(nearly_equal(next_pose.yaw, 0.0), "Expected yaw to be 0.0");
  }

  {
    const Pose2D start{0.0, 0.0, 0.0};
    const Twist2D cmd{0.0, kPi};

    const Pose2D next_pose = model.step(start, cmd, 1.0);
    expect(nearly_equal(next_pose.x, 0.0), "Expected x to be 0.0");
    expect(nearly_equal(next_pose.y, 0.0), "Expected y to be 0.0");
    expect(nearly_equal(next_pose.yaw, kPi), "Expected yaw to be pi");
  }

  {
    const Pose2D start{0.0, 0.0, 3.5};
    const Twist2D cmd{0.0, 0.0};

    const Pose2D next_pose = model.step(start, cmd, 1.0);
    expect(next_pose.yaw <= kPi, "Expected yaw to be <= pi");
    expect(next_pose.yaw >= -kPi, "Expected yaw to be >= -pi");
  }

  std::cout << "[TEST PASSED] robot_model_tests\n" << std::endl;
  return 0;
}