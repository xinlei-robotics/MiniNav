import mininav.core.robot_model;
import mininav.core.types;

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string_view>

namespace fs = std::filesystem;

namespace {
[[nodiscard]] mininav::Twist2D command_at(double t) noexcept {
  using mininav::Twist2D;

  if (t < 5.0) {
    return Twist2D{1.0, 0.0};
  }
  if (t < 10.0) {
    return Twist2D{1.0, 0.3};
  }
  if (t < 15.0) {
    return Twist2D{1.0, 0.0};
  }
  return Twist2D{0.5, 0.4};
}
} // namespace

int main() {
  constexpr double dt = 0.1;
  constexpr double total_time = 20.0;
  constexpr std::string_view output_file = "data/traj.csv";

  fs::create_directories("data");

  std::ofstream out(output_file.data());
  if (!out) {
    std::cerr << "Failed to open output file: " << output_file << std::endl;
    return 1;
  }

  out << "t,x,y,yaw,v,w\n";
  mininav::RobotModel model;
  mininav::Pose2D pose{};
  for (double t = 0.0; t <= total_time; t += dt) {
    const auto cmd = command_at(t);
    out << t << "," << pose.x << "," << pose.y << "," << pose.yaw << ","
        << cmd.v << "," << cmd.w << "\n";
    pose = model.step(pose, cmd, dt);
  }

  std::cout << "Trajectory saved to: " << output_file << std::endl;
  return 0;
}