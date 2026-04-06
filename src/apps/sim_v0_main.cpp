import mininav.core.robot_model;
import mininav.core.types;
import mininav.core.command_source;
import mininav.core.trajectory;
import mininav.core.logger;
import mininav.core.csv_writer;

#include <exception>
#include <filesystem>
#include <string>

int main() {
  constexpr double dt = 0.01;
  constexpr double total_time = 20.0;

  mininav::Logger logger;
  mininav::RobotModel model;
  mininav::StagedCommandSource command_source;
  mininav::Trajectory trajectory;
  mininav::Pose2D pose{0.0, 0.0, 0.0};
  mininav::CsvWriter writer;

  const std::string output_path =
      std::string(PROJECT_ROOT_DIR) + "/data/traj.csv";

  logger.info("MiniNav V0 Simulation Started.");
  try {
    for (double t = 0.0; t <= total_time; t += dt) {
      const auto cmd = command_source.command_at(t);
      trajectory.append(mininav::StateRecord{t, pose, cmd});
      pose = model.step(pose, cmd, dt);
    }

    writer.write_trajectory(trajectory, output_path);
    logger.info("Trajectory CSV written to " + output_path);
    logger.info("MiniNav V0 Simulation finished.");
  } catch (const std::exception &ex) {
    logger.error(ex.what());
    return 1;
  }

  return 0;
}