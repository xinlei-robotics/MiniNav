import mininav.core.types;
import mininav.core.robot_model;
import mininav.core.command_source;
import mininav.core.trajectory;
import mininav.core.csv_writer;
import mininav.core.logger;

#include <cstddef>
#include <exception>
#include <filesystem>

namespace
{
    constexpr double kSimulationDt = 0.01;
    constexpr double kSimulationTotalTime = 20;
}

int main()
{
    using namespace mininav;
    namespace fs = std::filesystem;

    Trajectory<SimStateV0> trajectory;

    Pose2D pose{0.0, 0.0, 0.0};

    constexpr auto step_count = static_cast<std::size_t>(kSimulationTotalTime / kSimulationDt) + 1;
    trajectory.reserve(step_count);

    const fs::path output_path = fs::path{PROJECT_ROOT_DIR} / "data" / "traj.csv";
    mininav::Logger::info("MiniNav V0 simulation started.");
    try
    {
        for (std::size_t i = 0; i < step_count; ++i)
        {
            const StagedCommandSource command_source;
            const RobotModel model;
            const double t = static_cast<double>(i) * kSimulationDt;
            const auto cmd = command_source.command_at(t);
            trajectory.append(SimStateV0{t, pose, cmd});
            pose = model.step(pose, cmd, kSimulationDt);
        }

        write_csv(trajectory, output_path);
        mininav::Logger::info("Trajectory CSV written to " + output_path.string());
        mininav::Logger::info("MiniNav V0 simulation ended.");
    }
    catch (const std::exception& ex)
    {
        mininav::Logger::error(ex.what());
        return 1;
    }
    return 0;
}
