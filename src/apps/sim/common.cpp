module;

#include <Eigen/Core>

#include <algorithm>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <numbers>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

module mininav.apps.sim;

import mininav.core.types;
import mininav.core.path;
import mininav.core.logger;
import mininav.core.robot_description;
import mininav.simulation.noise_presets;
import mininav.localization.ekf;
import mininav.localization.ekf_pipeline;
import mininav.localization.encoder_observation;
import mininav.planning.grid_types;
import mininav.planning.occupancy_grid;
import mininav.viz.plan_log;
import mininav.viz.rerun_sink;

namespace mininav::apps
{
    namespace fs = std::filesystem;

    fs::path output_csv_path(const OutputOptions& output, const std::string_view default_file_name)
    {
        return output.out_path.has_value()
                   ? fs::path{*output.out_path}
                   : fs::path{PROJECT_ROOT_DIR} / "data" / std::string{default_file_name};
    }

    std::optional<RerunSink> make_sink(const OutputOptions& output)
    {
        if (output.disable_viz)
        {
            log::info("Rerun: disabled by --no-viz.");
            return std::nullopt;
        }

        std::optional<RerunSink> sink;
        if (output.rrd_path.has_value())
        {
            sink.emplace(kApplicationId, fs::path{*output.rrd_path});
            log::info("Rerun: writing to " + *output.rrd_path);
        }
        else
        {
            sink.emplace(kApplicationId);
            log::info("Rerun: Viewer spawned (gRPC).");
        }
        return sink;
    }

    Eigen::Vector2d parse_xy(const std::string& s)
    {
        const auto comma = s.find(',');
        if (comma == std::string::npos)
        {
            throw std::runtime_error{"expected \"x,y\", got: " + s};
        }
        try
        {
            const double x = std::stod(s.substr(0, comma));
            const double y = std::stod(s.substr(comma + 1));
            return Eigen::Vector2d{x, y};
        }
        catch (const std::exception&)
        {
            throw std::runtime_error{"could not parse \"x,y\" from: " + s};
        }
    }

    PoseArg parse_pose(const std::string& s)
    {
        std::vector<double> values;
        std::stringstream ss{s};
        std::string field;
        while (std::getline(ss, field, ','))
        {
            try
            {
                std::size_t used = 0;
                values.push_back(std::stod(field, &used));
                if (used != field.size())
                {
                    throw std::invalid_argument{field};
                }
            }
            catch (const std::exception&)
            {
                throw std::runtime_error{"could not parse \"x,y[,yaw]\" from: " + s};
            }
        }
        if (values.size() != 2 && values.size() != 3)
        {
            throw std::runtime_error{"expected \"x,y\" or \"x,y,yaw\", got: " + s};
        }
        const double yaw = values.size() == 3 ? values[2] : 0.0;
        return PoseArg{.pose = Pose2D{values[0], values[1], yaw}, .has_yaw = values.size() == 3};
    }

    std::uint64_t resolve_seed(const std::optional<std::uint64_t> requested)
    {
        if (requested.has_value())
        {
            return *requested;
        }
        std::random_device rd;
        const auto hi = static_cast<std::uint64_t>(rd());
        const auto lo = static_cast<std::uint64_t>(rd());
        return (hi << 32) ^ lo;
    }

    ekf::EkfPipelineConfig make_pipeline_config(const RobotDescription& robot,
                                                const simulation::NoisePreset& preset,
                                                const EkfTuning& tuning)
    {
        // 陀螺的量化下限(与编码器 R 的量化 floor 同理):BNO055 陀螺 1 LSB = 1/16 °/s,
        // 均匀量化的标准差 LSB/√12 ≈ 3.2e-4 rad/s。滤波器的 R_imu 必须严格为正
        // (Ekf::update_imu 的前置条件);三档带噪声的预设都远大于它,取 max 不改变其值,
        // 只有无噪声预设(none)落到这个下限。
        constexpr double kGyroQuantizationSigma = (1.0 / 16.0) * std::numbers::pi / 180.0 / 3.4641016151377544;

        return ekf::EkfPipelineConfig{
            .encoder = EncoderNoiseParams{
                .sigma_slip = preset.slip_sigma,
                .distance_per_tick = robot.distance_per_tick(),
                .wheel_base = robot.wheel_base,
            },
            .sigma_imu = std::max(preset.sigma_imu, kGyroQuantizationSigma),
            .r_scale = tuning.r_scale,
            .process = ekf::ProcessNoiseParams{
                .alpha1 = preset.alpha1 * tuning.q_scale,
                .alpha2 = preset.alpha2 * tuning.q_scale,
                .alpha3 = preset.alpha3 * tuning.q_scale,
                .alpha4 = preset.alpha4 * tuning.q_scale,
                .q_bias_omega = tuning.bias_on ? preset.q_bias_omega * tuning.q_scale : 0.0,
                .q_dv = tuning.q_dv,
                .q_dw = tuning.q_dw,
            },
            .integrator = tuning.integrator,
        };
    }

    std::string heuristic_name_of(const planning::Heuristic h)
    {
        using planning::Heuristic;
        switch (h)
        {
        case Heuristic::Manhattan: return "manhattan";
        case Heuristic::Euclidean: return "euclidean";
        case Heuristic::Octile: return "octile";
        }
        return "euclidean";
    }

    Eigen::Vector2d grid_center(const planning::OccupancyGrid& g)
    {
        return g.origin() + Eigen::Vector2d{
                   g.width() * g.resolution() * 0.5,
                   g.height() * g.resolution() * 0.5
               };
    }

    PlanScene build_plan_scene(const planning::OccupancyGrid& grid,
                               const planning::OccupancyGrid& inflated,
                               const Path& path, const Pose2D& start, const Pose2D& goal)
    {
        using namespace planning;
        PlanScene scene;
        scene.start = start;
        scene.goal = goal;
        scene.cell_radius = static_cast<float>(grid.resolution() * 0.5);

        for (int y = 0; y < grid.height(); ++y)
        {
            for (int x = 0; x < grid.width(); ++x)
            {
                const GridCoord c{x, y};
                const Eigen::Vector2d w = grid.grid_to_world(c);
                if (grid.at(c) == kOccupied)
                {
                    scene.obstacle_cells.push_back(w);
                }
                else if (inflated.at(c) == kOccupied)
                {
                    scene.inflated_cells.push_back(w); // 膨胀新增的安全裕度
                }
            }
        }

        scene.path.reserve(path.poses.size());
        for (const Pose2D& p : path.poses)
        {
            scene.path.emplace_back(p.x(), p.y());
        }
        return scene;
    }
}
