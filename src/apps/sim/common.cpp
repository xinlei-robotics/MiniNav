module;

#include <Eigen/Core>

#include <exception>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

module mininav.apps.sim;

import mininav.core.logger;
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
}
