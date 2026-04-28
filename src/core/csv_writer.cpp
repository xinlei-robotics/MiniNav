module;

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

module mininav.core.csv_writer;

import mininav.core.trajectory;

namespace fs = std::filesystem;

namespace mininav
{
    void CsvWriter::write_trajectory(const Trajectory& trajectory,
                                     std::string_view output_path)
    {
        const fs::path path{output_path};
        if (path.has_parent_path())
        {
            fs::create_directories(path.parent_path());
        }

        std::ofstream out{path};
        if (!out)
        {
            throw std::runtime_error{"Failed to open output file" + path.string()};
        }

        out.precision(std::numeric_limits<double>::digits10);
        out << std::scientific;

        out << "t,x,y,yaw,v,w\n";
        for (const auto& [t, pose, twist] : trajectory.records())
        {
            out << t << ','
                << pose.x() << ','
                << pose.y() << ", "
                << pose.yaw() << ','
                << twist.v() << ','
                << twist.w() << '\n';
        }
    }
} // namespace mininav
