module;

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

module mininav.core.csv_writer;

import mininav.core.trajectory;

namespace fs = std::filesystem;

namespace mininav {
void CsvWriter::write_trajectory(const Trajectory &trajectory,
                                 std::string_view output_path) const {
  const fs::path path{output_path};
  if (path.has_parent_path()) {
    fs::create_directories(path.parent_path());
  }
  std::ofstream out{path};
  if (!out) {
    throw std::runtime_error{"Failed to open output file" + path.string()};
  }

  out << "t,x,y,yaw,v,w\n";
  for (const auto &record : trajectory.records()) {
    out << record.t << ", " << record.pose.x << ", " << record.pose.y << ", "
        << record.pose.yaw << ", " << record.twist.v << ", " << record.twist.w
        << "\n";
  }
}
} // namespace mininav