module;

#include <string_view>

export module mininav.core.csv_writer;

import mininav.core.trajectory;

export namespace mininav {
class CsvWriter {
public:
  void write_trajectory(const Trajectory &trajectory,
                        std::string_view output_path) const;
};
} // namespace mininav