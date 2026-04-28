module;

#include <string_view>

export module mininav.core.csv_writer;

import mininav.core.trajectory;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // CsvWriter: 将 Trajectory 写出为标准 CSV 格式
    // ---------------------------------------------------------------------------
    class CsvWriter
    {
    public:
        static void write_trajectory(const Trajectory& trajectory,
                                     std::string_view output_path);
    };
} // namespace mininav
