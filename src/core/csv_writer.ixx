module;

#include <filesystem>
#include <fstream>
#include <stdexcept>

export module mininav.core.csv_writer;

import mininav.core.trajectory;
import mininav.core.csv_format;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // write_csv: 将任意 Trajectory<T> 写出为 CSV 文件
    // 通过 ADL 调用 csv_header(T{}) 与 csv_row(record) 来完成序列化,
    // 因此 T 必须在它所在的命名空间中提供这两个自由函数重载
    // ---------------------------------------------------------------------------
    template <typename T>
    void write_csv(const Trajectory<T>& trajectory, const std::filesystem::path& path)
    {
        namespace fs = std::filesystem;
        if (path.has_parent_path())
        {
            fs::create_directory(path.parent_path());
        }

        std::ofstream out{path};
        if (!out)
        {
            throw std::runtime_error{"Failed to open output file: " + path.string()};
        }

        out << csv_header(T{}) << '\n';
        for (const auto& record : trajectory.records())
        {
            out << csv_row(record) << '\n';
        }
    }
} // namespace mininav
