module;

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <cctype>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

module mininav.planning.map_io;

import mininav.planning.occupancy_grid;

namespace mininav::planning
{
    namespace
    {
        namespace fs = std::filesystem;

        // 解析后的 PGM:pixels 行优先、自上而下(图像原始行序),长度 width*height,
        // 取值已归一到 [0, maxval]。
        struct Pgm
        {
            int width{0};
            int height{0};
            int maxval{0};
            std::vector<int> pixels;
        };

        [[nodiscard]] std::string read_file_binary(const fs::path& path, const char* what)
        {
            std::ifstream in{path, std::ios::binary};
            if (!in)
            {
                throw std::runtime_error(std::string{"map_io: cannot open "} + what +
                                         " '" + path.string() + "'");
            }
            std::ostringstream ss;
            ss << in.rdbuf();
            return ss.str();
        }

        // 从 bytes[pos] 起跳过空白与 '#' 注释行;返回下一个非空白/非注释字符的下标。
        [[nodiscard]] std::size_t skip_ws_and_comments(const std::string& bytes, std::size_t pos)
        {
            while (pos < bytes.size())
            {
                const unsigned char c = static_cast<unsigned char>(bytes[pos]);
                if (c == '#')
                {
                    while (pos < bytes.size() && bytes[pos] != '\n')
                    {
                        ++pos;
                    }
                }
                else if (std::isspace(c) != 0)
                {
                    ++pos;
                }
                else
                {
                    break;
                }
            }
            return pos;
        }

        // 读取一个十进制整型 header token(width/height/maxval)。
        [[nodiscard]] int read_header_int(const std::string& bytes, std::size_t& pos,
                                          const std::string& path, const char* field)
        {
            pos = skip_ws_and_comments(bytes, pos);
            const std::size_t start = pos;
            while (pos < bytes.size() && std::isdigit(static_cast<unsigned char>(bytes[pos])) != 0)
            {
                ++pos;
            }
            if (pos == start)
            {
                throw std::runtime_error("map_io: malformed PGM header (expected " +
                                         std::string{field} + ") in '" + path + "'");
            }
            return std::stoi(bytes.substr(start, pos - start));
        }

        [[nodiscard]] Pgm parse_pgm(const std::string& bytes, const std::string& path)
        {
            if (bytes.size() < 2 || bytes[0] != 'P' || (bytes[1] != '2' && bytes[1] != '5'))
            {
                throw std::runtime_error("map_io: not a PGM (bad magic, expected P2/P5) in '" +
                                         path + "'");
            }
            const bool ascii = bytes[1] == '2';

            Pgm pgm{};
            std::size_t pos = 2;
            pgm.width = read_header_int(bytes, pos, path, "width");
            pgm.height = read_header_int(bytes, pos, path, "height");
            pgm.maxval = read_header_int(bytes, pos, path, "maxval");

            if (pgm.width <= 0 || pgm.height <= 0)
            {
                throw std::runtime_error("map_io: PGM dimensions must be positive in '" +
                                         path + "'");
            }
            if (pgm.maxval <= 0 || pgm.maxval > 255)
            {
                throw std::runtime_error("map_io: only 8-bit PGM (0 < maxval <= 255) supported in '" +
                                         path + "'");
            }

            const std::size_t count =
                static_cast<std::size_t>(pgm.width) * static_cast<std::size_t>(pgm.height);
            pgm.pixels.reserve(count);

            if (ascii)
            {
                // P2:剩余部分是空白分隔的十进制整数。
                while (pgm.pixels.size() < count)
                {
                    pos = skip_ws_and_comments(bytes, pos);
                    const std::size_t start = pos;
                    while (pos < bytes.size() &&
                           std::isdigit(static_cast<unsigned char>(bytes[pos])) != 0)
                    {
                        ++pos;
                    }
                    if (pos == start)
                    {
                        break; // 数据提前耗尽
                    }
                    pgm.pixels.push_back(std::stoi(bytes.substr(start, pos - start)));
                }
            }
            else
            {
                // P5:maxval 之后恰好一个空白,随后是 width*height 个原始字节。
                ++pos; // 跳过 maxval 后的单个分隔空白
                for (std::size_t i = 0; i < count && pos < bytes.size(); ++i, ++pos)
                {
                    pgm.pixels.push_back(static_cast<unsigned char>(bytes[pos]));
                }
            }

            if (pgm.pixels.size() != count)
            {
                throw std::runtime_error(
                    "map_io: PGM pixel count " + std::to_string(pgm.pixels.size()) +
                    " does not match width*height " + std::to_string(count) + " in '" + path + "'");
            }
            return pgm;
        }

        // 灰度像素 → 占据三值(对齐 ROS map_server)。
        //   occ = negate ? (p/maxval) : (1 - p/maxval);暗像素 → occ 高 → 占据。
        [[nodiscard]] std::int8_t classify(const int pixel, const int maxval, const bool negate,
                                           const double occupied_thresh, const double free_thresh)
        {
            const double norm = static_cast<double>(pixel) / static_cast<double>(maxval);
            const double occ = negate ? norm : (1.0 - norm);
            if (occ > occupied_thresh)
            {
                return kOccupied;
            }
            if (occ < free_thresh)
            {
                return kFree;
            }
            return kUnknown;
        }

        [[nodiscard]] YAML::Node require_key(const YAML::Node& node, const char* key,
                                             const std::string& path)
        {
            if (!node[key])
            {
                throw std::runtime_error("map_io: missing required key '" + std::string{key} +
                                         "' in '" + path + "'");
            }
            return node[key];
        }
    } // namespace

    OccupancyGrid load_occupancy_grid(const std::string& map_yaml_path)
    {
        const fs::path yaml_path{map_yaml_path};
        const std::string yaml_text = read_file_binary(yaml_path, "map yaml");

        YAML::Node node;
        try
        {
            node = YAML::Load(yaml_text);
        }
        catch (const YAML::Exception& e)
        {
            throw std::runtime_error("map_io: failed to parse yaml '" + map_yaml_path +
                                     "': " + e.what());
        }

        const auto image_rel = require_key(node, "image", map_yaml_path).as<std::string>();
        const double resolution = require_key(node, "resolution", map_yaml_path).as<double>();
        const YAML::Node origin_node = require_key(node, "origin", map_yaml_path);
        if (!origin_node.IsSequence() || origin_node.size() < 2)
        {
            throw std::runtime_error("map_io: 'origin' must be a sequence [x, y, yaw] in '" +
                                     map_yaml_path + "'");
        }
        const Eigen::Vector2d origin{origin_node[0].as<double>(), origin_node[1].as<double>()};

        const double occupied_thresh =
            node["occupied_thresh"] ? node["occupied_thresh"].as<double>() : 0.65;
        const double free_thresh =
            node["free_thresh"] ? node["free_thresh"].as<double>() : 0.25;
        const bool negate = node["negate"] ? (node["negate"].as<int>() != 0) : false;

        const fs::path image_rel_path{image_rel};
        const fs::path image_path =
            image_rel_path.is_absolute() ? image_rel_path : yaml_path.parent_path() / image_rel_path;

        const std::string image_bytes = read_file_binary(image_path, "map image");
        const Pgm pgm = parse_pgm(image_bytes, image_path.string());

        // y 轴翻转:图像第一行(img_row=0)在顶部,对应 world 最大 y(grid 最高 row)。
        const int w = pgm.width;
        const int h = pgm.height;
        std::vector<std::int8_t> data(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
        for (int img_row = 0; img_row < h; ++img_row)
        {
            const int grid_y = h - 1 - img_row;
            for (int col = 0; col < w; ++col)
            {
                const int pixel = pgm.pixels[static_cast<std::size_t>(img_row) *
                                                 static_cast<std::size_t>(w) +
                                             static_cast<std::size_t>(col)];
                data[static_cast<std::size_t>(grid_y) * static_cast<std::size_t>(w) +
                     static_cast<std::size_t>(col)] =
                    classify(pixel, pgm.maxval, negate, occupied_thresh, free_thresh);
            }
        }

        return OccupancyGrid{w, h, resolution, origin, std::move(data)};
    }
}
