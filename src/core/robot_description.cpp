module;

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <initializer_list>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

module mininav.core.robot_description;

import mininav.core.math;

namespace mininav
{
    namespace
    {
        [[noreturn]] void fail(const std::string& message)
        {
            throw std::runtime_error("robot_description: " + message);
        }

        // 映射节点只允许 known 里的 key;ctx 是报错时的字段路径前缀。
        void require_map_with_keys(const YAML::Node& node, const std::string& ctx,
                                   std::initializer_list<std::string_view> known)
        {
            if (!node.IsDefined() || !node.IsMap())
            {
                fail("expected a mapping" + (ctx.empty() ? std::string{} : " for '" + ctx + "'"));
            }
            for (const auto& entry : node)
            {
                const std::string key = entry.first.as<std::string>();
                if (std::ranges::find(known, key) == known.end())
                {
                    std::string expected;
                    for (const std::string_view k : known)
                    {
                        expected += (expected.empty() ? "" : ", ") + std::string{k};
                    }
                    fail("unknown key '" + ctx + key + "' (expected " + expected + ")");
                }
            }
        }

        template <typename T>
        [[nodiscard]] T read(const YAML::Node& parent, const std::string& ctx, const char* key)
        {
            const YAML::Node node = parent[key];
            if (!node)
            {
                fail("missing required key '" + ctx + key + "'");
            }
            try
            {
                return node.as<T>();
            }
            catch (const YAML::Exception&)
            {
                fail("'" + ctx + key + "' has an invalid value '" + YAML::Dump(node) + "'");
            }
        }

        // `!(v > 0)` 同时拒绝 NaN。
        void require_positive(const double v, const char* key)
        {
            if (!(v > 0.0) || !std::isfinite(v))
            {
                fail(std::string{"'"} + key + "' must be positive and finite, got " + std::to_string(v));
            }
        }
    }

    double Footprint::circumscribed_radius() const noexcept
    {
        return std::hypot(0.5 * length, 0.5 * width);
    }

    double RobotDescription::distance_per_tick() const noexcept
    {
        return 2.0 * kPi * wheel_radius / static_cast<double>(ticks_per_rev);
    }

    RobotDescription parse_robot_description(const std::string_view yaml_text)
    {
        const YAML::Node root = YAML::Load(std::string{yaml_text});
        require_map_with_keys(root, "",
                              {"wheel_radius", "wheel_base", "ticks_per_rev", "footprint", "limits",
                               "actuator_time_constant"});

        RobotDescription robot{};
        robot.wheel_radius = read<double>(root, "", "wheel_radius");
        robot.wheel_base = read<double>(root, "", "wheel_base");
        robot.ticks_per_rev = read<std::int64_t>(root, "", "ticks_per_rev");
        robot.actuator_time_constant = read<double>(root, "", "actuator_time_constant");

        const YAML::Node footprint = root["footprint"];
        require_map_with_keys(footprint, "footprint.", {"length", "width"});
        robot.footprint.length = read<double>(footprint, "footprint.", "length");
        robot.footprint.width = read<double>(footprint, "footprint.", "width");

        const YAML::Node limits = root["limits"];
        require_map_with_keys(limits, "limits.", {"max_linear_vel", "max_angular_vel"});
        robot.limits.max_linear_vel = read<double>(limits, "limits.", "max_linear_vel");
        robot.limits.max_angular_vel = read<double>(limits, "limits.", "max_angular_vel");

        require_positive(robot.wheel_radius, "wheel_radius");
        require_positive(robot.wheel_base, "wheel_base");
        if (robot.ticks_per_rev <= 0)
        {
            fail("'ticks_per_rev' must be positive, got " + std::to_string(robot.ticks_per_rev));
        }
        require_positive(robot.footprint.length, "footprint.length");
        require_positive(robot.footprint.width, "footprint.width");
        require_positive(robot.limits.max_linear_vel, "limits.max_linear_vel");
        require_positive(robot.limits.max_angular_vel, "limits.max_angular_vel");
        if (!(robot.actuator_time_constant >= 0.0) || !std::isfinite(robot.actuator_time_constant))
        {
            fail("'actuator_time_constant' must be non-negative and finite, got " +
                 std::to_string(robot.actuator_time_constant));
        }
        return robot;
    }

    RobotDescription load_robot_description(const std::string& path)
    {
        std::ifstream in{path};
        if (!in)
        {
            fail("cannot open '" + path + "'");
        }
        std::ostringstream ss;
        ss << in.rdbuf();
        return parse_robot_description(ss.str());
    }
}
