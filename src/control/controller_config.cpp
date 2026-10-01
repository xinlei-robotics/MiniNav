module;

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <variant>

module mininav.control.controller_config;

import mininav.control.pure_pursuit;
import mininav.control.velocity_smoother;
import mininav.control.goal_checker;
import mininav.control.progress_checker;

namespace mininav::control
{
    namespace
    {
        // 一个 YAML key 与它写入的字段。
        struct Field
        {
            std::string_view key;
            std::variant<double*, bool*> target;
        };

        [[noreturn]] void fail(const std::string_view section, const std::string& message)
        {
            throw std::runtime_error{std::string{section} + "_config: " + message};
        }

        // 严格解析一个平铺映射:未知 key 报错,缺失 key 保留默认值。
        void parse_fields(const std::string_view section, const std::string_view yaml_text,
                          std::initializer_list<Field> fields)
        {
            const YAML::Node node = YAML::Load(std::string{yaml_text});
            if (!node.IsDefined() || node.IsNull())
            {
                return; // 空段:全部默认
            }
            if (!node.IsMap())
            {
                fail(section, "expected a YAML mapping");
            }

            for (const auto& entry : node)
            {
                const std::string key = entry.first.as<std::string>();
                const auto it = std::ranges::find(fields, key, &Field::key);
                if (it == fields.end())
                {
                    std::string expected;
                    for (const Field& f : fields)
                    {
                        expected += (expected.empty() ? "" : ", ") + std::string{f.key};
                    }
                    fail(section, "unknown key '" + key + "' (expected " + expected + ")");
                }
                try
                {
                    std::visit([&entry](auto* target)
                    {
                        *target = entry.second.as<std::remove_pointer_t<decltype(target)>>();
                    }, it->target);
                }
                catch (const YAML::Exception&)
                {
                    fail(section, "'" + key + "' has an invalid value '" + YAML::Dump(entry.second) + "'");
                }
            }
        }

        // validate(...) 抛 invalid_argument;对配置文件的使用者统一成 runtime_error。
        template <typename Config>
        void validate_section(const std::string_view section, const Config& cfg)
        {
            try
            {
                validate(cfg);
            }
            catch (const std::invalid_argument& e)
            {
                fail(section, e.what());
            }
        }
    }

    ControllerConfig parse_controller_config(const std::string_view yaml_text)
    {
        ControllerConfig cfg{};
        PurePursuitConfig& pp = cfg.pursuit;
        parse_fields("controller", yaml_text, {
                         {"control_frequency", &cfg.control_frequency},
                         {"desired_linear_vel", &pp.desired_linear_vel},
                         {"use_velocity_scaled_lookahead", &pp.use_velocity_scaled_lookahead},
                         {"lookahead_time", &pp.lookahead_time},
                         {"min_lookahead", &pp.min_lookahead},
                         {"max_lookahead", &pp.max_lookahead},
                         {"lookahead_dist", &pp.lookahead_dist},
                         {"use_curvature_regulation", &pp.use_curvature_regulation},
                         {"curvature_lookahead_dist", &pp.curvature_lookahead_dist},
                         {"regulated_min_radius", &pp.regulated_min_radius},
                         {"approach_dist", &pp.approach_dist},
                         {"min_approach_vel", &pp.min_approach_vel},
                         {"use_rotate_to_heading", &pp.use_rotate_to_heading},
                         {"rotate_to_heading_angle", &pp.rotate_to_heading_angle},
                         {"rotate_vel", &pp.rotate_vel},
                         {"max_projection_search_dist", &pp.max_projection_search_dist},
                         {"max_accel", &cfg.smoother.max_accel},
                         {"max_angular_accel", &cfg.smoother.max_angular_accel},
                     });

        if (!(cfg.control_frequency > 0.0))
        {
            fail("controller", "control_frequency must be positive");
        }
        validate_section("controller", cfg.pursuit);
        validate_section("controller", cfg.smoother);
        return cfg;
    }

    GoalCheckerConfig parse_goal_checker_config(const std::string_view yaml_text)
    {
        GoalCheckerConfig cfg{};
        parse_fields("goal_checker", yaml_text, {
                         {"xy_tolerance", &cfg.xy_tolerance},
                         {"yaw_tolerance", &cfg.yaw_tolerance},
                     });
        validate_section("goal_checker", cfg);
        return cfg;
    }

    ProgressCheckerConfig parse_progress_checker_config(const std::string_view yaml_text)
    {
        ProgressCheckerConfig cfg{};
        parse_fields("progress_checker", yaml_text, {
                         {"required_movement", &cfg.required_movement},
                         {"time_allowance", &cfg.time_allowance},
                     });
        validate_section("progress_checker", cfg);
        return cfg;
    }
}
