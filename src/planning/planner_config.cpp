module;

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

module mininav.planning.planner_config;

import mininav.planning.grid_types;

namespace mininav::planning
{
    namespace
    {
        [[nodiscard]] std::string heuristic_to_string(const Heuristic h)
        {
            switch (h)
            {
            case Heuristic::Manhattan:
                return "manhattan";
            case Heuristic::Euclidean:
                return "euclidean";
            case Heuristic::Octile:
                return "octile";
            }
            return "euclidean";
        }

        [[nodiscard]] Heuristic heuristic_from_string(const std::string& s)
        {
            if (s == "manhattan")
            {
                return Heuristic::Manhattan;
            }
            if (s == "euclidean")
            {
                return Heuristic::Euclidean;
            }
            if (s == "octile")
            {
                return Heuristic::Octile;
            }
            throw std::runtime_error("planner_config: unknown heuristic '" + s + "'");
        }

        [[nodiscard]] Connectivity connectivity_from_int(const int c)
        {
            switch (c)
            {
            case 4:
                return Connectivity::Four;
            case 8:
                return Connectivity::Eight;
            default:
                throw std::runtime_error(
                    "planner_config: connectivity must be 4 or 8, got " + std::to_string(c));
            }
        }

        [[nodiscard]] PlannerConfig from_node(const YAML::Node& node)
        {
            PlannerConfig cfg{};
            if (node["inflation_radius"])
            {
                cfg.inflation_radius = node["inflation_radius"].as<double>();
            }
            if (node["heuristic"])
            {
                cfg.heuristic = heuristic_from_string(node["heuristic"].as<std::string>());
            }
            if (node["connectivity"])
            {
                cfg.connectivity = connectivity_from_int(node["connectivity"].as<int>());
            }
            if (node["allow_unknown"])
            {
                cfg.allow_unknown = node["allow_unknown"].as<bool>();
            }
            if (node["cost_weight"])
            {
                cfg.cost_weight = node["cost_weight"].as<double>();
            }
            return cfg;
        }
    } // namespace

    PlannerConfig parse_planner_config(const std::string_view yaml_text)
    {
        const YAML::Node node = YAML::Load(std::string{yaml_text});
        return from_node(node);
    }

    PlannerConfig load_planner_config(const std::string& path)
    {
        std::ifstream in{path};
        if (!in)
        {
            throw std::runtime_error("planner_config: cannot open '" + path + "'");
        }
        std::ostringstream ss;
        ss << in.rdbuf();
        return parse_planner_config(ss.str());
    }

    std::string to_yaml(const PlannerConfig& cfg)
    {
        YAML::Node node;
        node["inflation_radius"] = cfg.inflation_radius;
        node["heuristic"] = heuristic_to_string(cfg.heuristic);
        node["connectivity"] = static_cast<int>(cfg.connectivity);
        node["allow_unknown"] = cfg.allow_unknown;
        node["cost_weight"] = cfg.cost_weight;

        YAML::Emitter out;
        out << node;
        return std::string{out.c_str()};
    }
}
