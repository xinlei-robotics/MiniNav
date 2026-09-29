import mininav.apps.sim;
import mininav.core.logger;

#include <CLI/CLI.hpp>

#include <exception>
#include <iostream>
#include <string>

// ===========================================================================
// sim 入口:CLI11 子命令解析与分派。仿真 / 规划逻辑在各模式的实现单元里。
//
//   sim ekf  [...]   V2 定位仿真(traj.csv)
//   sim plan [...]   V3 一次性全局规划(path.csv)
//
// 不带子命令时打印帮助。
// ===========================================================================

namespace
{
    using namespace mininav::apps;

    void add_output_options(CLI::App& cmd, OutputOptions& output, const std::string& default_csv)
    {
        cmd.add_option("--out", output.out_path,
                       "Output CSV path (default: data/" + default_csv + ").");
        auto* rrd_opt = cmd.add_option("--rrd", output.rrd_path,
                                       "Save the Rerun recording to the given .rrd path "
                                       "instead of spawning a Viewer.");
        auto* noviz_opt = cmd.add_flag("--no-viz", output.disable_viz,
                                       "Disable Rerun output entirely; CSV-only run (for CI / regression).");
        rrd_opt->excludes(noviz_opt);
    }

    CLI::App* add_ekf_command(CLI::App& app, EkfOptions& opts)
    {
        CLI::App* cmd = app.add_subcommand(
            "ekf",
            "Localization simulation: actuator + encoder + IMU noise, wheel-odometry "
            "baseline, and a 6D EKF fusing encoder + gyro with online bias estimation.");

        cmd->add_option("--seed", opts.seed,
                        "Master RNG seed; if omitted, seeded from std::random_device.");

        cmd->add_option("--preset", opts.preset_name, "Noise preset.")
           ->capture_default_str()
           ->check(CLI::IsMember({"low-noise", "default", "high-noise"}));

        cmd->add_option("--integrator", opts.integrator_name,
                        "EKF process-model integrator. rk4 = production; euler is kept only "
                        "for the RK4-vs-Euler attribution experiment.")
           ->capture_default_str()
           ->check(CLI::IsMember({"euler", "rk4"}));

        cmd->add_option("--q-scale", opts.q_scale,
                        "Multiplier on the EKF process noise Q (sensitivity analysis). "
                        "Default 1.0 keeps the physics-derived value; >1 trusts the motion "
                        "model less, <1 trusts it more. Does NOT touch the simulated truth.")
           ->capture_default_str()
           ->check(CLI::PositiveNumber);

        cmd->add_option("--r-scale", opts.r_scale,
                        "Multiplier on the EKF measurement noise R (encoder + IMU). "
                        "Default 1.0 keeps the physics-derived value; >1 trusts the sensors "
                        "less. Does NOT touch the simulated measurements.")
           ->capture_default_str()
           ->check(CLI::PositiveNumber);

        cmd->add_flag("--no-bias", opts.no_bias,
                      "Disable online gyro-bias estimation (force q_bias_omega = 0, the "
                      "no-bias compatibility path). Produces the 'ekf (no bias)' baseline for "
                      "the three-way RMSE comparison against 'ekf_with_bias'.");

        cmd->add_option("--robot", opts.robot_path,
                        "Robot description (robot.yaml): wheel geometry, encoder resolution, "
                        "footprint, actuator limits.")
           ->capture_default_str();

        add_output_options(*cmd, opts.output, "traj.csv");
        return cmd;
    }

    CLI::App* add_plan_command(CLI::App& app, PlanOptions& opts)
    {
        CLI::App* cmd = app.add_subcommand(
            "plan",
            "One-shot, RNG-free global planning: load map -> inflate -> A* -> path.csv "
            "(+ Rerun plan view).");

        cmd->add_option("--map", opts.map_path, "Map description (map.yaml, ROS map_server style).")
           ->required();
        cmd->add_option("--goal", opts.goal_str, "Goal as \"x,y\" in world meters.")->required();
        cmd->add_option("--start", opts.start_str,
                        "Start as \"x,y\" in world meters (default: grid center).");
        cmd->add_option("--config", opts.config_path,
                        "Planner config (planner.yaml). The flags below override it.");
        cmd->add_option("--heuristic", opts.heuristic_name,
                        "A* heuristic override (manhattan requires --connectivity 4).")
           ->check(CLI::IsMember({"manhattan", "euclidean", "octile"}));
        // 4 / 8 在 run_plan 里校验:CLI::IsMember 对 int 会实例化 std::function<int(int)>,
        // 在 clang 18 + libstdc++ 13 下触发库内部的弃用警告(-Werror)。
        cmd->add_option("--connectivity", opts.connectivity, "Grid connectivity override (4 or 8).");
        cmd->add_option("--inflation-radius", opts.inflation_radius,
                        "Obstacle inflation radius override (meters).")
           ->check(CLI::NonNegativeNumber);

        add_output_options(*cmd, opts.output, "path.csv");
        return cmd;
    }
}

int main(int argc, char** argv)
{
    CLI::App app{"MiniNav simulation. Pick a mode with a subcommand; `sim <mode> --help` "
                 "lists its options."};
    app.require_subcommand(0, 1);

    EkfOptions ekf_opts{};
    ekf_opts.robot_path = std::string{PROJECT_ROOT_DIR} + "/config/robot.yaml";
    PlanOptions plan_opts{};

    const CLI::App* ekf_cmd = add_ekf_command(app, ekf_opts);
    const CLI::App* plan_cmd = add_plan_command(app, plan_opts);

    try
    {
        app.parse(argc, argv);
    }
    catch (const CLI::ParseError& e)
    {
        return app.exit(e); // --help → stdout/exit 0; 错误 → stderr/exit != 0
    }

    try
    {
        if (ekf_cmd->parsed())
        {
            run_ekf(ekf_opts);
        }
        else if (plan_cmd->parsed())
        {
            run_plan(plan_opts);
        }
        else
        {
            std::cout << app.help();
        }
    }
    catch (const std::exception& ex)
    {
        mininav::log::error(ex.what());
        return 1;
    }
    return 0;
}
