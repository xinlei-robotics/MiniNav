module;

#include <cmath>
#include <string>
#include <string_view>

module mininav.viz.sim_state_log;

import mininav.core.types;
import mininav.core.math;
import mininav.viz.rerun_sink;

namespace mininav
{
    namespace
    {
        // ---------------------------------------------------------------------
        // 从 pose 实体路径推出对应的 trail 路径.
        //   "/world/robot/truth" -> "/world/trails/truth"
        //   "/world/robot"       -> "/world/trails/robot"
        //
        // 约定: trail 必须放在 /world/trails/ 下, 该子树不应被任何
        // Transform3D 覆盖, 这样轨迹点在世界系中是稳定的, 不会被父级
        // Transform 反复变换出现"漂移"假象.
        // ---------------------------------------------------------------------
        std::string trail_path_for(const std::string_view pose_path)
        {
            const auto last_slash = pose_path.rfind('/');
            const auto name = (last_slash == std::string_view::npos)
                                  ? pose_path
                                  : pose_path.substr(last_slash + 1);
            return std::string{"/world/trails/"} + std::string{name};
        }
    }

    // EKF 均值向量分量下标(与 mininav.localization.ekf_state 的 StateIdx 约定一致:
    // [px, py, θ, v, ω, b_ω])。viz 只取前三项做 pose, 用字面量避免反向依赖 localization。
    constexpr int kEkfPx = 0;
    constexpr int kEkfPy = 1;
    constexpr int kEkfTheta = 2;

    void register_statics(RerunSink& sink,
                          const std::string_view entity_root)
    {
        sink.log_axes_static("/world/origin", 1.0F);

        const std::string root{entity_root};
        sink.log_axes_static(root + "/truth/body", 0.5F);
        sink.log_axes_static(root + "/odom/body", 0.5F);
        sink.log_axes_static(root + "/ekf/body", 0.5F);
    }

    void reset_trails(RerunSink& sink, const std::string_view entity_root)
    {
        const std::string root{entity_root};
        sink.clear_trail(trail_path_for(root + "/truth"));
        sink.clear_trail(trail_path_for(root + "/odom"));
        sink.clear_trail(trail_path_for(root + "/ekf"));
    }

    void log_to_rerun(RerunSink& sink,
                      const SimState& state,
                      const std::string_view entity_root)
    {
        // 实体树设计:
        //   /world/origin                          世界坐标轴 (static)
        //   {entity_root}/truth                    真值 pose (Transform3D, 每帧)
        //   {entity_root}/odom                     wheel-odometry 估计 pose (基线)
        //   {entity_root}/ekf                      EKF 估计 pose
        //   {entity_root}/{truth,odom,ekf}/body    各自本体轴 (static)
        //   {entity_root}/cmd/{v,w}                原始命令时序
        //   {entity_root}/true_velocity/{v,w}      actuator 后真实速度时序
        //   {entity_root}/encoder/dticks/{l,r}     编码器增量时序
        //   {entity_root}/error/{position,yaw}     odom vs truth 漂移诊断时序
        //   /world/trails/{truth,odom,ekf}         三路轨迹 (世界系, 不继承 Transform3D)

        const std::string root{entity_root};

        // ---- 真值通道 ----
        const std::string truth_path = root + "/truth";
        sink.log_pose(truth_path, state.truth_pose);
        sink.log_trail_point(trail_path_for(truth_path),
                             state.truth_pose.x(), state.truth_pose.y());

        // ---- wheel-odometry 基线通道 ----
        const std::string odom_path = root + "/odom";
        sink.log_pose(odom_path, state.odom_pose);
        sink.log_trail_point(trail_path_for(odom_path),
                             state.odom_pose.x(), state.odom_pose.y());

        // ---- EKF 估计通道 ----
        const std::string ekf_path = root + "/ekf";
        const Pose2D ekf_pose{
            state.ekf_mean(kEkfPx), state.ekf_mean(kEkfPy), state.ekf_mean(kEkfTheta)
        };
        sink.log_pose(ekf_path, ekf_pose);
        sink.log_trail_point(trail_path_for(ekf_path), ekf_pose.x(), ekf_pose.y());

        // ---- 控制量 / 真实速度 ----
        sink.log_twist(root + "/cmd", state.cmd);
        sink.log_twist(root + "/true_velocity", state.true_velocity);

        // ---- 编码器读数 ----
        sink.log_scalar(root + "/encoder/dticks/l",
                        static_cast<double>(state.enc_dticks.left));
        sink.log_scalar(root + "/encoder/dticks/r",
                        static_cast<double>(state.enc_dticks.right));

        // ---- 漂移诊断 (odom vs truth): 让 wheel-odometry 的发散过程"自我量化" ----
        const double dx = state.odom_pose.x() - state.truth_pose.x();
        const double dy = state.odom_pose.y() - state.truth_pose.y();
        const double pos_err = std::hypot(dx, dy);

        const double yaw_err = wrap_angle(state.odom_pose.yaw() - state.truth_pose.yaw());

        sink.log_scalar(root + "/error/position", pos_err);
        sink.log_scalar(root + "/error/yaw", yaw_err);
    }
}