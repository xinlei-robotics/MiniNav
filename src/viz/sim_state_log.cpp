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

    // ---------------------------------------------------------------------------
    // V0
    // ---------------------------------------------------------------------------
    void register_v0_statics(RerunSink& sink,
                             const std::string_view entity_root)
    {
        sink.log_axes_static("/world/origin", 1.0F);

        const std::string body_path = std::string{entity_root} + "/body";
        sink.log_axes_static(body_path, 0.5F);
    }

    void reset_trails_v0(RerunSink& sink, const std::string_view entity_root)
    {
        sink.clear_trail(trail_path_for(entity_root));
    }

    void log_to_rerun(RerunSink& sink,
                      const SimStateV0& state,
                      const std::string_view entity_root)
    {
        // 实体树设计 (V0):
        //   /world/origin            世界坐标轴 (static, 由 register_v0_statics)
        //   {entity_root}            机器人 (Transform3D, 每帧)
        //   {entity_root}/body       本体轴 (static, 跟随父级 Transform3D)
        //   {entity_root}/cmd/{v,w}  控制量时序
        //   /world/trails/{name}     轨迹 (世界系, 不继承 Transform3D)
        sink.log_pose(entity_root, state.pose);
        sink.log_twist(std::string{entity_root} + "/cmd", state.cmd);

        sink.log_trail_point(trail_path_for(entity_root),
                             state.pose.x(), state.pose.y());
    }

    // ---------------------------------------------------------------------------
    // V1
    // ---------------------------------------------------------------------------
    void register_v1_statics(RerunSink& sink,
                             const std::string_view entity_root)
    {
        sink.log_axes_static("/world/origin", 1.0F);

        const std::string root{entity_root};
        sink.log_axes_static(root + "/truth/body", 0.5F);
        sink.log_axes_static(root + "/odom/body", 0.5F);
    }

    void reset_trails_v1(RerunSink& sink, const std::string_view entity_root)
    {
        const std::string root{entity_root};
        sink.clear_trail(trail_path_for(root + "/truth"));
        sink.clear_trail(trail_path_for(root + "/odom"));
    }

    void log_to_rerun(RerunSink& sink,
                      const SimStateV1& state,
                      const std::string_view entity_root)
    {
        // 实体树设计 (V1):
        //   /world/origin                          世界坐标轴 (static)
        //   {entity_root}/truth                    真值 pose (Transform3D, 每帧)
        //   {entity_root}/truth/body               真值本体轴 (static)
        //   {entity_root}/odom                     估计 pose (Transform3D, 每帧)
        //   {entity_root}/odom/body                估计本体轴 (static)
        //   {entity_root}/cmd/{v,w}                原始命令时序
        //   {entity_root}/true_velocity/{v,w}      actuator 后真实速度时序
        //   {entity_root}/encoder/dticks/{l,r}     编码器增量时序
        //   {entity_root}/error/{position,yaw}     漂移诊断时序
        //   /world/trails/truth                    真值轨迹 (世界系)
        //   /world/trails/odom                     估计轨迹 (世界系)

        const std::string root{entity_root};

        // ---- 真值通道 ----
        const std::string truth_path = root + "/truth";
        sink.log_pose(truth_path, state.truth_pose);
        sink.log_trail_point(trail_path_for(truth_path),
                             state.truth_pose.x(), state.truth_pose.y());

        // ---- 估计通道 ----
        const std::string odom_path = root + "/odom";
        sink.log_pose(odom_path, state.odom_pose);
        sink.log_trail_point(trail_path_for(odom_path),
                             state.odom_pose.x(), state.odom_pose.y());

        // ---- 控制量 / 真实速度 ----
        sink.log_twist(root + "/cmd", state.cmd);
        sink.log_twist(root + "/true_velocity", state.true_velocity);

        // ---- 编码器读数 ----
        sink.log_scalar(root + "/encoder/dticks/l",
                        static_cast<double>(state.enc_dticks.left));
        sink.log_scalar(root + "/encoder/dticks/r",
                        static_cast<double>(state.enc_dticks.right));

        // ---- 漂移诊断: 让发散过程"自我量化" ----
        const double dx = state.odom_pose.x() - state.truth_pose.x();
        const double dy = state.odom_pose.y() - state.truth_pose.y();
        const double pos_err = std::hypot(dx, dy);

        const double raw_yaw_err = state.odom_pose.yaw() - state.truth_pose.yaw();
        const double yaw_err = wrap_angle(raw_yaw_err);

        sink.log_scalar(root + "/error/position", pos_err);
        sink.log_scalar(root + "/error/yaw", yaw_err);
    }
}