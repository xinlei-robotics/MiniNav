// viz_sink_log_tests.cpp
//
// 用 gmock 把 Rerun 后端从 viz 逻辑里隔离掉:注入一个 MockVizSink,验证
// sim_state_log 的三个自由函数对 VizSink 接口的"调用契约"(次数 + 实体路径 +
// 派生量),而不启动任何 Rerun Viewer。
//
// 工程主张:能 mock 就证明可视化逻辑与 Rerun 后端被 VizSink 接口隔离干净
// (依赖倒置)。这些测试同时锁住实体树布局与 odom 漂移诊断的计算。

import mininav.core.types;
import mininav.viz.sink;
import mininav.viz.sim_state_log;

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <string_view>

namespace {

using ::testing::_;
using ::testing::DoubleEq;
using ::testing::Eq;
using ::testing::FloatEq;
using ::testing::NiceMock;

using mininav::Pose2D;
using mininav::SimState;
using mininav::VizSink;

// gmock 实现:每个纯虚方法记录调用,用于断言上层的调用契约。
class MockVizSink : public VizSink {
public:
    MOCK_METHOD(void, set_time, (double), (override));
    MOCK_METHOD(void, log_pose, (std::string_view, const Pose2D&), (override));
    MOCK_METHOD(void, log_twist, (std::string_view, const mininav::Twist2D&),
                (override));
    MOCK_METHOD(void, log_scalar, (std::string_view, double), (override));
    MOCK_METHOD(void, log_axes, (std::string_view, float), (override));
    MOCK_METHOD(void, log_axes_static, (std::string_view, float), (override));
    MOCK_METHOD(void, log_trail_point, (std::string_view, double, double),
                (override));
    MOCK_METHOD(void, clear_trail, (std::string_view), (override));
};

// 把字符串字面量显式包成 string_view 匹配器,匹配 string_view 形参。
auto path(const std::string_view sv) { return Eq(sv); }

// register_statics 应注册 1 个世界轴 + 三路本体轴,共 4 次 static axes。
TEST(VizSinkLog, RegisterStaticsLogsWorldAndThreeBodyAxes) {
    MockVizSink mock;

    EXPECT_CALL(mock, log_axes_static(path("/world/origin"), FloatEq(1.0F)));
    EXPECT_CALL(mock, log_axes_static(path("/world/robot/truth/body"), FloatEq(0.5F)));
    EXPECT_CALL(mock, log_axes_static(path("/world/robot/odom/body"), FloatEq(0.5F)));
    EXPECT_CALL(mock, log_axes_static(path("/world/robot/ekf/body"), FloatEq(0.5F)));

    register_statics(mock, "/world/robot");
}

// reset_trails 应清空三路轨迹,且轨迹路径被 remap 到 /world/trails/ 下。
TEST(VizSinkLog, ResetTrailsClearsThreeWorldFrameTrails) {
    MockVizSink mock;

    EXPECT_CALL(mock, clear_trail(path("/world/trails/truth")));
    EXPECT_CALL(mock, clear_trail(path("/world/trails/odom")));
    EXPECT_CALL(mock, clear_trail(path("/world/trails/ekf")));

    reset_trails(mock, "/world/robot");
}

// log_to_rerun 每帧应把三路 pose、三路 trail、两路 twist、四路 scalar 全部下沉。
TEST(VizSinkLog, LogToRerunRoutesEveryChannelOncePerFrame) {
    NiceMock<MockVizSink> mock;
    const SimState state{};

    EXPECT_CALL(mock, log_pose(path("/world/robot/truth"), _));
    EXPECT_CALL(mock, log_pose(path("/world/robot/odom"), _));
    EXPECT_CALL(mock, log_pose(path("/world/robot/ekf"), _));

    EXPECT_CALL(mock, log_trail_point(_, _, _)).Times(3);

    EXPECT_CALL(mock, log_twist(path("/world/robot/cmd"), _));
    EXPECT_CALL(mock, log_twist(path("/world/robot/true_velocity"), _));

    EXPECT_CALL(mock, log_scalar(_, _)).Times(4);

    log_to_rerun(mock, state, "/world/robot");
}

// odom 漂移诊断(position / yaw error)应由 odom 与 truth 之差正确算出。
TEST(VizSinkLog, LogToRerunComputesOdomDriftDiagnostics) {
    NiceMock<MockVizSink> mock;

    SimState state{};
    state.truth_pose = Pose2D{0.0, 0.0, 0.0};
    state.odom_pose = Pose2D{3.0, 4.0, 0.5};  // 距 truth: 5.0;yaw 偏差 0.5
    state.enc_dticks.left = 7;
    state.enc_dticks.right = 9;

    EXPECT_CALL(mock, log_scalar(path("/world/robot/encoder/dticks/l"), DoubleEq(7.0)));
    EXPECT_CALL(mock, log_scalar(path("/world/robot/encoder/dticks/r"), DoubleEq(9.0)));
    EXPECT_CALL(mock, log_scalar(path("/world/robot/error/position"), DoubleEq(5.0)));
    EXPECT_CALL(mock, log_scalar(path("/world/robot/error/yaw"), DoubleEq(0.5)));

    log_to_rerun(mock, state, "/world/robot");
}

}  // namespace
