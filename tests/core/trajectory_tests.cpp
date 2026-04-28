import mininav.core.trajectory;
import mininav.core.types;

#include <gtest/gtest.h>

namespace
{
    constexpr double kEps = 1e-12;
} // namespace

TEST(Trajectory, DefaultIsEmpty)
{
    const mininav::Trajectory<mininav::SimStateV0> traj;
    EXPECT_TRUE(traj.empty());
    EXPECT_EQ(traj.size(), 0u);
}

TEST(Trajectory, AppendIncreasesSize)
{
    mininav::Trajectory<mininav::SimStateV0> traj;
    traj.append(mininav::SimStateV0{0.0, {}, {}});
    traj.append(mininav::SimStateV0{0.1, {}, {}});
    traj.append(mininav::SimStateV0{0.2, {}, {}});
    EXPECT_EQ(traj.size(), 3u);
    EXPECT_FALSE(traj.empty());
}

TEST(Trajectory, RecordsPreservedInOrder)
{
    mininav::Trajectory<mininav::SimStateV0> traj;
    traj.append(mininav::SimStateV0{0.0, mininav::Pose2D{0.0, 0.0, 0.0}, {}});
    traj.append(mininav::SimStateV0{1.0, mininav::Pose2D{1.0, 2.0, 0.5}, {}});
    traj.append(mininav::SimStateV0{2.0, mininav::Pose2D{3.0, 4.0, 1.0}, {}});

    const auto& recs = traj.records();
    ASSERT_EQ(recs.size(), 3u);
    EXPECT_NEAR(recs[0].t, 0.0, kEps);
    EXPECT_NEAR(recs[1].pose.x(), 1.0, kEps);
    EXPECT_NEAR(recs[1].pose.y(), 2.0, kEps);
    EXPECT_NEAR(recs[2].pose.yaw(), 1.0, kEps);
}

TEST(Trajectory, ClearMakesEmpty)
{
    mininav::Trajectory<mininav::SimStateV0> traj;
    traj.append(mininav::SimStateV0{0.0, {}, {}});
    traj.append(mininav::SimStateV0{0.1, {}, {}});
    traj.clear();
    EXPECT_TRUE(traj.empty());
    EXPECT_EQ(traj.size(), 0u);
}

TEST(Trajectory, ReserveDoesNotChangeSize)
{
    mininav::Trajectory<mininav::SimStateV0> traj;
    traj.reserve(1000);
    EXPECT_TRUE(traj.empty());
    EXPECT_EQ(traj.size(), 0u);
}

TEST(Trajectory, MoveAppend)
{
    mininav::Trajectory<mininav::SimStateV0> traj;
    mininav::SimStateV0 r{1.0, mininav::Pose2D{1.0, 2.0, 0.0}, {}};
    traj.append(std::move(r));
    ASSERT_EQ(traj.size(), 1u);
    EXPECT_NEAR(traj.records()[0].pose.x(), 1.0, kEps);
}

// ---------------------------------------------------------------------------
// 验证 Trajectory<T> 真的是"模板":能用任意数据类型实例化
// ---------------------------------------------------------------------------
namespace
{
    struct DummyRecord
    {
        double t{0.0};
        int payload{0};
    };
} // namespace

TEST(Trajectory, WorksWithCustomRecordType)
{
    mininav::Trajectory<DummyRecord> traj;
    traj.append(DummyRecord{0.0, 42});
    traj.append(DummyRecord{1.0, 84});
    ASSERT_EQ(traj.size(), 2u);
    EXPECT_EQ(traj.records()[0].payload, 42);
    EXPECT_EQ(traj.records()[1].payload, 84);
}
