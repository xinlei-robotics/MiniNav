import mininav.control.goal_checker;
import mininav.control.progress_checker;
import mininav.control.controller;
import mininav.core.types;

#include <gtest/gtest.h>

#include <stdexcept>

namespace {

using mininav::Pose2D;
using mininav::Twist2D;
using mininav::control::GoalCheckerConfig;
using mininav::control::ProgressCheckerConfig;
using mininav::control::SimpleGoalChecker;
using mininav::control::SimpleProgressChecker;

const Pose2D kGoal{2.0, 1.0, 1.0};

}  // namespace

// ---------------------------------------------------------------------------
// SimpleGoalChecker
// ---------------------------------------------------------------------------

TEST(GoalChecker, PositionOnlyWhenYawIsNotRequested) {
  SimpleGoalChecker checker{GoalCheckerConfig{}, false};
  EXPECT_FALSE(checker.is_goal_reached(Pose2D{1.9, 1.0, 1.0}, kGoal, Twist2D{}));  // 0.10 m
  EXPECT_TRUE(checker.is_goal_reached(Pose2D{1.96, 1.0, -2.0}, kGoal, Twist2D{})); // 0.04 m,朝向不管
}

TEST(GoalChecker, YawCheckedWhenRequested) {
  SimpleGoalChecker checker{GoalCheckerConfig{}, true};
  EXPECT_FALSE(checker.is_goal_reached(Pose2D{2.0, 1.0, 0.8}, kGoal, Twist2D{}));  // 差 0.2 rad
  EXPECT_TRUE(checker.is_goal_reached(Pose2D{2.0, 1.0, 0.95}, kGoal, Twist2D{})); // 差 0.05 rad
}

TEST(GoalChecker, YawErrorIsWrapped) {
  SimpleGoalChecker checker{GoalCheckerConfig{}, true};
  const Pose2D goal{0.0, 0.0, 3.1};
  EXPECT_TRUE(checker.is_goal_reached(Pose2D{0.0, 0.0, -3.1}, goal, Twist2D{}));  // 实际只差 ≈ 0.083 rad
}

// 位置到达后锁存:原地转向时的微小漂移不会让判定倒退;reset 解除锁存。
TEST(GoalChecker, PositionIsLatchedUntilReset) {
  SimpleGoalChecker checker{GoalCheckerConfig{}, true};
  EXPECT_FALSE(checker.is_goal_reached(Pose2D{1.96, 1.0, 0.5}, kGoal, Twist2D{}));  // 位置到、朝向差
  EXPECT_TRUE(checker.is_goal_reached(Pose2D{1.94, 1.0, 0.95}, kGoal, Twist2D{}));  // 漂出 0.06 m,仍算到
  checker.reset();
  EXPECT_FALSE(checker.is_goal_reached(Pose2D{1.94, 1.0, 0.95}, kGoal, Twist2D{}));
}

TEST(GoalChecker, ReportsTolerances) {
  const SimpleGoalChecker checker{GoalCheckerConfig{.xy_tolerance = 0.1, .yaw_tolerance = 0.2}, true};
  const mininav::control::GoalTolerance tol = checker.tolerances();
  EXPECT_EQ(tol.xy, 0.1);
  EXPECT_EQ(tol.yaw, 0.2);
  EXPECT_TRUE(tol.check_yaw);
}

TEST(GoalChecker, RejectsNonPositiveTolerance) {
  EXPECT_THROW((SimpleGoalChecker{GoalCheckerConfig{.xy_tolerance = 0.0, .yaw_tolerance = 0.1}, false}),
               std::invalid_argument);
}

// ---------------------------------------------------------------------------
// SimpleProgressChecker
// ---------------------------------------------------------------------------

TEST(ProgressChecker, StuckAfterTimeAllowanceWithoutMovement) {
  SimpleProgressChecker checker{ProgressCheckerConfig{}};
  const Pose2D here{0.0, 0.0, 0.0};
  EXPECT_TRUE(checker.check(here, 0.0));  // 第一次调用只设基准
  EXPECT_TRUE(checker.check(Pose2D{0.15, 0.0, 1.0}, 5.0));
  EXPECT_TRUE(checker.check(here, 10.0));
  EXPECT_FALSE(checker.check(here, 10.01));
}

TEST(ProgressChecker, MovementBeyondRequiredDistanceResetsBaseline) {
  SimpleProgressChecker checker{ProgressCheckerConfig{}};
  EXPECT_TRUE(checker.check(Pose2D{0.0, 0.0, 0.0}, 0.0));
  EXPECT_TRUE(checker.check(Pose2D{0.25, 0.0, 0.0}, 8.0));   // 离开 0.25 m > 0.2:新基准 t = 8
  EXPECT_TRUE(checker.check(Pose2D{0.25, 0.0, 0.0}, 17.0));
  EXPECT_FALSE(checker.check(Pose2D{0.25, 0.0, 0.0}, 18.5));
}

TEST(ProgressChecker, ResetStartsANewWindow) {
  SimpleProgressChecker checker{ProgressCheckerConfig{}};
  EXPECT_TRUE(checker.check(Pose2D{}, 0.0));
  EXPECT_FALSE(checker.check(Pose2D{}, 11.0));
  checker.reset();
  EXPECT_TRUE(checker.check(Pose2D{}, 11.0));
  EXPECT_TRUE(checker.check(Pose2D{}, 20.0));
}

TEST(ProgressChecker, RejectsNonPositiveParameters) {
  EXPECT_THROW((SimpleProgressChecker{ProgressCheckerConfig{.required_movement = 0.0, .time_allowance = 10.0}}),
               std::invalid_argument);
}
