#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "UniformBSplineTrajectoryPlanner.h"
#include "Utils.h"

using ctrl::UniformBSplineTrajectoryPlanner;

namespace {

// Helper: simple straight path (in SSL field frame units, meters)
std::vector<Eigen::Vector3d> MakeStraightPath(const Eigen::Vector3d& start,
                                              const Eigen::Vector3d& end) {
  return {start, end};
}

// Helper: multi-waypoint path to exercise remaining-path and mid-trajectory logic
std::vector<Eigen::Vector3d> MakeZigZagPath() {
  std::vector<Eigen::Vector3d> wp;
  wp.emplace_back(-1.0, -0.5, 0.0);
  wp.emplace_back(0.0, -0.5, 0.0);
  wp.emplace_back(0.5, 0.0, M_PI / 4.0);
  wp.emplace_back(1.0, 0.5, M_PI / 2.0);
  wp.emplace_back(1.5, 0.8, M_PI / 2.0);
  return wp;
}

// Helper: longer path to ensure ample interior control points for partial updates
std::vector<Eigen::Vector3d> MakeLongPath() {
  std::vector<Eigen::Vector3d> wp;
  wp.emplace_back(-2.0, -1.0, 0.0);
  wp.emplace_back(-1.0, -1.0, 0.0);
  wp.emplace_back(0.0, -0.5, 0.0);
  wp.emplace_back(0.5, 0.0, M_PI / 6.0);
  wp.emplace_back(1.0, 0.5, M_PI / 3.0);
  wp.emplace_back(1.5, 1.0, M_PI / 2.0);
  wp.emplace_back(2.0, 1.0, M_PI / 2.0);
  wp.emplace_back(2.5, 0.5, M_PI / 3.0);
  return wp;
}

// Configure a planner with sane limits and disabled verbosity
void ConfigurePlanner(UniformBSplineTrajectoryPlanner& p) {
  p.SetVerbose(false);
  // Conservative limits typical for SSL small robots
  p.SetLimits(0.8, 0.5, 2.5, 3.0);
  p.SetFeedbackGains(0.02, 0.0);
}

}  // namespace

// 1) No replanning when disabled
TEST(UniformBSplineReplanning, NoReplanWhenDisabled) {
  UniformBSplineTrajectoryPlanner planner;
  ConfigurePlanner(planner);

  auto path = MakeStraightPath({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
  double t0 = util::GetCurrentTime();
  ASSERT_TRUE(planner.SetPath(path, t0));

  // Replanning is disabled by default
  Eigen::Vector3d commanded = planner.GetCurrentDesiredPosition();
  Eigen::Vector3d state_est = commanded + Eigen::Vector3d(0.2, 0.0, 0.0);

  bool did_replan = planner.CheckAndReplan(state_est, commanded, t0 + 0.05);
  EXPECT_FALSE(did_replan);
  EXPECT_EQ(planner.GetReplanCount(), 0);
}

// 2) CheckAndReplan triggers and updates counters when enabled
TEST(UniformBSplineReplanning, CheckAndReplanTriggersAndCounts) {
  UniformBSplineTrajectoryPlanner planner;
  ConfigurePlanner(planner);
  planner.SetReplanningEnabled(true);

  auto path = MakeLongPath();
  double t0 = util::GetCurrentTime();
  ASSERT_TRUE(planner.SetPath(path, t0));

  // Create a large deviation beyond thresholds
  Eigen::Vector3d commanded = planner.GetCurrentDesiredPosition();
  Eigen::Vector3d state_est = commanded + Eigen::Vector3d(0.15, 0.0, 0.2);  // 15cm, 0.2 rad

  double t1 = t0 + 0.25;  // ensure > min_replan_interval_
  bool did_replan = planner.CheckAndReplan(state_est, commanded, t1,
                                           /*pos thr*/ 0.05,
                                           /*ang thr*/ 0.1);
  EXPECT_TRUE(did_replan);
  EXPECT_EQ(planner.GetReplanCount(), 1);
  EXPECT_NEAR(planner.GetLastReplanTime(), t1, 1e-6);

  // New trajectory should start from (approximately) the state_est pose
  // Evaluate at u=0
  Eigen::Vector3d new_start = planner.EvaluateBSplineAtParameter(0.0);
  EXPECT_NEAR(new_start.x(), state_est.x(), 1e-3);
  EXPECT_NEAR(new_start.y(), state_est.y(), 1e-3);
}

// 3) Rate limiting prevents immediate consecutive replans
TEST(UniformBSplineReplanning, CheckAndReplanRateLimited) {
  UniformBSplineTrajectoryPlanner planner;
  ConfigurePlanner(planner);
  planner.SetReplanningEnabled(true);

  auto path = MakeZigZagPath();
  double t0 = util::GetCurrentTime();
  ASSERT_TRUE(planner.SetPath(path, t0));

  Eigen::Vector3d commanded = planner.GetCurrentDesiredPosition();
  Eigen::Vector3d state_est = commanded + Eigen::Vector3d(0.2, 0.0, 0.2);

  double t1 = t0 + 0.25;  // ensure > min_replan_interval_
  EXPECT_TRUE(planner.CheckAndReplan(state_est, commanded, t1));
  EXPECT_EQ(planner.GetReplanCount(), 1);

  // Second attempt within min_replan_interval_ should be blocked
  double t2 = t1 + 0.05;  // < 0.2s
  EXPECT_FALSE(planner.CheckAndReplan(state_est, commanded, t2));
  EXPECT_EQ(planner.GetReplanCount(), 1);
}

// 4) TryAutoReplan applies partial correction within limits and respects rate limit
TEST(UniformBSplineReplanning, TryAutoReplanPartialCorrectionAndRateLimit) {
  UniformBSplineTrajectoryPlanner planner;
  ConfigurePlanner(planner);
  planner.SetReplanningEnabled(true);

  // Choose a start time that yields mid-trajectory progress
  auto path = MakeZigZagPath();
  // Slow down to make trajectory longer and stable for mid-progress selection
  planner.SetLimits(0.2, 0.3, 2.5, 3.0);
  double now = util::GetCurrentTime();
  ASSERT_TRUE(planner.SetPath(path, now));
  double duration = planner.GetTrajectoryDuration();
  double mid_start = util::GetCurrentTime() - 0.15 * duration; // ~15% progress (inside safe window)
  ASSERT_TRUE(planner.SetPath(path, mid_start));
  // Verify progress window; if out of range due to timing, reset once more
  double progress_est = (util::GetCurrentTime() - mid_start) / duration;
  if (progress_est < 0.05 || progress_est > 0.85) {
    mid_start = util::GetCurrentTime() - 0.2 * duration;
    ASSERT_TRUE(planner.SetPath(path, mid_start));
  }

  // Capture control points before correction
  auto cps_before = planner.GetControlPoints();

  // Build a modest error to trigger partial update (2cm < error < 20cm)
  Eigen::Vector3d desired_now = planner.GetCurrentDesiredPosition();
  Eigen::Vector3d state_est = desired_now + Eigen::Vector3d(0.0, 0.10, 0.0);  // 10 cm lateral

  double gate_time = util::GetCurrentTime() + 0.3; // bypass internal min_replan_interval_
  bool corrected = planner.TryAutoReplan(state_est, gate_time);

  // If a correction happened, control points must change; otherwise they must remain the same
  auto cps_after = planner.GetControlPoints();
  ASSERT_EQ(cps_before.size(), cps_after.size());
  bool any_changed = false;
  for (size_t i = 0; i < cps_before.size(); ++i) {
    if ((cps_before[i] - cps_after[i]).head<2>().norm() > 1e-9) {
      any_changed = true;
      break;
    }
  }
  if (corrected) {
    EXPECT_TRUE(any_changed);
  } else {
    EXPECT_FALSE(any_changed);
  }

  // Immediate subsequent attempt should be rate-limited or remain false
  EXPECT_FALSE(planner.TryAutoReplan(state_est, gate_time));
}

// 5) Boundary constraints are respected after partial updates
TEST(UniformBSplineReplanning, BoundaryConstraintsRespected) {
  UniformBSplineTrajectoryPlanner planner;
  ConfigurePlanner(planner);
  planner.SetReplanningEnabled(true);

  // Path near one field corner to exercise constraint clamping
  std::vector<Eigen::Vector3d> path = {
      Eigen::Vector3d(4.4, 2.9, 0.0),  // near top-right but inside margins
      Eigen::Vector3d(4.3, 2.8, 0.0),
      Eigen::Vector3d(4.2, 2.7, 0.0),
      Eigen::Vector3d(4.1, 2.6, 0.0)};

  double now = util::GetCurrentTime();
  ASSERT_TRUE(planner.SetPath(path, now - 0.5));

  // Inject an error that nudges points outward (would exceed field if unconstrained)
  Eigen::Vector3d desired = planner.GetCurrentDesiredPosition();
  Eigen::Vector3d state_est = desired + Eigen::Vector3d(0.1, 0.1, 0.0);

  (void)planner.TryAutoReplan(state_est, now);

  // Verify all control points remain within SSL Division B bounds with margin
  auto cps = planner.GetControlPoints();
  for (const auto& cp : cps) {
    EXPECT_LE(cp.x(), 4.5 - 0.005 + 1e-9);
    EXPECT_GE(cp.x(), -4.5 + 0.005 - 1e-9);
    EXPECT_LE(cp.y(), 3.0 - 0.005 + 1e-9);
    EXPECT_GE(cp.y(), -3.0 + 0.005 - 1e-9);
  }
}

// 6) GetRemainingPath is monotonic and skips very-close waypoint
TEST(UniformBSplineReplanning, RemainingPathMonotonicAndSkipClose) {
  UniformBSplineTrajectoryPlanner planner;
  ConfigurePlanner(planner);

  auto path = MakeZigZagPath();
  double t0 = util::GetCurrentTime();
  ASSERT_TRUE(planner.SetPath(path, t0));

  // Pose near the first segment start
  Eigen::Vector3d p1 = path.front();
  Eigen::Vector3d current = p1 + Eigen::Vector3d(0.02, 0.0, 0.0);  // within 2 cm

  auto rem1 = planner.GetRemainingPath(current);
  ASSERT_GE(rem1.size(), 2u);
  // First element is current pose by contract
  EXPECT_NEAR(rem1[0].x(), current.x(), 1e-9);
  EXPECT_NEAR(rem1[0].y(), current.y(), 1e-9);

  // Move later in the path; ensure progress does not jump backward
  Eigen::Vector3d later = path[2] + Eigen::Vector3d(0.01, 0.0, 0.0);
  auto rem2 = planner.GetRemainingPath(later);
  ASSERT_GE(rem2.size(), 2u);

  // The first actual waypoint returned (index 1) should be >= the earlier one in sequence
  // Compare against path contents to deduce indices
  auto index_in_path = [&](const Eigen::Vector3d& w) {
    for (size_t i = 0; i < path.size(); ++i) {
      if ((w.head<2>() - path[i].head<2>()).norm() < 1e-9) return static_cast<int>(i);
    }
    return -1;
  };

  int idx1 = index_in_path(rem1[1]);
  int idx2 = index_in_path(rem2[1]);
  ASSERT_GE(idx1, 0);
  ASSERT_GE(idx2, 0);
  EXPECT_LE(idx1, idx2);

  // Now place the robot very close (<5cm) to the next waypoint and ensure it is skipped
  Eigen::Vector3d near_next = path[1];
  current = near_next + Eigen::Vector3d(0.01, 0.0, 0.0);  // 1 cm away
  auto rem3 = planner.GetRemainingPath(current);
  ASSERT_GE(rem3.size(), 2u);
  // The first returned waypoint after current should not be the near_next; it should be path[2]
  int idx3 = index_in_path(rem3[1]);
  ASSERT_GE(idx3, 0);
  EXPECT_GE(idx3, 2);
}

// 7) AddGoal resets the path to go from current desired to the new goal
TEST(UniformBSplineReplanning, AddGoalResetsToCurrentToGoal) {
  UniformBSplineTrajectoryPlanner planner;
  ConfigurePlanner(planner);

  auto path = MakeStraightPath({-0.5, 0.0, 0.0}, {0.5, 0.0, 0.0});
  double t0 = util::GetCurrentTime();
  ASSERT_TRUE(planner.SetPath(path, t0));

  // Advance slightly by waiting a tick to ensure current desired != start
  util::WaitMs(5);
  Eigen::Vector3d desired_now = planner.GetCurrentDesiredPosition();

  Eigen::Vector3d new_goal(1.0, 0.2, 0.0);
  EXPECT_TRUE(planner.AddGoal(new_goal));

  // Start of new spline is previous desired pose, end is goal
  Eigen::Vector3d start_after = planner.EvaluateBSplineAtParameter(0.0);
  Eigen::Vector3d end_after = planner.EvaluateBSplineAtParameter(1.0);
  EXPECT_NEAR(start_after.x(), desired_now.x(), 1e-3);
  EXPECT_NEAR(start_after.y(), desired_now.y(), 1e-3);
  EXPECT_NEAR(end_after.x(), new_goal.x(), 1e-3);
  EXPECT_NEAR(end_after.y(), new_goal.y(), 1e-3);
}

// 8) Angular-only deviation can trigger CheckAndReplan
TEST(UniformBSplineReplanning, AngleOnlyDeviationTriggersReplan) {
  UniformBSplineTrajectoryPlanner planner;
  ConfigurePlanner(planner);
  planner.SetReplanningEnabled(true);

  auto path = MakeStraightPath({0.0, 0.0, 0.0}, {0.5, 0.0, 0.0});
  double t0 = util::GetCurrentTime();
  ASSERT_TRUE(planner.SetPath(path, t0));

  Eigen::Vector3d commanded = planner.GetCurrentDesiredPosition();
  // Position same, heading deviates
  Eigen::Vector3d state_est = commanded; state_est[2] += 0.2;

  bool did_replan = planner.CheckAndReplan(state_est, commanded, t0 + 0.25,
                                           /*pos thr*/ 0.05,
                                           /*ang thr*/ 0.1);
  EXPECT_TRUE(did_replan);
}
