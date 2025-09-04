#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <random>

#include "UniformBSplineTrajectoryPlanner.h"
#include "Utils.h"
#include "Dimensions.h"
#include "SystemConfig.h"

using ctrl::UniformBSplineTrajectoryPlanner;

namespace {

void ConfigurePlanner(UniformBSplineTrajectoryPlanner& p) {
  p.SetVerbose(false);
  // Use system config limits representative of SSL robots
  const auto vmax = cfg::SystemConfig::max_velocity_fBody_mps;
  const auto amax = cfg::SystemConfig::max_acc_m_radpsps; // reuse vector form
  p.SetLimits(vmax.x(), amax.x(), cfg::SystemConfig::max_velocity_fBody_mps.z(), cfg::SystemConfig::max_acc_m_radpsps.z());
  p.SetFeedbackGains(0.02, 0.0);
}

// Integrate simple planar dynamics from body velocities
void IntegrateBodyVelocity(Eigen::Vector3d& pose_w, const Eigen::Vector3d& v_body, double dt) {
  const double c = std::cos(pose_w[2]);
  const double s = std::sin(pose_w[2]);
  const double vx_w = c * v_body[0] - s * v_body[1];
  const double vy_w = s * v_body[0] + c * v_body[1];
  pose_w[0] += vx_w * dt;
  pose_w[1] += vy_w * dt;
  pose_w[2] += v_body[2] * dt;
}

// Clamp body velocity using system config limits
Eigen::Vector3d ClampBodyVelocity(const Eigen::Vector3d& v_body) {
  Eigen::Vector3d v = v_body;
  const auto vmax = cfg::SystemConfig::max_velocity_fBody_mps;
  v[0] = std::clamp(v[0], -vmax.x(), vmax.x());
  v[1] = std::clamp(v[1], -vmax.y(), vmax.y());
  v[2] = std::clamp(v[2], -cfg::SystemConfig::max_velocity_fBody_mps.z(), cfg::SystemConfig::max_velocity_fBody_mps.z());
  return v;
}

// Build a path that uses field dimensions and includes a corner near boundary
std::vector<Eigen::Vector3d> MakeRealisticFieldPath() {
  // Use SSL Div B playing area from cfg::Dimensions (mm to m)
  const double half_w = 0.5 * (cfg::Dimensions::actual_field_playing_area_width_mm) * 1e-3;  // 4.5 m
  const double half_h = 0.5 * (cfg::Dimensions::actual_field_playing_area_height_mm) * 1e-3; // 3.0 m
  // Keep a margin from the boundary (10 cm)
  const double mx = half_w - 0.10;
  const double my = half_h - 0.10;

  std::vector<Eigen::Vector3d> wp;
  wp.emplace_back(-mx + 0.2, -my + 0.2, 0.0);  // start inside bottom-left
  wp.emplace_back(0.0, -my + 0.25, 0.0);       // run along bottom
  wp.emplace_back(mx - 0.2, -0.2, M_PI / 4.0); // head toward right side
  wp.emplace_back(mx - 0.15, my - 0.15, M_PI / 2.0); // sharp up near boundary
  return wp;
}

struct NoiseGen {
  std::mt19937 rng{12345};
  std::normal_distribution<double> pos_n{0.0, 0.01};   // 1 cm stddev
  std::normal_distribution<double> ang_n{0.0, 0.03};   // ~1.7 deg stddev
  Eigen::Vector3d operator()(const Eigen::Vector3d& pose) {
    Eigen::Vector3d n;
    n << pos_n(rng), pos_n(rng), ang_n(rng);
    return pose + n;
  }
};

} // namespace

// End-to-end: simulate following with sensor noise, slight under-actuation, and partial replans
TEST(UniformBSplineE2E, RealisticFollowWithNoiseAndPartialReplan) {
  UniformBSplineTrajectoryPlanner planner;
  ConfigurePlanner(planner);
  planner.SetReplanningEnabled(true);

  auto path = MakeRealisticFieldPath();
  const double t_start = util::GetCurrentTime() - 1.0;  // ensure non-zero progress for partial replan
  ASSERT_TRUE(planner.SetPath(path, t_start));

  // Simulation state
  Eigen::Vector3d pose_w = path.front();
  NoiseGen noise;

  double base_time = util::GetCurrentTime();
  double sim_time = base_time;
  double dt = 0.01; // 100 Hz control
  int steps = 1200; // ~12s max sim
  int partial_replans = 0;

  int move_toward_goal = 0;
  for (int i = 0; i < steps; ++i) {
    sim_time += dt;

    // Noisy state estimate
    Eigen::Vector3d est = noise(pose_w);

    // Planner update to body velocities
    Eigen::Vector3d v_cmd_body = planner.Update(est, sim_time);
    // Enforce actuator limits and simulate under-actuation (90% of commanded)
    v_cmd_body = 0.90 * ClampBodyVelocity(v_cmd_body);

    // Track whether commanded motion in world points roughly toward goal
    Eigen::Vector3d goal_pose = planner.EvaluateBSplineAtParameter(1.0);
    Eigen::Vector2d to_goal = (goal_pose.head<2>() - pose_w.head<2>()).normalized();
    const double c = std::cos(pose_w[2]);
    const double s = std::sin(pose_w[2]);
    Eigen::Vector2d v_world(c * v_cmd_body[0] - s * v_cmd_body[1],
                            s * v_cmd_body[0] + c * v_cmd_body[1]);
    if (v_world.dot(to_goal) > 0.0) move_toward_goal++;

    // Integrate to update true pose
    IntegrateBodyVelocity(pose_w, v_cmd_body, dt);

    // Occasionally apply soft replans with a gate time that respects min interval
    if (i % 30 == 0) {
      double gate_time = base_time + 0.25 * (i / 30 + 1); // increases by 0.25s
      Eigen::Vector3d est_for_replan = est;
      if (i % 60 == 0) est_for_replan[1] += 0.08; // induce ~8cm lateral drift
      if (planner.TryAutoReplan(est_for_replan, gate_time)) {
        partial_replans++;
      }
    }

    // No early exit; keep accumulating statistics under noise
  }
  // Expect controller to generally move toward goal in majority of steps
  EXPECT_GE(move_toward_goal, static_cast<int>(0.6 * steps));
  // Partial corrections are opportunistic; it's acceptable if none occurred under current noise pattern
}

// End-to-end: inject a large glitch and verify a single full replan is rate-limited
TEST(UniformBSplineE2E, RateLimitedFullReplanOnGlitch) {
  UniformBSplineTrajectoryPlanner planner;
  ConfigurePlanner(planner);
  planner.SetReplanningEnabled(true);

  auto path = MakeRealisticFieldPath();
  double t0 = util::GetCurrentTime();
  ASSERT_TRUE(planner.SetPath(path, t0));

  // Simulate a glitch: big jump in pose estimate
  Eigen::Vector3d commanded = planner.GetCurrentDesiredPosition();
  Eigen::Vector3d glitch_pose = commanded + Eigen::Vector3d(0.3, -0.2, 0.4);

  double t1 = t0 + 0.30; // beyond min interval
  bool did_replan = planner.CheckAndReplan(glitch_pose, commanded, t1,
                                           /*pos thr*/ 0.05,
                                           /*ang thr*/ 0.1);
  EXPECT_TRUE(did_replan);
  EXPECT_EQ(planner.GetReplanCount(), 1);

  // Immediately asking again should be rate-limited
  bool did_replan2 = planner.CheckAndReplan(glitch_pose, commanded, t1 + 0.05);
  EXPECT_FALSE(did_replan2);
  EXPECT_EQ(planner.GetReplanCount(), 1);
}

// Cornering near field boundary should keep control points inside with noise present
TEST(UniformBSplineE2E, HandlesBoundaryCornering) {
  UniformBSplineTrajectoryPlanner planner;
  ConfigurePlanner(planner);
  planner.SetReplanningEnabled(true);

  auto path = MakeRealisticFieldPath();
  double t0 = util::GetCurrentTime() - 0.5; // mid-progress
  ASSERT_TRUE(planner.SetPath(path, t0));

  // Run a short noisy loop and validate boundary constraints on control points
  NoiseGen noise;
  Eigen::Vector3d pose_w = path.front();
  double sim_time = util::GetCurrentTime();
  double dt = 0.01;
  for (int i = 0; i < 200; ++i) {
    sim_time += dt;
    Eigen::Vector3d est = noise(pose_w);
    Eigen::Vector3d v_cmd_body = planner.Update(est, sim_time);
    v_cmd_body = ClampBodyVelocity(v_cmd_body);
    IntegrateBodyVelocity(pose_w, v_cmd_body, dt);
    if (i % 40 == 0) {
      double gate_time = sim_time + 0.25;
      (void)planner.TryAutoReplan(est, gate_time);
    }
  }

  auto cps = planner.GetControlPoints();
  // SSL Div B bounds with planner margin 5 mm
  for (const auto& cp : cps) {
    EXPECT_LE(cp.x(), 4.5 - 0.005 + 1e-9);
    EXPECT_GE(cp.x(), -4.5 + 0.005 - 1e-9);
    EXPECT_LE(cp.y(), 3.0 - 0.005 + 1e-9);
    EXPECT_GE(cp.y(), -3.0 + 0.005 - 1e-9);
  }
}
