#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <random>

#include "Utils.h"
#include "RobotModel.h"

// Verify world->body frame conversion matches util::RotateAboutZ with -theta
TEST(CoordinateFrames, WorldToBodyMatchesRotation) {
  std::vector<double> thetas = {0.0, M_PI/6, M_PI/4, M_PI/2, -M_PI/3, M_PI, -M_PI/2};
  for (double theta : thetas) {
    for (int i = 0; i < 10; ++i) {
      Eigen::Vector3d v_w;
      v_w << (i - 5) * 0.1, (5 - i) * 0.07, 0.0;
      // Planner formula in UniformBSplineTrajectoryPlanner::Update
      double body_vx = v_w[0] * std::cos(theta) + v_w[1] * std::sin(theta);
      double body_vy = -v_w[0] * std::sin(theta) + v_w[1] * std::cos(theta);
      Eigen::Vector3d v_b_planner(body_vx, body_vy, 0.0);

      // Reference using util
      Eigen::Vector3d v_b_ref = util::RotateAboutZ(v_w, -theta);

      EXPECT_NEAR(v_b_planner[0], v_b_ref[0], 1e-9);
      EXPECT_NEAR(v_b_planner[1], v_b_ref[1], 1e-9);
    }
  }
}

// Verify body->world inverse of above using util::RotateAboutZ
TEST(CoordinateFrames, BodyToWorldInverse) {
  std::vector<double> thetas = {0.0, M_PI/5, -M_PI/5, M_PI/2, -M_PI/2};
  for (double theta : thetas) {
    for (int i = 0; i < 10; ++i) {
      Eigen::Vector3d v_w;
      v_w << (i - 3) * 0.2, (i - 7) * -0.11, 0.0;
      Eigen::Vector3d v_b = util::RotateAboutZ(v_w, -theta);
      Eigen::Vector3d v_w_back = util::RotateAboutZ(v_b, theta);
      EXPECT_NEAR(v_w_back[0], v_w[0], 1e-9);
      EXPECT_NEAR(v_w_back[1], v_w[1], 1e-9);
    }
  }
}

// Verify body<->wheel round-trip through RobotModel mappings
TEST(WheelKinematics, BodyToWheelAndBack) {
  kin::RobotModel model;
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> lin(-0.6, 0.6);
  std::uniform_real_distribution<double> ang(-2.0, 2.0);

  for (int i = 0; i < 100; ++i) {
    Eigen::Vector3d v_b(lin(rng), lin(rng), ang(rng));
    Eigen::Vector4d w_rpm = model.RobotVelocityToWheelSpeedsRpm(v_b);
    Eigen::Vector3d v_b_rec = model.WheelSpeedsRpmToRobotVelocity(w_rpm);
    // Allow small numerical tolerance due to pseudoinverse
    EXPECT_NEAR(v_b_rec[0], v_b[0], 1e-6);
    EXPECT_NEAR(v_b_rec[1], v_b[1], 1e-6);
    EXPECT_NEAR(v_b_rec[2], v_b[2], 1e-6);
  }
}

// Consistency with estimator convention: world=RotateZ(body, +theta)
TEST(CoordinateFrames, EstimatorConventionConsistency) {
  double theta = 0.9; // rad
  Eigen::Vector3d v_b(0.2, -0.1, 0.5);
  // Body to world
  Eigen::Vector3d v_w = util::RotateAboutZ(v_b, theta);
  // World back to body must match -theta
  Eigen::Vector3d v_b2 = util::RotateAboutZ(v_w, -theta);
  EXPECT_NEAR(v_b2[0], v_b[0], 1e-9);
  EXPECT_NEAR(v_b2[1], v_b[1], 1e-9);
}

