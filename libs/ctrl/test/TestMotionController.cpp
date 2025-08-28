#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "MotionController.h"
#include "SystemConfig.h"

class MotionControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {}
};

TEST_F(MotionControllerTest, TestDriveToPoint_AtGoal) {
  ctrl::MotionController controller;
  Eigen::Vector3d current_pose(0, 0, 0);
  Eigen::Vector3d target_pose(0, 0, 0);  // Same position

  auto [finished, velocity] = controller.DriveToPoint(current_pose, target_pose);

  EXPECT_TRUE(finished);
  EXPECT_NEAR(velocity.norm(), 0.0, 1e-6);
}

TEST_F(MotionControllerTest, TestDriveToPointNeedForwardMotion) {
  ctrl::MotionController controller;
  Eigen::Vector3d current_pose(0, 0, 0);
  Eigen::Vector3d target_pose(1, 0, 0);  // 1m forward

  auto [finished, velocity] = controller.DriveToPoint(current_pose, target_pose);

  EXPECT_FALSE(finished);
  EXPECT_GT(velocity[0], 0.0);          // Should have positive x velocity
  EXPECT_NEAR(velocity[1], 0.0, 1e-6);  // No sideways motion
}

TEST_F(MotionControllerTest, TestDriveToPointNeedRotation) {
  ctrl::MotionController controller;
  Eigen::Vector3d current_pose(0, 0, 0);
  Eigen::Vector3d target_pose(0, 0, M_PI / 2);  // 90 degree rotation

  auto [finished, velocity] = controller.DriveToPoint(current_pose, target_pose);

  EXPECT_FALSE(finished);
  EXPECT_GT(velocity[2], 0.0);  // Should have positive angular velocity
}

TEST_F(MotionControllerTest, TestDriveToPointVelocityClamping) {
  ctrl::MotionController controller;
  Eigen::Vector3d current_pose(0, 0, 0);
  Eigen::Vector3d target_pose(10, 10, 0);  // Far away target

  auto [finished, velocity] = controller.DriveToPoint(current_pose, target_pose);

  EXPECT_FALSE(finished);
  // Check velocity is clamped to max
  EXPECT_LE(std::abs(velocity[0]), cfg::SystemConfig::max_velocity_fBody_mps[0]);
  EXPECT_LE(std::abs(velocity[1]), cfg::SystemConfig::max_velocity_fBody_mps[1]);
  EXPECT_LE(std::abs(velocity[2]), cfg::SystemConfig::max_velocity_fBody_mps[2]);
}

TEST_F(MotionControllerTest, TestInterpolateToPointAtGoal) {
  ctrl::MotionController controller;
  Eigen::Vector3d current_pose(0, 0, 0);
  Eigen::Vector3d target_pose(0, 0, 0);

  auto [finished, velocity] = controller.InterpolateToPoint(current_pose, target_pose);

  EXPECT_TRUE(finished);
  EXPECT_NEAR(velocity.norm(), 0.0, 1e-6);
}

TEST_F(MotionControllerTest, TestInterpolateToPointForwardMotion) {
  ctrl::MotionController controller;
  Eigen::Vector3d current_pose(0, 0, 0);
  Eigen::Vector3d target_pose(2, 0, 0);

  auto [finished, velocity] = controller.InterpolateToPoint(current_pose, target_pose);

  EXPECT_FALSE(finished);
  EXPECT_NEAR(velocity[0], cfg::SystemConfig::max_velocity_fBody_mps[0], 1e-6);
  EXPECT_NEAR(velocity[1], 0.0, 1e-6);
}

TEST_F(MotionControllerTest, TestInterpolateToPointDiagonalMotion) {
  ctrl::MotionController controller;
  Eigen::Vector3d current_pose(0, 0, 0);
  Eigen::Vector3d target_pose(1, 1, 0);

  auto [finished, velocity] = controller.InterpolateToPoint(current_pose, target_pose);

  EXPECT_FALSE(finished);
  // Should move in normalized direction
  double expected_speed = cfg::SystemConfig::max_velocity_fBody_mps[0];
  EXPECT_NEAR(velocity.head<2>().norm(), expected_speed, 1e-6);
}

TEST_F(MotionControllerTest, TestInterpolateToPointWithinPositionTolerance) {
  ctrl::MotionController controller;
  Eigen::Vector3d current_pose(0, 0, 0);
  Eigen::Vector3d target_pose(0.005, 0, 0);  // Within 5cm tolerance

  auto [finished, velocity] = controller.InterpolateToPoint(current_pose, target_pose);

  EXPECT_TRUE(finished);
  EXPECT_NEAR(velocity.head<2>().norm(), 0.0, 1e-6);
}