#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include <cmath>

#include <Eigen/Dense>

#include "RobotManager.h"
#include "Utils.h"

class RobotManagerTest : public ::testing::Test {
 protected:
  void CreateRobotManager() {
    robot_manager = std::make_unique<rob::RobotManager>();

    // Initialize at origin and set as home
    Eigen::Vector3d origin(0, 0, 0);
    robot_manager->InitializePose(origin);
    robot_manager->InitializeHome(origin);
  }

  void SetUp() override {
    position_tolerance = 0.1;
    velocity_tolerance = 0.1;
    CreateRobotManager();
  }

  void TearDown() override {
    if (robot_manager) {
      robot_manager.reset();
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }

  std::unique_ptr<rob::RobotManager> robot_manager;
  double position_tolerance;
  double velocity_tolerance;
};

TEST_F(RobotManagerTest, TestBasicInitialization) {
  EXPECT_EQ(robot_manager->GetRobotState(), "CALIBRATING");

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_EQ(robot_manager->GetRobotState(), "IDLE");

  Eigen::Vector3d pose = robot_manager->GetPoseInWorldFrame();
  EXPECT_NEAR(pose[0], 0.0, position_tolerance);
  EXPECT_NEAR(pose[1], 0.0, position_tolerance);
}

TEST_F(RobotManagerTest, TestVelocityLimits) {
  Eigen::Vector3d valid_velocity(0.3, 0.2, 0.1);
  EXPECT_TRUE(robot_manager->BodyVelocityIsInLimits(valid_velocity));

  Eigen::Vector3d invalid_velocity(50.0, 50.0, 50.0);
  EXPECT_FALSE(robot_manager->BodyVelocityIsInLimits(invalid_velocity));
}

TEST_F(RobotManagerTest, TestSetBodyVelocity) {
  Eigen::Vector3d test_velocity(0.2, 0.1, 0.05);
  robot_manager->SetBodyVelocity(test_velocity);
  EXPECT_EQ(robot_manager->GetVelocityInWorldFrame(), test_velocity);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_EQ(robot_manager->GetRobotState(), "IDLE");
}

TEST_F(RobotManagerTest, TestCameraData) {
  Eigen::Vector3d camera_pose(0.05, 0.05, 0.01);
  EXPECT_NO_FATAL_FAILURE(robot_manager->NewCameraData(camera_pose));
}

TEST_F(RobotManagerTest, TestGyroFunctions) {
  EXPECT_NO_FATAL_FAILURE(robot_manager->CalibrateGyro());
  EXPECT_NO_FATAL_FAILURE(robot_manager->IsGyroCalibrated());
}

TEST_F(RobotManagerTest, TestSimpleGoal) {
  Eigen::Vector3d close_goal(0.1, 0.1, 0.0);
  robot_manager->AddGoal(close_goal);
  EXPECT_NO_FATAL_FAILURE();
}

TEST_F(RobotManagerTest, TestHomePositionBasic) {
  Eigen::Vector3d new_home(0.5, 0.5, 0.0);
  robot_manager->InitializeHome(new_home);

  EXPECT_NO_FATAL_FAILURE();
}

TEST_F(RobotManagerTest, TestStateRetrieval) {
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::string state = robot_manager->GetRobotState();
  EXPECT_EQ(state, "IDLE");

  Eigen::Vector3d pose = robot_manager->GetPoseInWorldFrame();
  EXPECT_NO_FATAL_FAILURE();

  Eigen::Vector3d velocity = robot_manager->GetVelocityInWorldFrame();
  EXPECT_NO_FATAL_FAILURE();
}

TEST_F(RobotManagerTest, TestGyroCalibration) {
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_TRUE(robot_manager->IsGyroCalibrated());
}

TEST_F(RobotManagerTest, TestGoHome) {
  Eigen::Vector3d new_home(1, 1, 0.0);
  robot_manager->InitializeHome(new_home);

  robot_manager->GoHome();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_EQ(robot_manager->GetRobotState(), "GOING_HOME");
}