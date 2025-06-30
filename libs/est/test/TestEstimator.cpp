#include <gtest/gtest.h>
#include <thread>
#include <chrono>

#include "Estimator.h"
#include "HwConfig.h"
#include "Utils.h"
#include "MotorModel.h"

#include <gtest/gtest.h>
#include <cmath>

#include <Eigen/Dense>

#include "HardwareManager.h"
#include "RobotModel.h"

class EstimatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    tolerance << 0.05, 0.05, 0.05;
    kin::RobotDescription robot_desc;

    // Square configuration with wheels at corners
    robot_desc.wheel_positions_m = {
        {0.15, 0.15},    // wheel 1: front-left
        {-0.15, 0.15},   // wheel 2: rear-left
        {-0.15, -0.15},  // wheel 3: rear-right
        {0.15, -0.15}    // wheel 4: front-right
    };

    // Wheel angles (perpendicular to radial direction for typical omniwheel setup)
    robot_desc.wheel_angles_rad = {
        -M_PI / 4,     // -45° (wheel 1)
        M_PI / 4,      // 45° (wheel 2)
        3 * M_PI / 4,  // 135° (wheel 3)
        -3 * M_PI / 4  // -135° (wheel 4)
    };

    std::shared_ptr<kin::RobotModel> robot_model = std::make_shared<kin::RobotModel>(robot_desc);

    estimator = std::make_unique<est::Estimator>(robot_model);
    hardware_manager = std::make_unique<hw::HardwareManager>(robot_model);
  }

  std::unique_ptr<est::Estimator> estimator;
  std::unique_ptr<hw::HardwareManager> hardware_manager;
  Eigen::Vector3d tolerance;
  std::vector<hw::MotorModel> motors = std::vector<hw::MotorModel>(4);
  std::vector<double> wheel_speeds_radps = std::vector<double>(4);
};

TEST_F(EstimatorTest, TestForwardMotionWithCamera) {
  double t_sec = 1;

  // Ground Truth
  Eigen::Vector3d velocity_fBody(1, 0, 0);
  Eigen::Vector3d pose_true = velocity_fBody * t_sec;  // [1, 0, 0]

  // START THE MOTORS
  hardware_manager->SetBodyVelocity(velocity_fBody);

  double start_time_s = util::GetCurrentTime();
  double elapsed_time_s = util::GetCurrentTime() - start_time_s;
  int iteration = 0;
  Eigen::Vector4d ticks;

  while (elapsed_time_s < t_sec) {
    elapsed_time_s = util::GetCurrentTime() - start_time_s;
    ticks = hardware_manager->GetEncoderTicks();
    estimator->NewEncoderData(ticks);

    if (iteration % 2 == 0) {
      Eigen::Vector3d pose_cam = velocity_fBody * elapsed_time_s;
      estimator->NewCameraData(pose_cam);
    }

    ++iteration;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Sleep for 10ms
  }

  Eigen::Vector3d error = (estimator->GetPose() - pose_true).cwiseAbs();
  EXPECT_TRUE((error.array() <= tolerance.array()).all());
}

TEST_F(EstimatorTest, TestForwardMotionWithCameraAgain) {
  double t_sec = 1;
  estimator->initialized_pose = true;

  // GROUND TRUTH
  Eigen::Vector3d velocity_fBody(2, 0, 0);
  Eigen::Vector3d pose_true = velocity_fBody * t_sec;

  // START THE MOTORS
  hardware_manager->SetBodyVelocity(velocity_fBody);

  // NOW PERFORM ESTIMATION. Encoder ticks at 100 Hz
  Eigen::Vector4d ticks;
  double start_time = util::GetCurrentTime();
  double elapsed_time_s = 0;

  int iteration = 0;
  while (elapsed_time_s < t_sec) {
    elapsed_time_s = util::GetCurrentTime() - start_time;
    ticks = hardware_manager->GetEncoderTicks();
    estimator->NewEncoderData(ticks);

    if (iteration % 4 == 0) {
      Eigen::Vector3d pose_cam = velocity_fBody * elapsed_time_s;
      estimator->NewCameraData(pose_cam);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ++iteration;
  }

  Eigen::Vector3d error = (estimator->GetPose() - pose_true).cwiseAbs();
  EXPECT_TRUE((error.array() <= tolerance.array()).all());
}

TEST_F(EstimatorTest, TestForwardMotionDeadReckoning) {
  double t_sec = 1;
  estimator->initialized_pose = true;

  // GROUND TRUTH
  Eigen::Vector3d velocity_fBody(1, 0, 0);
  Eigen::Vector3d pose_true = velocity_fBody * t_sec;

  // ESTIMATION
  hardware_manager->SetBodyVelocity(velocity_fBody);
  double start_time = util::GetCurrentTime();
  double elapsed_time_s = 0;
  Eigen::Vector4d ticks;

  while (elapsed_time_s < t_sec) {
    elapsed_time_s = util::GetCurrentTime() - start_time;
    ticks = hardware_manager->GetEncoderTicks();
    estimator->NewEncoderData(ticks);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  Eigen::Vector3d error = (estimator->GetPose() - pose_true).cwiseAbs();
  EXPECT_TRUE((error.array() <= tolerance.array()).all());
}

TEST_F(EstimatorTest, TestForwardMotionDeadReckoningWithRotation) {
  double t_sec = 1;
  estimator->initialized_pose = true;

  // GROUND TRUTH
  Eigen::Vector3d velocity_fBody(1, -1, 2);
  Eigen::Vector3d pose_true = velocity_fBody * t_sec;

  // ESTIMATION
  hardware_manager->SetBodyVelocity(velocity_fBody);
  double start_time = util::GetCurrentTime();
  double elapsed_time_s = 0;
  Eigen::Vector4d ticks;

  while (elapsed_time_s < t_sec) {
    elapsed_time_s = util::GetCurrentTime() - start_time;
    ticks = hardware_manager->GetEncoderTicks();
    estimator->NewEncoderData(ticks);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  Eigen::Vector3d error = (estimator->GetPose() - pose_true).cwiseAbs();
  EXPECT_TRUE((error.array() <= tolerance.array()).all());
}