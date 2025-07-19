#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include <cmath>

#include <Eigen/Dense>

#include "StateEstimator.h"
#include "HwConfig.h"
#include "Utils.h"
#include "HardwareManager.h"
#include "RobotModel.h"

class StateEstimatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    tolerance << 0.25, 0.25, 0.25;
    state_estimator = std::make_unique<est::StateEstimator>();
    hardware_manager = std::make_unique<hw::HardwareManager>();
  }

  std::unique_ptr<est::StateEstimator> state_estimator;
  std::unique_ptr<hw::HardwareManager> hardware_manager;
  Eigen::Vector3d tolerance;
  std::vector<hw::MotorModel> motors = std::vector<hw::MotorModel>(4);
  std::vector<double> wheel_speeds_radps = std::vector<double>(4);
};

TEST_F(StateEstimatorTest, TestForwardMotionWithCamera) {
  double t_sec = 1;

  // Ground Truth
  Eigen::Vector3d velocity_fBody(1, 0, 0);
  Eigen::Vector3d pose_true = velocity_fBody * t_sec;  // [1, 0, 0]

  // START THE MOTORS
  hardware_manager->SetBodyVelocity(velocity_fBody);

  double start_time_s = util::GetCurrentTime();
  double elapsed_time_s = util::GetCurrentTime() - start_time_s;
  int iteration = 0;

  while (elapsed_time_s < t_sec) {
    elapsed_time_s = util::GetCurrentTime() - start_time_s;
    std::optional<Eigen::Vector4d> motors_rpms = hardware_manager->NewMotorsRpms();
    if (motors_rpms.has_value()) state_estimator->NewMotorsData(motors_rpms.value());

    if (iteration % 2 == 0) {
      Eigen::Vector3d pose_cam = velocity_fBody * elapsed_time_s;
      state_estimator->NewCameraData(pose_cam);
    }

    ++iteration;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Sleep for 10ms
  }

  Eigen::Vector3d error = (state_estimator->GetPose() - pose_true).cwiseAbs();
  std::cout << "Pose tru: " << pose_true.transpose() << std::endl;
  std::cout << "Pose est: " << state_estimator->GetPose().transpose() << std::endl;
  std::cout << "Error: " << error.transpose() << std::endl;
  EXPECT_TRUE((error.array() <= tolerance.array()).all());
}

TEST_F(StateEstimatorTest, TestForwardMotionWithCameraAgain) {
  double t_sec = 1;
  state_estimator->initialized_pose = true;

  // GROUND TRUTH
  Eigen::Vector3d velocity_fBody(2, 0, 0);
  Eigen::Vector3d pose_true = velocity_fBody * t_sec;

  // START THE MOTORS
  hardware_manager->SetBodyVelocity(velocity_fBody);

  // NOW PERFORM ESTIMATION. Wheel Speeds at 100 Hz
  double start_time = util::GetCurrentTime();
  double elapsed_time_s = 0;

  int iteration = 0;
  while (elapsed_time_s < t_sec) {
    elapsed_time_s = util::GetCurrentTime() - start_time;
    std::optional<Eigen::Vector4d> motors_rpms = hardware_manager->NewMotorsRpms();
    if (motors_rpms.has_value()) state_estimator->NewMotorsData(motors_rpms.value());

    if (iteration % 4 == 0) {
      Eigen::Vector3d pose_cam = velocity_fBody * elapsed_time_s;
      state_estimator->NewCameraData(pose_cam);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ++iteration;
  }

  Eigen::Vector3d error = (state_estimator->GetPose() - pose_true).cwiseAbs();
  std::cout << "Pose tru: " << pose_true.transpose() << std::endl;
  std::cout << "Pose est: " << state_estimator->GetPose().transpose() << std::endl;
  std::cout << "Error: " << error.transpose() << std::endl;
  EXPECT_TRUE((error.array() <= tolerance.array()).all());
}

TEST_F(StateEstimatorTest, TestForwardMotionDeadReckoning) {
  double t_sec = 1;
  Eigen::Vector3d pose_init(0, 0, 0);
  state_estimator->initialized_pose = true;

  // GROUND TRUTH
  Eigen::Vector3d velocity_fBody(1, 0, 0);
  Eigen::Vector3d velocity_fWorld = util::RotateAboutZ(velocity_fBody, 0);
  Eigen::Vector3d pose_true = pose_init + velocity_fWorld * t_sec;

  // ESTIMATION
  hardware_manager->SetBodyVelocity(velocity_fBody);
  double start_time = util::GetCurrentTime();
  double elapsed_time_s = 0;

  while (elapsed_time_s < t_sec) {
    elapsed_time_s = util::GetCurrentTime() - start_time;
    std::optional<Eigen::Vector4d> motors_rpms = hardware_manager->NewMotorsRpms();
    if (motors_rpms.has_value()) state_estimator->NewMotorsData(motors_rpms.value());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  Eigen::Vector3d error = (state_estimator->GetPose() - pose_true).cwiseAbs();
  std::cout << "Pose tru: " << pose_true.transpose() << std::endl;
  std::cout << "Pose est: " << state_estimator->GetPose().transpose() << std::endl;
  std::cout << "Error: " << error.transpose() << std::endl;
  EXPECT_TRUE((error.array() <= tolerance.array()).all());
}

TEST_F(StateEstimatorTest, TestForwardMotionDeadReckoningWithRotation) {
  double t_sec = 1;
  state_estimator->initialized_pose = true;

  // GROUND TRUTH
  Eigen::Vector3d pose_init(0, 0, 0);
  Eigen::Vector3d velocity_fWorld(1, -1, 2);
  Eigen::Vector3d pose_true = pose_init + velocity_fWorld * t_sec;
  std::cout << "This test is failing" << std::endl;
  // ESTIMATION
  double start_time = util::GetCurrentTime();
  double elapsed_time_s = 0;

  while (elapsed_time_s < t_sec) {
    elapsed_time_s = util::GetCurrentTime() - start_time;

    // Control
    Eigen::Vector3d pose_est = state_estimator->GetPose();
    Eigen::Vector3d velocity_fBody = util::RotateAboutZ(velocity_fWorld, -pose_est[2]);
    hardware_manager->SetBodyVelocity(velocity_fBody);

    std::optional<Eigen::Vector4d> motors_rpms = hardware_manager->NewMotorsRpms();
    if (motors_rpms.has_value()) state_estimator->NewMotorsData(motors_rpms.value());
    state_estimator->NewGyroData(velocity_fWorld[2]);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  Eigen::Vector3d error = (state_estimator->GetPose() - pose_true).cwiseAbs();
  std::cout << "Pose tru: " << pose_true.transpose() << std::endl;
  std::cout << "Pose est: " << state_estimator->GetPose().transpose() << std::endl;
  std::cout << "Error: " << error.transpose() << std::endl;
  EXPECT_TRUE((error.array() <= tolerance.array()).all());
}

TEST_F(StateEstimatorTest, TestMotionWithInitialPose) {
  double t_sec = 1;
  Eigen::Vector3d pose_init(1, 1, 1);

  // GROUND TRUTH
  Eigen::Vector3d velocity_fBody(1, -1, 2);
  // Transform body velocities to world frame
  Eigen::Vector3d velocity_fWorld = util::RotateAboutZ(velocity_fBody, pose_init[2]);
  Eigen::Vector3d pose_true = pose_init + velocity_fWorld * t_sec;

  // ESTIMATION
  hardware_manager->SetBodyVelocity(velocity_fBody);
  double start_time = util::GetCurrentTime();
  double elapsed_time_s = 0;

  int iteration = 0;
  while (elapsed_time_s < t_sec) {
    elapsed_time_s = util::GetCurrentTime() - start_time;
    std::optional<Eigen::Vector4d> motors_rpms = hardware_manager->NewMotorsRpms();
    if (motors_rpms.has_value()) state_estimator->NewMotorsData(motors_rpms.value());

    if (iteration % 2 == 0) {
      Eigen::Vector3d pose_cam =
          pose_init + velocity_fWorld * elapsed_time_s;  // TODO: can bring it back in time
      state_estimator->NewCameraData(pose_cam);
    }

    state_estimator->NewGyroData(velocity_fWorld[2]);
    ++iteration;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  Eigen::Vector3d error = (state_estimator->GetPose() - pose_true).cwiseAbs();
  std::cout << "Pose tru: " << pose_true.transpose() << std::endl;
  std::cout << "Pose est: " << state_estimator->GetPose().transpose() << std::endl;
  std::cout << "Error: " << error.transpose() << std::endl;
  EXPECT_TRUE((error.array() <= tolerance.array()).all());
}

TEST_F(StateEstimatorTest, TestMotionWithAllSensors) {
  double t_sec = 1;
  Eigen::Vector3d pose_init(3, 5, 0.5);

  // GROUND TRUTH
  Eigen::Vector3d velocity_fBody(0.6, -0.7, -1.5);
  // Transform body velocities to world frame
  Eigen::Vector3d velocity_fWorld = util::RotateAboutZ(velocity_fBody, pose_init[2]);
  Eigen::Vector3d pose_true = pose_init + velocity_fWorld * t_sec;

  // ESTIMATION
  hardware_manager->SetBodyVelocity(velocity_fBody);
  double start_time = util::GetCurrentTime();
  double elapsed_time_s = 0;

  int iteration = 0;
  while (elapsed_time_s < t_sec) {
    elapsed_time_s = util::GetCurrentTime() - start_time;
    std::optional<Eigen::Vector4d> motors_rpms = hardware_manager->NewMotorsRpms();
    if (motors_rpms.has_value()) state_estimator->NewMotorsData(motors_rpms.value());
    state_estimator->NewGyroData(velocity_fWorld[2]);

    if (iteration % 2 == 0) {
      Eigen::Vector3d pose_cam = pose_init + velocity_fWorld * elapsed_time_s;
      state_estimator->NewCameraData(pose_cam);
    }

    ++iteration;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  Eigen::Vector3d error = (state_estimator->GetPose() - pose_true).cwiseAbs();
  std::cout << "Pose tru: " << pose_true.transpose() << std::endl;
  std::cout << "Pose est: " << state_estimator->GetPose().transpose() << std::endl;
  std::cout << "Error: " << error.transpose() << std::endl;
  EXPECT_TRUE((error.array() <= tolerance.array()).all());
}