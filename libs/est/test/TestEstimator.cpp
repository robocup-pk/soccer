#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include <cmath>

#include <Eigen/Dense>

#include "Estimator.h"
#include "HwConfig.h"
#include "Utils.h"
#include "HardwareManager.h"
#include "RobotModel.h"

class EstimatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    tolerance << 0.25, 0.25, 0.25;
    estimator = std::make_unique<est::Estimator>();
    hardware_manager = std::make_unique<hw::HardwareManager>();
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
    std::optional<Eigen::Vector4d> motors_rpms = hardware_manager->NewMotorsRpms();
    if (motors_rpms.has_value()) estimator->NewMotorsData(motors_rpms.value());

    if (iteration % 2 == 0) {
      Eigen::Vector3d pose_cam = velocity_fBody * elapsed_time_s;
      estimator->NewCameraData(pose_cam);
    }

    ++iteration;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Sleep for 10ms
  }

  Eigen::Vector3d error = (estimator->GetPose() - pose_true).cwiseAbs();
  std::cout << "Pose tru: " << pose_true.transpose() << std::endl;
  std::cout << "Pose est: " << estimator->GetPose().transpose() << std::endl;
  std::cout << "Error: " << error.transpose() << std::endl;
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
    std::optional<Eigen::Vector4d> motors_rpms = hardware_manager->NewMotorsRpms();
    if (motors_rpms.has_value()) estimator->NewMotorsData(motors_rpms.value());

    if (iteration % 4 == 0) {
      Eigen::Vector3d pose_cam = velocity_fBody * elapsed_time_s;
      estimator->NewCameraData(pose_cam);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ++iteration;
  }

  Eigen::Vector3d error = (estimator->GetPose() - pose_true).cwiseAbs();
  std::cout << "Pose tru: " << pose_true.transpose() << std::endl;
  std::cout << "Pose est: " << estimator->GetPose().transpose() << std::endl;
  std::cout << "Error: " << error.transpose() << std::endl;
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
    std::optional<Eigen::Vector4d> motors_rpms = hardware_manager->NewMotorsRpms();
    if (motors_rpms.has_value()) estimator->NewMotorsData(motors_rpms.value());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  Eigen::Vector3d error = (estimator->GetPose() - pose_true).cwiseAbs();
  std::cout << "Pose tru: " << pose_true.transpose() << std::endl;
  std::cout << "Pose est: " << estimator->GetPose().transpose() << std::endl;
  std::cout << "Error: " << error.transpose() << std::endl;
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
    std::optional<Eigen::Vector4d> motors_rpms = hardware_manager->NewMotorsRpms();
    if (motors_rpms.has_value()) estimator->NewMotorsData(motors_rpms.value());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  Eigen::Vector3d error = (estimator->GetPose() - pose_true).cwiseAbs();
  std::cout << "Pose tru: " << pose_true.transpose() << std::endl;
  std::cout << "Pose est: " << estimator->GetPose().transpose() << std::endl;
  std::cout << "Error: " << error.transpose() << std::endl;
  EXPECT_TRUE((error.array() <= tolerance.array()).all());
}

TEST_F(EstimatorTest, TestMotionWithInitialPose) {
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
  Eigen::Vector4d ticks;

  int iteration = 0;
  while (elapsed_time_s < t_sec) {
    elapsed_time_s = util::GetCurrentTime() - start_time;
    std::optional<Eigen::Vector4d> motors_rpms = hardware_manager->NewMotorsRpms();
    if (motors_rpms.has_value()) estimator->NewMotorsData(motors_rpms.value());

    if (iteration % 2 == 0) {
      Eigen::Vector3d pose_cam =
          pose_init + velocity_fWorld * elapsed_time_s;  // TODO: can bring it back in time
      estimator->NewCameraData(pose_cam);
    }

    ++iteration;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  Eigen::Vector3d error = (estimator->GetPose() - pose_true).cwiseAbs();
  std::cout << "Pose tru: " << pose_true.transpose() << std::endl;
  std::cout << "Pose est: " << estimator->GetPose().transpose() << std::endl;
  std::cout << "Error: " << error.transpose() << std::endl;
  EXPECT_TRUE((error.array() <= tolerance.array()).all());
}

TEST_F(EstimatorTest, TestMotionWithAllSensors) {
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
  Eigen::Vector4d ticks;

  int iteration = 0;
  while (elapsed_time_s < t_sec) {
    elapsed_time_s = util::GetCurrentTime() - start_time;
    std::optional<Eigen::Vector4d> motors_rpms = hardware_manager->NewMotorsRpms();
    if (motors_rpms.has_value()) estimator->NewMotorsData(motors_rpms.value());
    estimator->NewGyroData(velocity_fWorld[2]);

    if (iteration % 2 == 0) {
      Eigen::Vector3d pose_cam = pose_init + velocity_fWorld * elapsed_time_s;
      estimator->NewCameraData(pose_cam);
    }

    ++iteration;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  Eigen::Vector3d error = (estimator->GetPose() - pose_true).cwiseAbs();
  std::cout << "Pose tru: " << pose_true.transpose() << std::endl;
  std::cout << "Pose est: " << estimator->GetPose().transpose() << std::endl;
  std::cout << "Error: " << error.transpose() << std::endl;
  EXPECT_TRUE((error.array() <= tolerance.array()).all());
}