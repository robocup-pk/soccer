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

#include "RobotModel.h"

class EstimatorTest : public ::testing::Test {
 protected:
  void SetUp() override { tolerance << 0.10, 0.10, 0.10; }  // 5 cm, 0.05 rad (3 degrees)

  est::Estimator estimator;
  Eigen::Vector3d tolerance;
  std::vector<hw::MotorModel> motors = std::vector<hw::MotorModel>(4);
  std::vector<double> wheel_speeds_radps = std::vector<double>(4);
};

TEST_F(EstimatorTest, TestEncoders) {
  double t_sec = 1;

  // Ground Truth
  Eigen::Vector3d velocity_fBody(1, 0, 0);
  Eigen::Vector3d pose_true = velocity_fBody * t_sec;  // [1, 0, 0]

  // START THE MOTORS
  Eigen::Vector4d wheel_speeds_rpm =
      estimator.robot_model->RobotVelocityToWheelSpeedsRpm(velocity_fBody);
  for (int i = 0; i < 4; ++i) motors[i].SetWheelSpeedRpm(wheel_speeds_rpm[i]);

  Eigen::Vector4d ticks;
  for (int i = 0; i < (t_sec * 100); i++) {
    for (int t = 0; t < 4; ++t) ticks[t] = motors[t].GetTicks();

    estimator.NewEncoderData(ticks);
    if (i % 2 == 0) estimator.NewCameraData(pose_true);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Sleep for 10ms
  }

  Eigen::Vector3d error = (estimator.GetPose() - pose_true).cwiseAbs();
  EXPECT_TRUE((error.array() <= tolerance.array()).all());
}

TEST_F(EstimatorTest, TestForwardMotionWithCamera) {
  double t_sec = 1;

  // GROUND TRUTH
  Eigen::Vector3d velocity_fBody(10, 0, 0);
  Eigen::Vector3d pose_true = velocity_fBody * t_sec;

  // START THE MOTORS
  Eigen::Vector4d wheel_speeds_rpm =
      estimator.robot_model->RobotVelocityToWheelSpeedsRpm(velocity_fBody);
  for (int i = 0; i < 4; ++i) motors[i].SetWheelSpeedRpm(wheel_speeds_rpm[i]);

  // NOW PERFORM ESTIMATION. Encoder ticks at 100 Hz
  Eigen::Vector4d ticks;
  double start_time = util::GetCurrentTime();
  double elapsed_time_s = 0;

  std::cout << std::fixed << std::setprecision(3) << std::endl;

  int iteration = 0;
  do {
    for (int t = 0; t < 4; ++t) ticks[t] = motors[t].GetTicks();

    elapsed_time_s = util::GetCurrentTime() - start_time;
    Eigen::Vector3d pose_cam =
        velocity_fBody * elapsed_time_s;  //  + Eigen::Vector3d(0.01, 0.01, 0.01);
    estimator.NewEncoderData(ticks);

    if (iteration % 4 == 0) estimator.NewCameraData(pose_cam);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ++iteration;
  } while (elapsed_time_s < t_sec);

  Eigen::Vector3d error = (estimator.GetPose() - pose_true).cwiseAbs();
  EXPECT_TRUE((error.array() <= tolerance.array()).all());
}