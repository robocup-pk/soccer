#include <iostream>

#include "HardwareManager.h"
#include "RobotModel.h"

hw::HardwareManager::HardwareManager(std::shared_ptr<kin::RobotModel> robot_model)
    : robot_model(robot_model) {
  motor_driver = std::make_unique<MotorDriver>(motor_driver_port);
  // gyro_driver = std::make_unique<GyroDriver>(gyro_driver_port);
}

void hw::HardwareManager::SetBodyVelocity(Eigen::Vector3d& velocity_fBody) {
  Eigen::Vector4d wheel_speeds_rpm = robot_model->RobotVelocityToWheelSpeedsRpm(velocity_fBody);
  SetWheelSpeedsRpm(wheel_speeds_rpm);
}

void hw::HardwareManager::SetWheelSpeedsRpm(Eigen::Vector4d& wheel_speeds_rpm) {
  motor_driver->SetWheelSpeedsRpm(wheel_speeds_rpm);
}

std::optional<Eigen::Vector4d> hw::HardwareManager::NewEncoderTicks() {
  if (motor_driver->NewDataAvailable()) return motor_driver->GetEncoderTicks();
  return std::nullopt;
}

std::optional<double> hw::HardwareManager::NewGyroAngularVelocity() {
  // if (gyro_driver.NewDataAvailable()) return gyro_driver.GetAngularVelocity();
  return std::nullopt;
}

std::optional<Eigen::Vector3d> hw::HardwareManager::NewCameraData() {
  // if (camera_driver.NewDataAvailable()) return camera_driver.GetPose();
  return std::nullopt;
}

hw::HardwareManager::~HardwareManager() { robot_model = nullptr; }