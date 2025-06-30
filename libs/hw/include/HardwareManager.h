#ifndef HARDWARE_MANAGER_H
#define HARDWARE_MANAGER_H

#include <vector>
#include <thread>

#include <Eigen/Dense>

#include "RobotModel.h"
#include "MotorDriver.h"

namespace hw {
class HardwareManager {
 public:
  HardwareManager(std::shared_ptr<kin::RobotModel> robot_model);

  // Sensing
  std::optional<Eigen::Vector4d> NewEncoderTicks();
  std::optional<double> NewGyroAngularVelocity();
  std::optional<Eigen::Vector3d> NewCameraData();

  // Control
  void SetBodyVelocity(Eigen::Vector3d& velocity_fBody);
  void SetWheelSpeedsRpm(Eigen::Vector4d& wheel_speeds_rpm);

  ~HardwareManager();

 private:
  std::unique_ptr<MotorDriver> motor_driver;
  // std::unique_ptr<GyroDriver> gyro_driver;
  // std::unique_ptr<CameraDriver> camera_driver;

  std::string motor_driver_port = "/dev/ttyUSB0";
  std::string gyro_driver_port = "/dev/ttyUSB1";

  bool new_encoder_ticks = false;
  bool new_gyro_data = false;
  bool new_camera_data = false;

  std::shared_ptr<kin::RobotModel> robot_model;
};
}  // namespace hw

#endif  // HARDWARE_MANAGER_H