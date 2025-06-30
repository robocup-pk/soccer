#ifndef HARDWARE_MANAGER_H
#define HARDWARE_MANAGER_H

#include <vector>
#include <thread>

#include <Eigen/Dense>

#include "RobotModel.h"
#include "MotorModel.h"

namespace hw {
class HardwareManager {
 public:
  HardwareManager(std::shared_ptr<kin::RobotModel> robot_model);

  // Sensing
  void SenseLoop();
  Eigen::Vector4d GetEncoderTicks();
  // double GetGyroAngularVelocity();

  // Control
  void ControlLoop();
  void SetBodyVelocity(Eigen::Vector3d& velocity_fBody);
  void SetWheelSpeedsRpm(Eigen::Vector4d& wheel_speeds_rpm);

  ~HardwareManager();

 private:
  std::vector<MotorModel> motors;
  // GyroModel gyro;

  std::thread sense_thread;
  std::thread control_thread;

  Eigen::Vector4d current_wheel_speeds_rpm;
  Eigen::Vector4d current_encoder_ticks;

  bool hw_manager_running;

  std::shared_ptr<kin::RobotModel> robot_model;
};
}  // namespace hw

#endif  // HARDWARE_MANAGER_H