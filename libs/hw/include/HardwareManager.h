#ifndef HARDWARE_MANAGER_H
#define HARDWARE_MANAGER_H

#include <vector>
#include <thread>
#include <optional>
#include <memory>

#include <Eigen/Dense>
#include <libserial/SerialPort.h>

#include "RobotModel.h"
#include "SensorDriver.h"

namespace hw {
class HardwareManager {
 public:
  HardwareManager();

  // Sensing
  std::optional<Eigen::Vector4d> NewMotorsRpms();
  std::optional<double> NewGyroAngularVelocity();
  std::optional<Eigen::Vector3d> NewCameraData();
  void NewCameraData(Eigen::Vector3d pose_from_camera);

  // Control
  void SetBodyVelocity(Eigen::Vector3d velocity_fBody);
  void SetWheelSpeedsRpm(Eigen::Vector4d& wheel_speeds_rpm);

  // Serial Comms
  void InitializeSerialPort();

  bool IsGyroCalibrated();
  void CalibrateGyro();

  ~HardwareManager();

 private:
  // std::unique_ptr<CameraDriver> camera_driver;
  Eigen::Vector3d camera_data;

  bool new_sensor_data;
  bool new_camera_data;

  std::unique_ptr<SensorDriver> sensor_driver;
  std::shared_ptr<kin::RobotModel> robot_model;

  // Serial port
  std::shared_ptr<LibSerial::SerialPort> shared_serial_port;
};
}  // namespace hw

#endif  // HARDWARE_MANAGER_H