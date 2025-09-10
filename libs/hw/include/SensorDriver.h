#ifndef SENSOR_DRIVER_H
#define SENSOR_DRIVER_H

#include <libserial/SerialPort.h>
#include <mutex>
#include <Eigen/Dense>
#include <vector>

#include "SensorModel.h"

namespace hw {

enum class SensorType { MODEL, REAL };

class SensorDriver {
 public:
  SensorDriver(std::shared_ptr<LibSerial::SerialPort> shared_serial_port);

  void SetAngularVelocityRadps(double w_radps);
  double GetAngularVelocityRadps();
  bool NewDataAvailable();

  void SendWheelSpeedRpm(Eigen::Vector4d& wheel_speeds_rpm);
  std::pair<Eigen::Vector4d, int> GetSensorsData();
  bool VerifyRpms(std::vector<int> rpms);
  void CalibrateGyro();
  bool IsGyroCalibrated();
  void SetGyroOnCalibration();

  bool new_data_available = true;
  
  private:
  #ifdef BUILD_ON_PI
  SensorType sensor_type = SensorType::REAL;
  #else
  SensorType sensor_type = SensorType::MODEL;
  #endif
  
  SensorModel gyro;
  
  bool gyro_calibrated;
  double gyro_wradps;

  bool reset_gyro_calibration; 

  std::vector<SensorModel> motors;
  Eigen::Vector4d motors_rpms;
  int gyro_mdeg_ps;

  int num_of_iterations_for_gyro;
  double bias_in_gyro;

  std::shared_ptr<LibSerial::SerialPort> shared_serial_port;
  std::mutex shared_serial_port_mutex;
};

}  // namespace hw

#endif  // SENSOR_DRIVER_H