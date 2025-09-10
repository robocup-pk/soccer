#ifndef SENSOR_DRIVER_H
#define SENSOR_DRIVER_H

#ifndef NO_SERIAL_PORT
#include <libserial/SerialPort.h>
#endif
#include <mutex>
#include <Eigen/Dense>
#include <vector>
#include <memory>

#include "SensorModel.h"

namespace hw {

enum class SensorType { MODEL, REAL };

// Forward declaration for cross-platform compatibility
#ifndef NO_SERIAL_PORT
namespace LibSerial { class SerialPort; }
#endif

class SensorDriver {
 public:
#ifndef NO_SERIAL_PORT
  SensorDriver(std::shared_ptr<LibSerial::SerialPort> shared_serial_port);
#else
  SensorDriver(std::shared_ptr<void> dummy_port = nullptr);
#endif

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

#ifndef NO_SERIAL_PORT
  std::shared_ptr<LibSerial::SerialPort> shared_serial_port;
#else
  std::shared_ptr<void> shared_serial_port;  // Dummy pointer for macOS
#endif
  std::mutex shared_serial_port_mutex;
};

}  // namespace hw

#endif  // SENSOR_DRIVER_H