#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <mutex>

#include <libserial/SerialPort.h>

#include "MotorModel.h"

namespace hw {

enum class MotorType { MODEL, REAL };

class MotorDriver {
 public:
  MotorDriver(std::shared_ptr<LibSerial::SerialPort> shared_serial_port);

  void SendWheelSpeedsRpm(Eigen::Vector4d& wheel_speeds_rpm);

  Eigen::Vector4d GetMotorsRpms();

  bool NewDataAvailable();

  bool VerifyRpms(std::vector<int> rpms);

  bool new_data_available = true;

 private:
#ifdef BUILD_ON_PI
  MotorType motor_type = MotorType::REAL;
#else
  MotorType motor_type = MotorType::MODEL;
#endif

  std::vector<MotorModel> motors;
  Eigen::Vector4d motors_rpms;

  std::shared_ptr<LibSerial::SerialPort> shared_serial_port;
  std::mutex shared_serial_port_mutex;
};

}  // namespace hw

#endif  // MOTOR_DRIVER_H