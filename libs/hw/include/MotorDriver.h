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

  void SetWheelSpeedsRpm(Eigen::Vector4d& wheel_speeds_rpm);

  Eigen::Vector4d GetEncoderTicks();

  bool NewDataAvailable();

  bool VerifyRpms(std::vector<int> rpms);

 private:
#ifdef BUILD_ON_PI
  MotorType motor_type = MotorType::REAL;
#else
  MotorType motor_type = MotorType::MODEL;
#endif

  std::vector<MotorModel> motors;
  Eigen::Vector4d encoder_ticks;

  std::shared_ptr<LibSerial::SerialPort> shared_serial_port;
  std::mutex shared_serial_port_mutex;

  bool new_data_available = true;
};

}  // namespace hw

#endif  // MOTOR_DRIVER_H