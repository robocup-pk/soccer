#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "MotorModel.h"

namespace hw {

enum class MotorType { MODEL, REAL };

class MotorDriver {
 public:
  MotorDriver(std::string port);

  void SetWheelSpeedsRpm(Eigen::Vector4d& wheel_speeds_rpm);

  Eigen::Vector4d GetEncoderTicks();

  bool NewDataAvailable();

 private:
  MotorType motor_type = MotorType::MODEL;

  std::vector<MotorModel> motors;
  Eigen::Vector4d encoder_ticks;

  std::string serial_port;

  bool new_data_available = true;
};

}  // namespace hw

#endif  // MOTOR_DRIVER_H