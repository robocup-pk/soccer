
#include "RobotModel.h"
#include "MotorDriver.h"

hw::MotorDriver::MotorDriver(std::string port) {
  serial_port = port;
  encoder_ticks << 0, 0, 0, 0;
  if (motor_type == MotorType::MODEL)
    motors = std::vector<hw::MotorModel>(kin::RobotDescription::num_wheels);
  // TODO: else if (motor_type == MotorType::REAL) {}
}

void hw::MotorDriver::SetWheelSpeedsRpm(Eigen::Vector4d& wheel_speeds_rpm) {
  for (int i = 0; i < kin::RobotDescription::num_wheels; ++i) {
    motors[i].SetWheelSpeedRpm(wheel_speeds_rpm[i]);
  }
}

Eigen::Vector4d hw::MotorDriver::GetEncoderTicks() {
  for (int i = 0; i < kin::RobotDescription::num_wheels; ++i) {
    encoder_ticks[i] = motors[i].GetTicks();
  }
  return encoder_ticks;
}

bool hw::MotorDriver::NewDataAvailable() { return new_data_available; }