#include <iostream>

#include <libserial/SerialPort.h>

#include "RobotModel.h"
#include "MotorDriver.h"

hw::MotorDriver::MotorDriver(std::shared_ptr<LibSerial::SerialPort> shared_serial_port)
    : shared_serial_port(shared_serial_port) {
  std::cout << "[hw::MotorDriver::MotorDriver]" << std::endl;
  encoder_ticks << 0, 0, 0, 0;
  motors = std::vector<hw::MotorModel>(kin::RobotDescription::num_wheels);
}

void hw::MotorDriver::SetWheelSpeedsRpm(Eigen::Vector4d& wheel_speeds_rpm) {
  if (motor_type == MotorType::MODEL) {
    for (int i = 0; i < kin::RobotDescription::num_wheels; ++i) {
      motors[i].SetWheelSpeedRpm(wheel_speeds_rpm[i]);
    }
    return;
  }

  // Send speeds over serial
  std::vector<int> speeds_rpm(4);
  speeds_rpm.push_back(wheel_speeds_rpm[0]);
  speeds_rpm.push_back(wheel_speeds_rpm[1]);
  speeds_rpm.push_back(wheel_speeds_rpm[2]);
  speeds_rpm.push_back(wheel_speeds_rpm[3]);

  std::vector<uint8_t> buffer(17);
  buffer[0] = 'x';  // header
  std::memcpy(&buffer[1], &speeds_rpm[0], sizeof(int32_t));
  std::memcpy(&buffer[5], &speeds_rpm[1], sizeof(int32_t));
  std::memcpy(&buffer[9], &speeds_rpm[2], sizeof(int32_t));
  std::memcpy(&buffer[13], &speeds_rpm[3], sizeof(int32_t));
  shared_serial_port->Write(buffer);
  shared_serial_port->DrainWriteBuffer();
}

Eigen::Vector4d hw::MotorDriver::GetEncoderTicks() {
  if (motor_type == MotorType::MODEL) {
    for (int i = 0; i < kin::RobotDescription::num_wheels; ++i) {
      encoder_ticks[i] = motors[i].GetTicks();
    }
    return encoder_ticks;
  }

  // Get encoder ticks using serial
  return Eigen::Vector4d();
}

bool hw::MotorDriver::NewDataAvailable() { return new_data_available; }