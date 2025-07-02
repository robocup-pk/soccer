#include <iostream>
#include <thread>
#include <chrono>

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
  std::cout << "[hw::MotorDriver::SetWheelSpeedsRpm] wheel_speeds_rpm = "
            << wheel_speeds_rpm.transpose() << std::endl;
  // Send speeds over serial

  std::vector<int32_t> speeds_rpm = {static_cast<int32_t>(std::round(wheel_speeds_rpm[0])),
                                     static_cast<int32_t>(std::round(wheel_speeds_rpm[1])),
                                     static_cast<int32_t>(std::round(wheel_speeds_rpm[2])),
                                     static_cast<int32_t>(std::round(wheel_speeds_rpm[3]))};

  std::vector<uint8_t> buffer(17);
  buffer[0] = 'x';  // header
  // STM and Raspberry pi both are little-endian
  std::memcpy(&buffer[1], &speeds_rpm[0], sizeof(int32_t));
  std::memcpy(&buffer[5], &speeds_rpm[1], sizeof(int32_t));
  std::memcpy(&buffer[9], &speeds_rpm[2], sizeof(int32_t));
  std::memcpy(&buffer[13], &speeds_rpm[3], sizeof(int32_t));

  std::cout << "[hwff::MotorDriver::SetWheelSpeedsRpm] Sent = " << speeds_rpm[0] << " "
            << speeds_rpm[1] << " " << speeds_rpm[2] << " " << speeds_rpm[3] << std::endl;
  std::cout << "Size of buffer: " << buffer.size() << std::endl;
  shared_serial_port->Write(buffer);
  shared_serial_port->DrainWriteBuffer();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

Eigen::Vector4d hw::MotorDriver::GetEncoderTicks() {
  if (motor_type == MotorType::MODEL) {
    for (int i = 0; i < kin::RobotDescription::num_wheels; ++i) {
      encoder_ticks[i] = motors[i].GetTicks();
    }
    return encoder_ticks;
  }

  // Get encoder ticks using serial

  // First Byte
  if (shared_serial_port->GetNumberOfBytesAvailable() < 1) {
    return Eigen::Vector4d();
  }
  // Check if it's header
  uint8_t header;
  shared_serial_port->ReadByte(header, 1000);
  if (header != 'x') return Eigen::Vector4d();
  // Next 16 bytes
  std::vector<uint8_t> buffer(16);
  shared_serial_port->Read(buffer, 16, 1000);

  // Extract the rpms
  std::vector<int> rpm(4);
  for (int i = 0; i < 4; ++i) {
    std::memcpy(&rpm[i], &buffer[i * 4], sizeof(int32_t));
  }

  std::cout << "RPMS: " << rpm[0] << " " << rpm[1] << " " << rpm[2] << " " << rpm[3] << std::endl;

  // Convert to Eigen

  return Eigen::Vector4d();
}

bool hw::MotorDriver::NewDataAvailable() { return new_data_available; }