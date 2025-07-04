#include <iostream>
#include <thread>
#include <chrono>

#include <libserial/SerialPort.h>

#include "RobotModel.h"
#include "Utils.h"
#include "MotorDriver.h"

hw::MotorDriver::MotorDriver(std::shared_ptr<LibSerial::SerialPort> shared_serial_port)
    : shared_serial_port(shared_serial_port) {
  std::cout << "[hw::MotorDriver::MotorDriver]" << std::endl;
  motors_rpms << 0, 0, 0, 0;
  motors = std::vector<hw::MotorModel>(kin::RobotDescription::num_wheels);
}

void hw::MotorDriver::SendWheelSpeedsRpm(Eigen::Vector4d& wheel_speeds_rpm) {
  if (motor_type == MotorType::MODEL) {
    for (int i = 0; i < kin::RobotDescription::num_wheels; ++i) {
      motors[i].SetWheelSpeedRpm(wheel_speeds_rpm[i]);
    }
    // std::cout << "[hw::MotorDriver::SendWheelSpeedsRpm] MODEL RPMS: "
    //           << wheel_speeds_rpm.transpose() << std::endl;

    return;
  }

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

  std::cout << "[hw::MotorDriver::SendWheelSpeedsRpm] " << speeds_rpm[0] << " " << speeds_rpm[1]
            << " " << speeds_rpm[2] << " " << speeds_rpm[3] << std::endl;

  std::unique_lock<std::mutex> lock(shared_serial_port_mutex);
  shared_serial_port->FlushOutputBuffer();
  util::WaitMs(2);
  shared_serial_port->Write(buffer);
  util::WaitMs(2);
  shared_serial_port->DrainWriteBuffer();
  util::WaitMs(1);
}

Eigen::Vector4d hw::MotorDriver::GetMotorsRpms() {
  if (motor_type == MotorType::MODEL) {
    for (int i = 0; i < kin::RobotDescription::num_wheels; ++i) {
      motors_rpms[i] = motors[i].GetRpm();
    }
    new_data_available = true;
    // std::cout << "[hw::MotorDriver::GetMotorsRpms] MODEL RPMS: " << motors_rpms.transpose()
    //           << std::endl;
    return motors_rpms;
  }

  // Get encoder ticks using serial
  // First Byte
  if (shared_serial_port->GetNumberOfBytesAvailable() < 1) return Eigen::Vector4d();

  std::vector<int> rpm(4);
  std::vector<uint8_t> buffer(16);

  {
    // Check if it's header
    uint8_t header;
    std::unique_lock<std::mutex> lock(shared_serial_port_mutex);
    shared_serial_port->ReadByte(header, 1000);
    if (header != 'x') return Eigen::Vector4d();
    util::WaitMs(1);
    shared_serial_port->Read(buffer, 16, 1000);
    util::WaitMs(1);
    shared_serial_port->FlushInputBuffer();
  }

  // Extract the rpms
  for (int i = 0; i < 4; ++i) {
    std::memcpy(&rpm[i], &buffer[i * 4], sizeof(int32_t));
  }

  if (VerifyRpms(rpm)) {
    new_data_available = true;
    std::cout << "[hw::MotorDriver::GetMotorRpms] " << rpm[0] << " " << rpm[1] << " " << rpm[2]
              << " " << rpm[3] << std::endl;
    return Eigen::Vector4d(rpm[0], rpm[1], rpm[2], rpm[3]);
  } else {
    std::cout << "[hw::MotorDriver::GetMotorRpms] INVALID: " << rpm[0] << " " << rpm[1] << " "
              << rpm[2] << " " << rpm[3] << std::endl;
    return Eigen::Vector4d(0, 0, 0, 0);
  }
}

bool hw::MotorDriver::NewDataAvailable() { return new_data_available; }

bool hw::MotorDriver::VerifyRpms(std::vector<int> rpms) {
  for (int& rpm : rpms) {
    if (std::abs(rpm) > 400) return false;
  }
  return true;
}