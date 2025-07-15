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
  // std::cout << "hw::MotorDriver::SendWheelSpeedsRpm" << std::endl;
  if (motor_type == MotorType::MODEL) {
    for (int i = 0; i < kin::RobotDescription::num_wheels; ++i) {
      motors[i].SetWheelSpeedRpm(wheel_speeds_rpm[i]);
    }
    std::cout << "[hw::MotorDriver::SendWheelSpeedsRpm] MODEL RPMS: "
              << wheel_speeds_rpm.transpose() << std::endl;

    return;
  }

  // PROTOCOL: Send 17 bytes to microcontroller
  // Format: [header 'x'][motor0 4bytes][motor1 4bytes][motor2 4bytes][motor3 4bytes]
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

  std::unique_lock<std::mutex> lock(shared_serial_port_mutex);  // 5 ms
  shared_serial_port->FlushOutputBuffer();
  util::WaitMs(2);
  shared_serial_port->Write(buffer);
  util::WaitMs(2);
  shared_serial_port->DrainWriteBuffer();
  util::WaitMs(1);

  // std::cout << "[DEBUG] Sending " << buffer.size() << " bytes:" << std::endl;
  // std::cout << "Header: 0x" << std::hex << (int)buffer[0] << std::dec << " ('" << (char)buffer[0]
  //           << "')" << std::endl;

  // for (int motor = 0; motor < 4; motor++) {
  //   int32_t speed_value;
  //   std::memcpy(&speed_value, &buffer[1 + motor * 4], sizeof(int32_t));

  //   std::cout << "Motor " << motor << ": " << speeds_rpm[motor] << " -> bytes: ";
  //   for (int i = 0; i < 4; i++) {
  //     std::cout << "0x" << std::hex << (int)buffer[1 + motor * 4 + i] << " ";
  //   }
  //   std::cout << std::dec << "(reconstructed: " << speed_value << ")" << std::endl;
  // }
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

  std::vector<int> rpm(4);
  std::vector<uint8_t> buffer(20);
  // std::cout << "a\n";

  {
    std::unique_lock<std::mutex> lock(shared_serial_port_mutex);

    size_t available_bytes = shared_serial_port->GetNumberOfBytesAvailable();
    // std::cout << "[hw::MotorDriver::GetMotorRpms] Bytes Available: " << available_bytes
              // << std::endl;

    // Need at least 21 bytes for complete response packet
    if (available_bytes < 21) {
      return Eigen::Vector4d();
    }

    // Find sync - search for header 'x'
    uint8_t byte;
    bool found_header = false;
    size_t max_search = std::min(available_bytes, size_t(100));  // Limit search

    for (size_t i = 0; i < max_search; ++i) {
      if (shared_serial_port->GetNumberOfBytesAvailable() < 1) break;

      shared_serial_port->ReadByte(byte, 10);
      if (byte == 'x') {
        found_header = true;
        // std::cout << "[hw::MotorDriver::GetMotorRpms] Found header 'x'" << std::endl;
        break;
      }
      std::cout << "[SYNC] Discarded byte: '" << (char)byte << "' (0x" << std::hex << (int)byte
                << std::dec << ")" << std::endl;
    }

    if (!found_header) {
      std::cout << "[hw::MotorDriver::GetMotorRpms] No header found, flushing buffer" << std::endl;
      shared_serial_port->FlushInputBuffer();
      return Eigen::Vector4d();
    }

    // Check if we have enough bytes for the complete payload (20 bytes)
    if (shared_serial_port->GetNumberOfBytesAvailable() < 20) {
      std::cout << "[hw::MotorDriver::GetMotorRpms] Incomplete packet: need 20 bytes, have "
                << shared_serial_port->GetNumberOfBytesAvailable() << std::endl;
      return Eigen::Vector4d();
    }

    // Read the 20-byte payload (16 bytes motors + 4 bytes gyro)
    shared_serial_port->Read(buffer, 20, 50);
    // Note: Read() returns void in this libserial version, so we can't check bytes_read

    // std::cout << "[hw::MotorDriver::GetMotorRpms] Successfully read 21-byte packet" << std::endl;
  }

  // Extract motor RPMs (first 16 bytes of payload)
  for (int i = 0; i < 4; ++i) {
    std::memcpy(&rpm[i], &buffer[i * 4], sizeof(int32_t));
  }

  // Extract gyro data (last 4 bytes of payload)
  std::memcpy(&gyro_mdeg_ps, &buffer[16], sizeof(int32_t));

  if (VerifyRpms(rpm)) {
    new_data_available = true;
    std::cout << "[hw::MotorDriver::GetMotorRpms] RPMS: " << rpm[0] << " " << rpm[1]
              << " " << rpm[2] << " " << rpm[3] << ", Gyro (mdeg/s): " << gyro_mdeg_ps
              << std::endl;
    return Eigen::Vector4d(rpm[0], rpm[1], rpm[2], rpm[3]);
  } else {
    std::cout << "[hw::MotorDriver::GetMotorRpms] INVALID RPMs: " << rpm[0] << " " << rpm[1] << " "
              << rpm[2] << " " << rpm[3] << std::endl;
    return Eigen::Vector4d(0, 0, 0, 0);
  }
}

double hw::MotorDriver::GetAngularVelocityRadps() {
  return (gyro_mdeg_ps / 1000.0) * M_PI / 180.0;
}

bool hw::MotorDriver::NewDataAvailable() { return new_data_available; }

bool hw::MotorDriver::VerifyRpms(std::vector<int> rpms) {
  for (int& rpm : rpms) {
    if (std::abs(rpm) > 400) return false;
  }
  return true;
}