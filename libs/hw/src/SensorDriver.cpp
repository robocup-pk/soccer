#include <iostream>
#include <thread>
#include <chrono>
#include <libserial/SerialPort.h>

#include "SensorModel.h"
#include "SensorDriver.h"
#include "Utils.h"
#include "Kinematics.h"
#include "RobotDescription.h"

hw::SensorDriver::SensorDriver(std::shared_ptr<LibSerial::SerialPort> shared_serial_port)
    : shared_serial_port(shared_serial_port) {
  std::cout << "[hw::SensorDriver::SensorDriver]" << std::endl;
  gyro_wradps = 0;
  motors_rpms << 0, 0, 0, 0;
  motors = std::vector<SensorModel>(kin::RobotDescription::num_wheels);
}

void hw::SensorDriver::SetAngularVelocityRadps(double w_radps) {
  gyro.SetAngularVelocityRadps(w_radps);
}

double hw::SensorDriver::GetAngularVelocityRadps() {
  if (sensor_type == SensorType::MODEL) {
    gyro_wradps = gyro.GetAngularVelocityRadps();
    new_data_available = true;
    return gyro_wradps;
  }
  return (gyro_mdeg_ps / 1000.0) * M_PI / 180.0;
}

void hw::SensorDriver::SendWheelSpeedRpm(Eigen::Vector4d& wheel_speeds_rpm) {
  if (sensor_type == SensorType::MODEL) {
    for (int i = 0; i < kin::RobotDescription::num_wheels; ++i) {
      motors[i].SetWheelSpeedRpm(wheel_speeds_rpm[i]);
    }
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
  std::cout << "[hw::SensorDriver::SendWheelSpeedRpm] " << speeds_rpm[0] << " " << speeds_rpm[1]
            << " " << speeds_rpm[2] << " " << speeds_rpm[3] << std::endl;

  std::unique_lock<std::mutex> lock(shared_serial_port_mutex);  // 5 ms
  shared_serial_port->FlushOutputBuffer();
  util::WaitMs(2);
  shared_serial_port->Write(buffer);
  util::WaitMs(2);
  shared_serial_port->DrainWriteBuffer();
  util::WaitMs(1);
}

std::pair<Eigen::Vector4d, int> hw::SensorDriver::GetSensorsData() {
  if (sensor_type == SensorType::MODEL) {
    for (int i = 0; i < kin::RobotDescription::num_wheels; ++i) {
      motors_rpms[i] = motors[i].GetRpm();
    }
    new_data_available = true;
    return {motors_rpms, ((gyro_mdeg_ps / 1000.0) * M_PI / 180.0)};
  }

  std::vector<int> rpm(4, 0);
  std::vector<uint8_t> buffer(20);

  {
    std::unique_lock<std::mutex> lock(shared_serial_port_mutex);
    size_t available_bytes = shared_serial_port->GetNumberOfBytesAvailable();

    if (available_bytes < 21) {
      std::cout << "[hw::SensorDriver::GetSensorsData] Not enough data available: "
                << available_bytes << " bytes, need at least 21" << std::endl;
      std::cout << "[hw::SensorDriver::GetSensorsData] Available Bytes: " << available_bytes
                << std::endl;
      return {Eigen::Vector4d(), 0};
    }

    uint8_t byte;
    bool found_header = false;
    size_t max_search = std::min(available_bytes, size_t(100));  // Limit search

    for (size_t i = 0; i < max_search; ++i) {
      if (shared_serial_port->GetNumberOfBytesAvailable() < 1) break;

      shared_serial_port->ReadByte(byte, 10);
      if (byte == 'x') {
        found_header = true;
        std::cout << "[hw::SensorDriver::GetSensorsData] Found header 'x'" << std::endl;
        break;
      }
      std::cout << "[SYNC] Discarded byte: '" << (char)byte << "' (0x" << std::hex << (int)byte
                << std::dec << ")" << std::endl;
    }

    if (!found_header) {
      std::cout << "[hw::SensorDriver::GetSensorsData] No header found, flushing buffer"
                << std::endl;
      shared_serial_port->FlushInputBuffer();
      return {Eigen::Vector4d(), 0};
    }

    if (shared_serial_port->GetNumberOfBytesAvailable() < 20) {
      std::cout << "[hw::SensorDriver::GetSensorsData] Incomplete packet: need 20 bytes, have "
                << shared_serial_port->GetNumberOfBytesAvailable() << std::endl;
      return {Eigen::Vector4d(), 0};
    }

    shared_serial_port->Read(buffer, 20, 50);
  }

  for (int i = 0; i < 4; ++i) {
    std::memcpy(&rpm[i], &buffer[i * 4], sizeof(int32_t));
  }

  std::memcpy(&gyro_mdeg_ps, &buffer[16], sizeof(int32_t));
  if (VerifyRpms(rpm)) {
    new_data_available = true;
    std::cout << "[hw::SensorDriver::GetSensorsData] RPMS: " << rpm[0] << " " << rpm[1] << " "
              << rpm[2] << " " << rpm[3] << ", Gyro (mdeg/s): " << gyro_mdeg_ps << std::endl;
    return {Eigen::Vector4d(rpm[0], rpm[1], rpm[2], rpm[3]), gyro_mdeg_ps};
  } else {
    std::cout << "[hw::SensorDriver::GetSensorsData] INVALID RPMs: " << rpm[0] << " " << rpm[1]
              << " " << rpm[2] << " " << rpm[3] << std::endl;
    return {Eigen::Vector4d(0, 0, 0, 0), 0};
  }
}

bool hw::SensorDriver::NewDataAvailable() { return new_data_available; }

bool hw::SensorDriver::VerifyRpms(std::vector<int> rpms) {
  for (int& rpm : rpms) {
    if (std::abs(rpm) > 400) return false;
  }
  return true;
}