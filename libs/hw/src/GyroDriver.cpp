#include <iostream>
#include <thread>
#include <chrono>

#include <libserial/SerialPort.h>

#include "RobotModel.h"
#include "Utils.h"
#include "GyroDriver.h"

hw::GyroDriver::GyroDriver(std::shared_ptr<LibSerial::SerialPort> shared_serial_port)
    : shared_serial_port(shared_serial_port) {
  std::cout << "[hw::GyroDriver::GyroDriver]" << std::endl;
  gyro_wradps = 0;
}

void hw::GyroDriver::SetAngularVelocityRadps(double w_radps) {
  gyro.SetAngularVelocityRadps(w_radps);
}

double hw::GyroDriver::GetAngularVelocityRadps() {
  // if (gyro_type == GyroType::MODEL) {
  gyro_wradps = gyro.GetAngularVelocityRadps();
  new_data_available = true;
  // std::cout << "[hw::GyroDriver::GetMotorsRpms] MODEL RPMS: " << motors_rpms.transpose()
  //           << std::endl;
  return gyro_wradps;
  // }

  // TODO: Get gyro angular velocity using serial
}

bool hw::GyroDriver::NewDataAvailable() { return new_data_available; }