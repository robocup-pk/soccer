#include <iostream>

#include "HardwareManager.h"
#include "RobotModel.h"
#include "RobotConstants.h"

hw::HardwareManager::HardwareManager() {
  std::cout << "[hw::HardwareManager::HardwareManager]" << std::endl;

  robot_model = std::make_shared<kin::RobotModel>();

  if (cfg::RobotConstants::shared_serial_port_name != "null") InitializeSerialPort();

  motor_driver = std::make_unique<MotorDriver>(shared_serial_port);
  // gyro_driver = std::make_unique<GyroDriver>(shared_serial_port);
}

void hw::HardwareManager::SetBodyVelocity(Eigen::Vector3d& velocity_fBody) {
  Eigen::Vector4d wheel_speeds_rpm = robot_model->RobotVelocityToWheelSpeedsRpm(velocity_fBody);
  SetWheelSpeedsRpm(wheel_speeds_rpm);
}

void hw::HardwareManager::SetWheelSpeedsRpm(Eigen::Vector4d& wheel_speeds_rpm) {
  motor_driver->SetWheelSpeedsRpm(wheel_speeds_rpm);
}

std::optional<Eigen::Vector4d> hw::HardwareManager::NewEncoderTicks() {
  // if (motor_driver->NewDataAvailable()) return motor_driver->GetEncoderTicks();
  // return std::nullopt;
  return motor_driver->GetEncoderTicks();
}

std::optional<double> hw::HardwareManager::NewGyroAngularVelocity() {
  // if (gyro_driver.NewDataAvailable()) return gyro_driver.GetAngularVelocity();
  return std::nullopt;
}

std::optional<Eigen::Vector3d> hw::HardwareManager::NewCameraData() {
  // if (camera_driver.NewDataAvailable()) return camera_driver.GetPose();
  return std::nullopt;
}

hw::HardwareManager::~HardwareManager() { robot_model = nullptr; }

void hw::HardwareManager::InitializeSerialPort() {
  shared_serial_port = std::make_shared<LibSerial::SerialPort>();
  shared_serial_port->Open(cfg::RobotConstants::shared_serial_port_name);
  shared_serial_port->SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  shared_serial_port->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
  shared_serial_port->SetParity(LibSerial::Parity::PARITY_NONE);
  shared_serial_port->SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  shared_serial_port->SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
  std::cout << "[hw::HardwareManager::InitializeSerialPort] Opened Serial Port: "
            << cfg::RobotConstants::shared_serial_port_name << std::endl;
}