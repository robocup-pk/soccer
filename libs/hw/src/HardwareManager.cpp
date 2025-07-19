#include <iostream>

#include "HardwareManager.h"
#include "RobotModel.h"
#include "RobotConstants.h"

hw::HardwareManager::HardwareManager() {
  std::cout << "[hw::HardwareManager::HardwareManager]" << std::endl;

  robot_model = std::make_shared<kin::RobotModel>();

  if (cfg::RobotConstants::shared_serial_port_name != "null") InitializeSerialPort();

  sensor_driver = std::make_unique<SensorDriver>(shared_serial_port);
}

void hw::HardwareManager::SetBodyVelocity(Eigen::Vector3d velocity_fBody) {
  Eigen::Vector4d wheel_speeds_rpm = robot_model->RobotVelocityToWheelSpeedsRpm(velocity_fBody);

  std::cout << "[hw::HardwareManager::SetBodyVelocity] Body Velocity: "
            << velocity_fBody.transpose() << " m/s" << std::endl;

  std::cout << "[hw::HardwareManager::SetBodyVelocity] Wheel Speeds RPM: "
            << wheel_speeds_rpm.transpose() << std::endl;
  SetWheelSpeedsRpm(wheel_speeds_rpm);
  sensor_driver->SetAngularVelocityRadps(velocity_fBody[2]);
}

void hw::HardwareManager::SetWheelSpeedsRpm(Eigen::Vector4d& wheel_speeds_rpm) {
  sensor_driver->SendWheelSpeedRpm(wheel_speeds_rpm);
}

std::optional<Eigen::Vector4d> hw::HardwareManager::NewMotorsRpms() {
  auto [motor_rpms, gyro_data] = sensor_driver->GetSensorsData();

  sensor_driver->SetAngularVelocityRadps(gyro_data);

  if (sensor_driver->NewDataAvailable()) {
    sensor_driver->new_data_available = false;
    return motor_rpms;
  }
  return std::nullopt;
}

double hw::HardwareManager::ComputeGyroAngle(double angular_velocity_radps) {
  static double angle_rad = 0.0;
  static auto prev_time = std::chrono::steady_clock::now();

  auto now = std::chrono::steady_clock::now();
  std::chrono::duration<double> dt = now - prev_time;
  prev_time = now;

  angle_rad += angular_velocity_radps * dt.count();
  return angle_rad;
}

std::optional<double> hw::HardwareManager::NewGyroAngularVelocity() {
  double w_radps = sensor_driver->GetAngularVelocityRadps();
  std::cout << "[hw::HardwareManager::NewGyroAngularVelocity] Gyro Data: " << w_radps << std::endl;

  double angle = ComputeGyroAngle(w_radps) * 180.0 / M_PI;  // Convert to degrees

  std::cout << "[hw::HardwareManager::NewGyroAngularVelocity] Gyro Angle(Degree): " << angle
            << std::endl;
  return w_radps;
}

std::optional<Eigen::Vector3d> hw::HardwareManager::NewCameraData() {
  // if (camera_driver.NewDataAvailable()) return camera_driver.GetPose();
  return std::nullopt;
}

hw::HardwareManager::~HardwareManager() {
  // Properly close the serial port before destruction
  if (shared_serial_port && shared_serial_port->IsOpen()) {
    std::cout << "[hw::HardwareManager::~HardwareManager] Closing serial port..." << std::endl;
    shared_serial_port->Close();
  }
  robot_model = nullptr;
}

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