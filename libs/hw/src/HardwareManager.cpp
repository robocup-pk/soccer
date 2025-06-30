#include <iostream>

#include "HardwareManager.h"
#include "RobotModel.h"

hw::HardwareManager::HardwareManager(std::shared_ptr<kin::RobotModel> robot_model)
    : robot_model(robot_model) {
  motors = std::vector<hw::MotorModel>(kin::RobotDescription::num_wheels);
  // gyro = hw::GyroModel(gyro_port);

  current_wheel_speeds_rpm << 0, 0, 0, 0;
  current_encoder_ticks << 0, 0, 0, 0;

  hw_manager_running = true;
  sense_thread = std::thread(&HardwareManager::SenseLoop, this);
  control_thread = std::thread(&HardwareManager::ControlLoop, this);
}

void hw::HardwareManager::SenseLoop() {
  // Runs at 100 Hz
  while (hw_manager_running) {
    for (int i = 0; i < kin::RobotDescription::num_wheels; ++i) {
      current_encoder_ticks[i] = motors[i].GetTicks();
    }
    // w_radps = gyro.GetAngularVelocity();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void hw::HardwareManager::ControlLoop() {
  // Runs at 50 Hz
  while (hw_manager_running) {
    for (int i = 0; i < kin::RobotDescription::num_wheels; ++i) {
      motors[i].SetWheelSpeedRpm(current_wheel_speeds_rpm[i]);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

void hw::HardwareManager::SetBodyVelocity(Eigen::Vector3d& velocity_fBody) {
  Eigen::Vector4d wheel_speeds_rpm = robot_model->RobotVelocityToWheelSpeedsRpm(velocity_fBody);
  SetWheelSpeedsRpm(wheel_speeds_rpm);
}

void hw::HardwareManager::SetWheelSpeedsRpm(Eigen::Vector4d& wheel_speeds_rpm) {
  current_wheel_speeds_rpm = wheel_speeds_rpm;
}

Eigen::Vector4d hw::HardwareManager::GetEncoderTicks() { return current_encoder_ticks; }

// double hw::HardwareManager::GetGyroAngularVelocity() { return w_radps; }

hw::HardwareManager::~HardwareManager() {
  robot_model = nullptr;
  hw_manager_running = false;
  if (sense_thread.joinable()) sense_thread.join();
  if (control_thread.joinable()) control_thread.join();
}