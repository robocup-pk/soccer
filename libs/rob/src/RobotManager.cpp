#include <iostream>

#include "RobotManager.h"

rob::RobotManager::RobotManager() {
  velocity_fBody << 0, 0, 0;
  rob_manager_running = true;

  // TODO: remove when we have the camera system ready
  estimator.initialized_pose = true;

  sense_thread = std::thread(&RobotManager::SenseLoop, this);
  control_thread = std::thread(&RobotManager::ControlLoop, this);
}

void rob::RobotManager::SenseLoop() {
  auto time_step = std::chrono::steady_clock::now();
  int next_time_step_ms = 1000 / sense_loop_frequency_hz;

  while (rob_manager_running) {
    SenseLogic();
    time_step += std::chrono::milliseconds(next_time_step_ms);
    std::this_thread::sleep_until(time_step);
  }
}

void rob::RobotManager::ControlLoop() {
  auto time_step = std::chrono::steady_clock::now();
  int next_time_step_ms = 1000 / control_loop_frequency_hz;

  while (rob_manager_running) {
    ControlLogic();
    time_step += std::chrono::milliseconds(next_time_step_ms);
    std::this_thread::sleep_until(time_step);
  }
}

void rob::RobotManager::ControlLogic() {
  pose = estimator.GetPose();
  // More control
}

void rob::RobotManager::SenseLogic() {
  std::optional<Eigen::Vector4d> encoder_ticks = hardware_manager.NewEncoderTicks();
  std::optional<double> w_radps = hardware_manager.NewGyroAngularVelocity();
  std::optional<Eigen::Vector3d> pose_from_camera = hardware_manager.NewCameraData();

  if (encoder_ticks.has_value()) estimator.NewEncoderData(encoder_ticks.value());
  if (w_radps.has_value()) estimator.NewGyroData(w_radps.value());
  if (pose_from_camera.has_value()) estimator.NewCameraData(pose_from_camera.value());
}

void rob::RobotManager::SetBodyVelocity(Eigen::Vector3d& velocity_fBody) {
  hardware_manager.SetBodyVelocity(velocity_fBody);
}

Eigen::Vector3d rob::RobotManager::GetPose() { return pose; }