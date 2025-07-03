#include <iostream>

#include "MotionController.h"
#include "RobotManager.h"

rob::RobotManager::RobotManager() {
  robot_state = RobotState::IDLE;
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
  switch (robot_state) {
    case RobotState::IDLE:
      hardware_manager.SetBodyVelocity(Eigen::Vector3d(0, 0, 0));
      break;
    case RobotState::DRIVING_TO_POINT:
      velocity_fBody = motion_controller.DriveToPoint(pose_fWorld, pose_destination);
      hardware_manager.SetBodyVelocity(velocity_fBody);
      if (velocity_fBody.norm() == 0) robot_state = RobotState::IDLE;
      break;
    case RobotState::MANUAL_DRIVING:
      hardware_manager.SetBodyVelocity(velocity_fBody);
      break;
    case RobotState::AUTONOMOUS_DRIVING:
      // TODO
      break;
  }
}

void rob::RobotManager::SenseLogic() {
  std::optional<Eigen::Vector4d> encoder_ticks = hardware_manager.NewEncoderTicks();
  std::optional<double> w_radps = hardware_manager.NewGyroAngularVelocity();
  std::optional<Eigen::Vector3d> pose_from_camera = hardware_manager.NewCameraData();

  if (encoder_ticks.has_value()) estimator.NewEncoderData(encoder_ticks.value());
  if (w_radps.has_value()) estimator.NewGyroData(w_radps.value());
  if (pose_from_camera.has_value()) estimator.NewCameraData(pose_from_camera.value());

  pose_fWorld = estimator.GetPose();
}

void rob::RobotManager::SetBodyVelocity(Eigen::Vector3d& velocity_fBody) {
  this->velocity_fBody = velocity_fBody;
  robot_state = RobotState::MANUAL_DRIVING;
}

Eigen::Vector3d rob::RobotManager::GetPoseInWorldFrame() { return pose_fWorld; }

void rob::RobotManager::StartDrivingToPoint(Eigen::Vector3d pose_dest) {
  pose_destination = pose_dest;
  robot_state = RobotState::DRIVING_TO_POINT;
}