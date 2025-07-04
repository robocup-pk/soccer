#include <iostream>

#include "MotionController.h"
#include "RobotManager.h"
#include "Utils.h"

rob::RobotManager::RobotManager() {
  previous_robot_state = RobotState::IDLE;
  robot_state = RobotState::IDLE;
  start_time_idle_s = util::GetCurrentTime();
  velocity_fBody << 0, 0, 0;
  initialized_pose_home = false;

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
  if (previous_robot_state == RobotState::IDLE && robot_state != RobotState::IDLE) {
    start_time_idle_s = util::GetCurrentTime();
  }

  bool finished_motion;
  Eigen::Vector3d velocity_fBody;

  switch (robot_state) {
    case RobotState::IDLE:
      elapsed_time_idle_s = util::GetCurrentTime() - start_time_idle_s;
      velocity_fBody = Eigen::Vector3d(0, 0, 0);
      break;
    case RobotState::DRIVING_TO_POINT:
      std::tie(finished_motion, velocity_fBody) =
          motion_controller.DriveToPoint(pose_fWorld, pose_destination);
      AssignNextGoal(finished_motion);
      break;
    case RobotState::MANUAL_DRIVING:
      finished_motion = true;
      break;
    case RobotState::GO_TO_HOME:
      std::tie(finished_motion, velocity_fBody) =
          motion_controller.DriveToPoint(pose_fWorld, pose_home_fWorld);
      break;
    case RobotState::AUTONOMOUS_DRIVING:
      // TODO
      break;
  }

  if (elapsed_time_idle_s > sleep_time_while_idle_s) return;
  if (finished_motion) robot_state = RobotState::IDLE;
  previous_robot_state = robot_state;

  hardware_manager.SetBodyVelocity(velocity_fBody);
}

void rob::RobotManager::SenseLogic() {
  std::optional<Eigen::Vector4d> encoder_ticks = hardware_manager.NewEncoderTicks();
  std::optional<double> w_radps = hardware_manager.NewGyroAngularVelocity();
  std::optional<Eigen::Vector3d> pose_from_camera = hardware_manager.NewCameraData();

  if (encoder_ticks.has_value()) estimator.NewEncoderData(encoder_ticks.value());
  if (w_radps.has_value()) estimator.NewGyroData(w_radps.value());
  if (pose_from_camera.has_value()) estimator.NewCameraData(pose_from_camera.value());

  pose_fWorld = estimator.GetPose();

  // Set home pose
  if (!initialized_pose_home && estimator.initialized_pose) {
    pose_home_fWorld = estimator.GetPoseInit();
    initialized_pose_home = true;
  }
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

void rob::RobotManager::AddGoal(const Eigen::Vector3d& goal) {
  goal_queue.push(goal);
  if (robot_state == RobotState::IDLE) {
    pose_destination = goal_queue.front();
    goal_queue.pop();
    std::cout << "[rob::RobotManager::ControlLogic] Drive to point. Goal: "
              << pose_destination.transpose() << std::endl;
    robot_state = RobotState::DRIVING_TO_POINT;
  }
}

void rob::RobotManager::AssignNextGoal(bool finished_motion) {
  if (finished_motion && !goal_queue.empty()) {
    pose_destination = goal_queue.front();
    goal_queue.pop();
    std::cout << "[rob::RobotManager::AssignNextGoal] Drive to point. Goal: "
              << pose_destination.transpose() << std::endl;
  }
}