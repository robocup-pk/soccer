#include <iostream>
#include <tuple>

#include "SystemConfig.h"
#include "MotionController.h"
#include "RobotManager.h"
#include "Utils.h"

rob::RobotManager::RobotManager() {
  previous_robot_state = RobotState::IDLE;
  robot_state = RobotState::IDLE;
  start_time_idle_s = util::GetCurrentTime();
  velocity_fBody << 0, 0, 0;
  initialized_pose_home = false;
  finished_motion = true;
  num_sensor_readings_failed = 0;
  rob_manager_running.store(true);

  // TODO: remove when we have the camera system ready
  estimator.initialized_pose = true;

  sense_thread = std::thread(&RobotManager::SenseLoop, this);
  control_thread = std::thread(&RobotManager::ControlLoop, this);
}

void rob::RobotManager::SenseLoop() {
  auto time_step = std::chrono::steady_clock::now();
  int next_time_step_ms = 1000 / sense_loop_frequency_hz;

  while (rob_manager_running.load()) {
    SenseLogic();
    time_step += std::chrono::milliseconds(next_time_step_ms);
    std::this_thread::sleep_until(time_step);
  }
}

void rob::RobotManager::ControlLoop() {
  auto time_step = std::chrono::steady_clock::now();
  int next_time_step_ms = 1000 / control_loop_frequency_hz;

  while (rob_manager_running.load()) {
    {
      std::unique_lock<std::mutex> lock(robot_state_mutex);
      ControlLogic();
    }
    time_step += std::chrono::milliseconds(next_time_step_ms);
    std::this_thread::sleep_until(time_step);
  }
}

void rob::RobotManager::ControlLogic() {
  Eigen::Vector3d velocity_fBody_;

  switch (robot_state) {
    case RobotState::IDLE:
      velocity_fBody_ = Eigen::Vector3d(0, 0, 0);
      break;
    case RobotState::DRIVING_TO_POINT:
      std::tie(finished_motion, velocity_fBody_) =
          motion_controller.DriveToPoint(pose_fWorld, pose_destination);
      TryAssignNextGoal();
      break;
    case RobotState::INTERPOLATING_TO_POINT:
      std::tie(finished_motion, velocity_fBody_) =
          motion_controller.InterpolateToPoint(pose_fWorld, pose_destination);
      std::cout << "Pose: " << std::fixed << std::setprecision(2) << pose_fWorld.transpose()
                << " and destination: " << pose_destination.transpose() << std::endl;
      TryAssignNextGoal();
      std::cout << "v_w: " << velocity_fBody_.transpose() << std::endl << std::endl;
      break;
    case RobotState::MANUAL_DRIVING:
      velocity_fBody_ = velocity_fBody;
      // finished_motion = true;
      break;
    case RobotState::GOING_HOME:
      std::tie(finished_motion, velocity_fBody_) =
          motion_controller.DriveToPoint(pose_fWorld, pose_home_fWorld);
      break;
    case RobotState::AUTONOMOUS_DRIVING:
      // TODO
      break;
  }

  if (finished_motion) robot_state = RobotState::IDLE;

  if (!BodyVelocityIsInLimits(velocity_fBody_)) {
    std::cout << "[rob::RobotManager::ControlLogic] Velocity is too high. Stopping the robot and "
                 "going to IDLE mode. velocity_fBody: "
              << velocity_fBody_.transpose() << std::endl;
    velocity_fBody_ = Eigen::Vector3d::Zero();
    robot_state = RobotState::IDLE;
  }

  hardware_manager.SetBodyVelocity(velocity_fBody_);
}

void rob::RobotManager::SenseLogic() {
  std::optional<Eigen::Vector4d> motors_rpms = hardware_manager.NewMotorsRpms();
  std::optional<double> w_radps = hardware_manager.NewGyroAngularVelocity();
  std::optional<Eigen::Vector3d> pose_from_camera = hardware_manager.NewCameraData();

  {
    if (motors_rpms.has_value()) estimator.NewMotorsData(motors_rpms.value());
    if (w_radps.has_value()) estimator.NewGyroData(w_radps.value());

    // TODO: This is not needed right now, but is useful for the physical robot
    // When both motors and gyro fail
    // if (!(motors_rpms.has_value() && w_radps.has_value())) {
    //   std::cout
    //       << "[rob::RobotManager::SenseLogic] No motors or gyro data. Please check the sensors"
    //       << std::endl;
    //   num_sensor_readings_failed++;
    //   if (num_sensor_readings_failed > 10) {
    //     std::cout << "[rob::RobotManager::SenseLogic] Error! Too many sensor readings failed. "
    //                  "Shutting down the robot manager"
    //               << std::endl;
    //     rob_manager_running.store(false);
    //     return;
    //   }
    // } else {
    //   num_sensor_readings_failed = 0;
    // }

    if (pose_from_camera.has_value()) estimator.NewCameraData(pose_from_camera.value());

    // Pose shall not be used in control while it is being updated
    std::unique_lock<std::mutex> lock(robot_state_mutex);
    pose_fWorld = estimator.GetPose();
  }
  // std::cout << "Pose (est): " << pose_fWorld.transpose() << std::endl;

  // Set home pose
  if (!initialized_pose_home && estimator.initialized_pose) {
    pose_home_fWorld = estimator.GetPoseInit();
    initialized_pose_home = true;
  }
}

void rob::RobotManager::SetPath(std::vector<Eigen::Vector3d> path) {
  for (int i = 0; i < path.size(); ++i) {
    AddGoal(path[i]);
  }
}

void rob::RobotManager::SetBodyVelocity(Eigen::Vector3d& velocity_fBody) {
  std::unique_lock<std::mutex> lock(robot_state_mutex);
  this->velocity_fBody = velocity_fBody;
  robot_state = RobotState::MANUAL_DRIVING;
}

Eigen::Vector3d rob::RobotManager::GetPoseInWorldFrame() { return pose_fWorld; }

void rob::RobotManager::AddGoal(const Eigen::Vector3d& goal) {
  {
    std::unique_lock<std::mutex> lock(goal_queue_mutex);
    if (goal_queue.size() > 20) {
      std::cout << "[rob::RobotManager::AddGoal] Error! Goal queue is too long. Size: "
                << goal_queue.size() << std::endl;
      return;
    }
    goal_queue.push(goal);
    std::cout << "[rob::RobotManager::AddGoal] Set Goal: " << goal.transpose() << std::endl;
  }
  std::unique_lock<std::mutex> lock(robot_state_mutex);
  if (robot_state == RobotState::IDLE) {
    pose_destination = goal_queue.front();
    goal_queue.pop();
    std::cout << "[rob::RobotManager::ControlLogic] Drive to point. Goal: "
              << pose_destination.transpose() << std::endl;

    // If RobotManager is running on PC, we can perform motion using interpolation
    // If on PI, we must use d2p
#ifdef BUILD_ON_PI
    robot_state = RobotState::DRIVING_TO_POINT;
#else
    robot_state = RobotState::INTERPOLATING_TO_POINT;
#endif
  }
}

void rob::RobotManager::TryAssignNextGoal() {
  if (finished_motion && !goal_queue.empty()) {
    std::unique_lock<std::mutex> lock(goal_queue_mutex);
    pose_destination = goal_queue.front();
    goal_queue.pop();
    finished_motion = false;
    std::cout << "[rob::RobotManager::AssignNextGoal] Drive to point. Goal: "
              << pose_destination.transpose() << std::endl;
  }
}

void rob::RobotManager::InitializeHome(Eigen::Vector3d pose_home) {
  pose_home_fWorld = pose_home;
  initialized_pose_home = true;
}

void rob::RobotManager::GoHome() {
  if (!initialized_pose_home) {
    std::cout << "[rob::RobotManager::GoHome] Can't go home. It is uninitialized" << std::endl;
    return;
  }
  std::unique_lock<std::mutex> lock(robot_state_mutex);
  robot_state = RobotState::GOING_HOME;
}

std::string rob::RobotManager::GetRobotState() {
  std::unique_lock<std::mutex> lock(robot_state_mutex);
  switch (robot_state) {
    case RobotState::IDLE:
      return "IDLE";
    case RobotState::DRIVING_TO_POINT:
      return "DRIVING_TO_POINT";
    case RobotState::MANUAL_DRIVING:
      return "MANUAL_DRIVING";
    case RobotState::AUTONOMOUS_DRIVING:
      return "AUTONOMOUS_DRIVING";
    case RobotState::GOING_HOME:
      return "GOING_HOME";
    case RobotState::INTERPOLATING_TO_POINT:
      return "INTERPOLATING_TO_POINT";
  }
  return "ERROR";
}

bool rob::RobotManager::BodyVelocityIsInLimits(Eigen::Vector3d& velocity_fBody) {
  for (int i = 0; i < 3; i++) {
    if (std::fabs(velocity_fBody[i]) > cfg::SystemConfig::max_velocity_fBody[i]) {
      return false;
    }
  }
  return true;
}

rob::RobotManager::~RobotManager() {
  rob_manager_running.store(false);
  if (sense_thread.joinable()) sense_thread.join();
  if (control_thread.joinable()) control_thread.join();
}