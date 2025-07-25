#include <iostream>
#include <tuple>

#include "SystemConfig.h"
#include "MotionController.h"
#include "RobotManager.h"
#include "Utils.h"
#include "Trajectory3D.h"
#include "Waypoint.h"

rob::RobotManager::RobotManager() {
  std::cout << "[rob::RobotManager::RobotManager]" << std::endl;
  previous_robot_state = RobotState::IDLE;
  robot_state = RobotState::CALIBRATING;
  start_time_idle_s = util::GetCurrentTime();
  velocity_fBody << 0, 0, 0;
  initialized_pose_home = false;
  finished_motion = true;
  num_sensor_readings_failed = 0;
  rob_manager_running.store(true);

#ifdef BUILD_ON_PI
  state_estimator.initialized_pose = false;
#else
  state_estimator.initialized_pose = true;
#endif

  sense_thread = std::thread(&RobotManager::SenseLoop, this);
  control_thread = std::thread(&RobotManager::ControlLoop, this);
}

void rob::RobotManager::SenseLoop() {
  const std::chrono::microseconds time_step_us(1000000 / (int)sense_loop_frequency_hz);
  auto next_time = std::chrono::steady_clock::now();
  while (rob_manager_running.load()) {
    next_time += time_step_us;
    SenseLogic();
    std::this_thread::sleep_until(next_time);
  }
}

void rob::RobotManager::ControlLoop() {
  const std::chrono::microseconds time_step_us(1000000 / (int)control_loop_frequency_hz);
  auto next_time = std::chrono::steady_clock::now();
  while (rob_manager_running.load()) {
    next_time += time_step_us;
    {
      std::unique_lock<std::mutex> lock(robot_state_mutex);
      ControlLogic();
    }
    std::this_thread::sleep_until(next_time);
  }
}

void rob::RobotManager::ControlLogic() {
  Eigen::Vector3d velocity_fBody_;

  switch (robot_state) {
    case RobotState::CALIBRATING:
      velocity_fBody_ = Eigen::Vector3d::Zero();
      if (hardware_manager.IsGyroCalibrated()) {
        robot_state = RobotState::IDLE;
        std::cout << "[rob::RobotManager::ControlLogic] Gyro is calibrated. Going to IDLE state."
                  << std::endl;
      }
      break;
    case RobotState::IDLE:
      velocity_fBody_ = velocity_fBody;
      break;
    case RobotState::DRIVING_TO_POINT:
      std::tie(finished_motion, velocity_fBody_) =
          motion_controller.DriveToPoint(pose_fWorld, pose_destination);
      TryAssignNextGoal();
      break;
    case RobotState::INTERPOLATING_TO_POINT:
      std::tie(finished_motion, velocity_fBody_) =
          motion_controller.InterpolateToPoint(pose_fWorld, pose_destination);
      TryAssignNextGoal();
      break;
    case RobotState::MANUAL_DRIVING:
      velocity_fBody_ = velocity_fBody;
      // finished_motion = true;
      break;
    case RobotState::GOING_HOME:
      std::tie(finished_motion, velocity_fBody_) = trajectory_manager.Update(pose_fWorld);
      // std::tie(finished_motion, velocity_fBody_) =
      //     motion_controller.DriveToPoint(pose_fWorld, pose_home_fWorld);
      break;
    case RobotState::AUTONOMOUS_DRIVING:
      std::tie(finished_motion, velocity_fBody_) = trajectory_manager.Update(pose_fWorld);
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
  if (motors_rpms.has_value()) state_estimator.NewMotorsData(motors_rpms.value());
  if (w_radps.has_value()) state_estimator.NewGyroData(w_radps.value());
  if (pose_from_camera.has_value()) state_estimator.NewCameraData(pose_from_camera.value());

  {
    // Pose shall not be used in control while it is being updated
    std::unique_lock<std::mutex> lock(robot_state_mutex);
    pose_fWorld = state_estimator.GetPose();
  }

  if (!hardware_manager.IsGyroCalibrated()) {
    std::cout << "[rob::RobotManager::SenseLogic] Gyro is not calibrated. Waiting for calibration."
              << std::endl;
    robot_state = RobotState::CALIBRATING;
    return;
  }

  // Print pose every few seconds
  // static int num = 0;
  // ++num;
  // if (num % 200 == 0)
  //   std::cout << "[rob::RobotManager::SenseLogic] Pose (est): " << pose_fWorld.transpose()
  //             << std::endl;

  // Set home pose
  if (!initialized_pose_home && state_estimator.initialized_pose) {
    pose_home_fWorld = state_estimator.GetPoseInit();
    initialized_pose_home = true;
  }
}

void rob::RobotManager::SetPath(std::vector<Eigen::Vector3d> path_fWorld, double t_start_s) {
  bool is_path_valid;
  {
    // Print the path
    std::cout << "[rob::RobotManager::SetPath] ";
    for (int i = 0; i < path_fWorld.size() - 1; ++i) {
      std::cout << path_fWorld[i].transpose() << " -> ";
    }
    std::cout << path_fWorld[path_fWorld.size() - 1].transpose() << std::endl;

    // Create trajectories if valid
    is_path_valid = trajectory_manager.CreateTrajectoriesFromPath(path_fWorld, t_start_s);
    std::cout << "[rob::RobotManager::SetPath] Finish creating trajectories. t_finish_s: "
              << trajectory_manager.active_traj_t_finish_s << "\n\n";
  }

  if (is_path_valid) {
    std::unique_lock<std::mutex> lock(robot_state_mutex);
    robot_state = RobotState::AUTONOMOUS_DRIVING;
  } else {
    std::cout << "[rob::RobotManager::SetPath] Give path is invalid. Failed to create "
                 "trajectories\nPath: ";
  }
}

void rob::RobotManager::SetBodyVelocity(Eigen::Vector3d& velocity_fBody) {
  std::unique_lock<std::mutex> lock(robot_state_mutex);
  this->velocity_fBody = velocity_fBody;
  robot_state = RobotState::IDLE;
}

Eigen::Vector3d rob::RobotManager::GetPoseInWorldFrame() const { return pose_fWorld; }
Eigen::Vector3d rob::RobotManager::GetVelocityInWorldFrame() const {
  return util::RotateAboutZ(this->velocity_fBody, -pose_fWorld[2]);
}

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
  std::vector<Eigen::Vector3d> path;
  path.push_back(pose_fWorld);
  path.push_back(pose_home_fWorld);
  bool is_path_valid = trajectory_manager.CreateTrajectoriesFromPath(path);
  if (!is_path_valid) {
    std::cout << "[rob::RobotManager::GoHome] Can't go home. Path invalid\n";
  }
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
    if (std::fabs(velocity_fBody[i]) > cfg::SystemConfig::max_velocity_fBody_mps[i]) {
      return false;
    }
  }
  return true;
}

void rob::RobotManager::InitializePose(Eigen::Vector3d& pose_fWorld) {
  std::unique_lock<std::mutex> lock(robot_state_mutex);
  state_estimator.InitializePose(pose_fWorld);
}

rob::RobotAction rob::RobotManager::GetRobotAction() { return robot_action; }

void rob::RobotManager::SetRobotAction(RobotAction action) { robot_action = action; }

rob::RobotManager::~RobotManager() {
  rob_manager_running.store(false);
  if (sense_thread.joinable()) sense_thread.join();
  if (control_thread.joinable()) control_thread.join();
}

void rob::RobotManager::NewCameraData(Eigen::Vector3d pose_from_camera) {
  hardware_manager.NewCameraData(pose_from_camera);
}

void rob::RobotManager::CalibrateGyro() { hardware_manager.CalibrateGyro(); }

bool rob::RobotManager::IsGyroCalibrated() {
  if (!hardware_manager.IsGyroCalibrated()) {
    std::cout << "[rob::RobotManager::IsGyroCalibrated] Gyro is not calibrated." << std::endl;
    return false;
  }
  return true;
}