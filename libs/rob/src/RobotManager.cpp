#include <iostream>
#include <tuple>

#include "SystemConfig.h"
#include "AdvancedMotionPlanner.h"
#include "RobotManager.h"
#include "Utils.h"
#include "Waypoint.h"
#include "RobotPositions.h"

rob::RobotManager::RobotManager() {
  std::cout << "[rob::RobotManager::RobotManager]" << std::endl;
  previous_robot_state = RobotState::IDLE;
  robot_state = RobotState::CALIBRATING;
  start_time_idle_s = util::GetCurrentTime();
  velocity_fBody << 0, 0, 0;
  home_position = cfg::RobotHomePosition::CENTER_FORWARD;
  InitializeHome(cfg::RightRobotHomeCoordinates.at(home_position));
  start_from_home = false;
  finished_motion = true;
  num_sensor_readings_failed = 0;
  rob_manager_running.store(true);
  trajectory_manager_type_ = TrajectoryManagerType::AdvancedTrajectory;

#ifdef BUILD_ON_PI
  state_estimator.initialized_pose = false;
#else
  state_estimator.initialized_pose = true;
#endif

  disable_gyro_checks = true;

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
      if (disable_gyro_checks || hardware_manager.IsGyroCalibrated()) {
        robot_state = RobotState::IDLE;
        std::cout << "[rob::RobotManager::ControlLogic] " 
                  << (disable_gyro_checks ? "Gyro checks disabled" : "Gyro is calibrated") 
                  << ". Going to IDLE state." << std::endl;
      }
      break;
    case RobotState::IDLE:
      velocity_fBody_ = velocity_fBody;
      break;
    case RobotState::MANUAL_DRIVING:
      velocity_fBody_ = velocity_fBody;
      break;
    case RobotState::TRAJECTORY_FOLLOWING:
      velocity_fBody_ = trajectory_tracker.update(pose_fWorld);
      finished_motion = trajectory_tracker.isFinished();
      break;
    case RobotState::REPLANNING_CONTROL:
      velocity_fBody_ = replanning_controller_.update(pose_fWorld, GetBodyVelocity());
      finished_motion = replanning_controller_.isDestinationReached();
      break;
    default:
      velocity_fBody_ = Eigen::Vector3d::Zero();
      break;
  }

  if (BodyVelocityIsInLimits(velocity_fBody_)) {
    SetBodyVelocity(velocity_fBody_);
  }

  if (finished_motion) {
    TryAssignNextGoal();
  }
}

void rob::RobotManager::SenseLogic() {
  pose_fWorld = state_estimator.GetPose();
}

void rob::RobotManager::SetBodyVelocity(Eigen::Vector3d& velocity_fBody_) {
  velocity_fBody = velocity_fBody_;
  hardware_manager.SetBodyVelocity(velocity_fBody_);
}

void rob::RobotManager::AddGoal(const Eigen::Vector3d& goal) {
  std::unique_lock<std::mutex> lock(goal_queue_mutex);
  goal_queue.push(goal);
}

void rob::RobotManager::SetBangBangPath(std::vector<Eigen::Vector3d> path, double t_start_s) {
  // Use planSmoothTrajectory with default constraints for now
  advanced_motion_planner.planSmoothTrajectory(path, 1.5, 2.0, 5.0, 10.0);
  trajectory_tracker.setTrajectory(std::make_shared<ctrl::AdvancedMotionPlanner>(advanced_motion_planner));
  robot_state = RobotState::TRAJECTORY_FOLLOWING;
  finished_motion = false;
}

void rob::RobotManager::SetAdvancedTrajectory(const ctrl::AdvancedMotionPlanner& advanced_planner) {
  advanced_motion_planner = advanced_planner;
  trajectory_tracker.setTrajectory(std::make_shared<ctrl::AdvancedMotionPlanner>(advanced_motion_planner));
  robot_state = RobotState::TRAJECTORY_FOLLOWING;
  finished_motion = false;
}

void rob::RobotManager::SetTrajectoryManagerType(TrajectoryManagerType type) {
  trajectory_manager_type_ = type;
}

void rob::RobotManager::SetReplanningGoal(const Eigen::Vector3d& goal) {
  replanning_controller_.setDestination(goal);
  robot_state = RobotState::REPLANNING_CONTROL;
  finished_motion = false;
}

void rob::RobotManager::SetReplanningEnabled(bool enabled) {
  replanning_controller_.setReplanningEnabled(enabled);
}

void rob::RobotManager::AddObstacles(const std::vector<std::shared_ptr<ctrl::IObstacle>>& obstacles) {
  replanning_controller_.setObstacles(obstacles);
}

void rob::RobotManager::ClearObstacles() {
  std::vector<std::shared_ptr<ctrl::IObstacle>> empty_obstacles;
  replanning_controller_.setObstacles(empty_obstacles);
}

ctrl::ReplanningController::ReplanningStats rob::RobotManager::GetReplanningStats() const {
  return replanning_controller_.getStats();
}

int rob::RobotManager::GetReplanCount() const {
  return replanning_controller_.getStats().total_replans;
}

void rob::RobotManager::GoHome() {
  if (!initialized_pose_home) {
    std::cerr << "[rob::RobotManager::GoHome] Home position not initialized!" << std::endl;
    return;
  }
  AddGoal(pose_home_fWorld);
}

void rob::RobotManager::InitializeHome(Eigen::Vector3d pose_home) {
  pose_home_fWorld = pose_home;
  initialized_pose_home = true;
}

bool rob::RobotManager::BodyVelocityIsInLimits(Eigen::Vector3d& velocity_fBody) {
  return velocity_fBody.norm() <= cfg::SystemConfig::max_velocity_fBody_mps.norm();
}

Eigen::Vector3d rob::RobotManager::GetPoseInWorldFrame() const {
  return pose_fWorld;
}

void rob::RobotManager::InitializePose(Eigen::Vector3d& pose_fWorld_) {
  pose_fWorld = pose_fWorld_;
  state_estimator.InitializePose(pose_fWorld_);
}

Eigen::Vector3d rob::RobotManager::GetVelocityInWorldFrame() const {
  return velocity_fBody; // This should be transformed to world frame in a real implementation
}

Eigen::Vector3d rob::RobotManager::GetBodyVelocity() const {
  return velocity_fBody;
}

Eigen::Vector3d rob::RobotManager::GetStateEstimationPose() const {
  return pose_fWorld; // Return cached pose since GetPose() is not const
}

void rob::RobotManager::TryAssignNextGoal() {
  std::unique_lock<std::mutex> lock(goal_queue_mutex);
  if (!goal_queue.empty()) {
    Eigen::Vector3d next_goal = goal_queue.front();
    goal_queue.pop();
    SetReplanningGoal(next_goal);
  } else {
    robot_state = RobotState::IDLE;
  }
}

void rob::RobotManager::SetStateEstimationNoise(double position_noise, double angle_noise) {
  state_estimation_position_noise_ = position_noise;
  state_estimation_angle_noise_ = angle_noise;
}

std::string rob::RobotManager::GetRobotState() {
  switch (robot_state) {
    case RobotState::IDLE: return "IDLE";
    case RobotState::DRIVING_TO_POINT: return "DRIVING_TO_POINT";
    case RobotState::INTERPOLATING_TO_POINT: return "INTERPOLATING_TO_POINT";
    case RobotState::MANUAL_DRIVING: return "MANUAL_DRIVING";
    case RobotState::AUTONOMOUS_DRIVING: return "AUTONOMOUS_DRIVING";
    case RobotState::CALIBRATING: return "CALIBRATING";
    case RobotState::TRAJECTORY_FOLLOWING: return "TRAJECTORY_FOLLOWING";
    case RobotState::REPLANNING_CONTROL: return "REPLANNING_CONTROL";
    default: return "UNKNOWN";
  }
}

rob::RobotAction rob::RobotManager::GetRobotAction() { 
  return robot_action; 
}

void rob::RobotManager::SetRobotAction(RobotAction action) { 
  robot_action = action; 
}

void rob::RobotManager::KickBall() {
  // Kick implementation
}

void rob::RobotManager::PassBall() {
  // Pass implementation  
}

void rob::RobotManager::CalibrateGyro() { 
  hardware_manager.CalibrateGyro(); 
}

bool rob::RobotManager::IsGyroCalibrated() {
  return hardware_manager.IsGyroCalibrated();
}

void rob::RobotManager::NewCameraData(Eigen::Vector3d pose_from_camera) {
  state_estimator.NewCameraData(pose_from_camera);
}

void rob::RobotManager::NewGyroData(double w_radps) {
  state_estimator.NewGyroData(w_radps);
}

void rob::RobotManager::NewMotorsData(const Eigen::Vector4d& motors_rpms) {
  state_estimator.NewMotorsData(motors_rpms);
}

rob::RobotManager::~RobotManager() {
  rob_manager_running.store(false);
  if (control_thread.joinable()) control_thread.join();
  if (sense_thread.joinable()) sense_thread.join();
}