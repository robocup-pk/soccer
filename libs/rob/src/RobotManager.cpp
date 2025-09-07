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
  trajectory_manager_type_ = TrajectoryManagerType::AdvancedTrajectory;  // Default to advanced trajectory system

#ifdef BUILD_ON_PI
  state_estimator.initialized_pose = false;
#else
  state_estimator.initialized_pose = true;
#endif

  // Disable gyro functionality when not connected (for demo/simulation mode)
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
      // finished_motion = true;
      break;
<<<<<<< HEAD
<<<<<<< HEAD
    case RobotState::GOING_HOME:
      // std::tie(finished_motion, velocity_fBody_) = trajectory_manager.Update(pose_fWorld);
      std::tie(finished_motion, velocity_fBody_) = motion_controller.DriveToPoint(pose_fWorld, pose_home_fWorld);
      break;
    case RobotState::AUTONOMOUS_DRIVING:
      std::tie(finished_motion, velocity_fBody_) = trajectory_manager.Update(pose_fWorld);
      break;
    case RobotState::M_AUTONOMOUS_DRIVING:
      std::tie(finished_motion, velocity_fBody_) = m_trajectory_manager.Update(pose_fWorld);
      break;
    case RobotState::PURE_PURSUIT_DRIVING:
      std::tie(finished_motion, velocity_fBody_) = pure_pursuit_manager.Update(pose_fWorld);
      break;
    case RobotState::HERMITE_SPLINE_DRIVING:
      std::tie(finished_motion, velocity_fBody_) = hermite_spline_manager.Update(pose_fWorld);
      break;
=======
>>>>>>> 97084d4f (Added Smooth uniform BSpline Trajectory Planner)
    case RobotState::BSPLINE_DRIVING:
      std::tie(finished_motion, velocity_fBody_) = bspline_manager.Update(pose_fWorld);
      break;
    case RobotState::UNIFORM_BSPLINE_DRIVING: {
      velocity_fBody_ = uniform_bspline_planner.Update(pose_fWorld, util::GetCurrentTime());
      finished_motion = uniform_bspline_planner.IsFinished();
      
      // EWOK-style soft trajectory correction
      // This only makes small adjustments to control points, avoiding discontinuities
      if (uniform_bspline_planner.IsReplanningEnabled() && state_estimator.initialized_pose) {
        const double now = util::GetCurrentTime();
        const Eigen::Vector3d est_pose = state_estimator.GetPose();
        // TryAutoReplan now uses UpdatePartialTrajectory for soft corrections
        (void)uniform_bspline_planner.TryAutoReplan(est_pose, now, 0.03, 0.08);  // 3cm, 0.08rad thresholds (more sensitive)
      }
      
      break;
    }
    case RobotState::BEZIER_TRAJECTORY_DRIVING:
      velocity_fBody_ = bezier_trajectory_planner.Update(pose_fWorld, util::GetCurrentTime());
      finished_motion = bezier_trajectory_planner.IsFinished();
      break;
    case RobotState::DBRRT_DRIVING:
      velocity_fBody_ = dbrrt_planner.Update(pose_fWorld, util::GetCurrentTime());
      finished_motion = !dbrrt_planner.IsTrajectoryValid();
      break;
    case RobotState::BANGBANG_DRIVING:
      velocity_fBody_ = bangbang_planner.Update(pose_fWorld, util::GetCurrentTime());
      finished_motion = bangbang_planner.IsFinished();
=======
    case RobotState::TRAJECTORY_FOLLOWING:
      velocity_fBody_ = trajectory_tracker.update(pose_fWorld);
      finished_motion = trajectory_tracker.isFinished();
>>>>>>> 5eb85243 (Tiger Manim BangBang2D Trajectory Planner)
      break;
    case RobotState::REPLANNING_CONTROL:
      // Advanced-style replanning with dynamic obstacle avoidance
      velocity_fBody_ = replanning_controller_.update(pose_fWorld, this->GetVelocityInWorldFrame());
      finished_motion = replanning_controller_.isDestinationReached();
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

  // Store the computed velocity for external access
  velocity_fBody = velocity_fBody_;
  hardware_manager.SetBodyVelocity(velocity_fBody_);
}

void rob::RobotManager::SenseLogic() {
  std::optional<Eigen::Vector4d> motors_rpms = hardware_manager.NewMotorsRpms();
  std::optional<double> w_radps = hardware_manager.NewGyroAngularVelocity();
   
  std::optional<Eigen::Vector3d> pose_from_camera = hardware_manager.NewCameraData();
  if (motors_rpms.has_value()) state_estimator.NewMotorsData(motors_rpms.value());
  if (w_radps.has_value() && hardware_manager.IsGyroCalibrated()) state_estimator.NewGyroData(w_radps.value());
  if (pose_from_camera.has_value()) state_estimator.NewCameraData(pose_from_camera.value());

  {
    // Pose shall not be used in control while it is being updated
    std::unique_lock<std::mutex> lock(robot_state_mutex);
    pose_fWorld = state_estimator.GetPose();
  }

<<<<<<< HEAD
   if (!hardware_manager.IsGyroCalibrated()) {
=======
  if (!disable_gyro_checks && !hardware_manager.IsGyroCalibrated()) {
>>>>>>> 26be03f3 (Temp Mode for Now)
    std::cout << "[rob::RobotManager::SenseLogic] Gyro is not calibrated. Waiting for calibration."
              << std::endl;
    robot_state = RobotState::CALIBRATING;
    return;
  }

  if (state_estimator.initialized_pose && initialized_pose_home && !start_from_home) {
    if ((pose_fWorld - pose_home_fWorld).norm() > 0.05) {
      std::cout
          << "[rob::RobotManager::SenseLogic] Robot is not at home, Going to Home from Pose: "
          << pose_fWorld.transpose() << std::endl;
      robot_state = RobotState::GOING_HOME;
    } else {
      start_from_home = true;
      std::cout << "[rob::RobotManager::SenseLogic] Robot is at home, starting from home."
                << std::endl;
    }
  }
}






void rob::RobotManager::GoHome(){
  
}

void rob::RobotManager::SetSmoothPathTrackerPath(std::vector<Eigen::Vector3d> path_fWorld, double t_start_s) {
  if (path_fWorld.size() < 2) {
    std::cout << "[rob::RobotManager::SetSmoothPathTrackerPath] Error: Need at least 2 waypoints" << std::endl;
    return;
  }
  
  std::cout << "[rob::RobotManager::SetSmoothPathTrackerPath] Creating REAL Team smooth trajectory using TrajPath with " 
            << path_fWorld.size() << " waypoints" << std::endl;
  
  // EXACT Advanced approach: Use complete PathFinder system with MoveConstraints
  ctrl::MoveConstraints moveConstraints;
  moveConstraints.setVelMax(1.0)        // m/s - matches Advanced's default
                 .setAccMax(0.8)        // m/s² - matches Advanced's default  
                 .setVelMaxW(3.0)       // rad/s - matches Advanced's default
                 .setAccMaxW(2.5);      // rad/s² - matches Advanced's default
  
  if (path_fWorld.size() == 2) {
    // Single destination: Use PathFinder system
    std::vector<std::shared_ptr<ctrl::IObstacle>> obstacles; // Empty for now
    
    advanced_motion_planner.planTrajectory(
      this->GetPoseInWorldFrame(),      // Current robot position
      this->GetVelocityInWorldFrame(),  // Current robot velocity
      path_fWorld.back(),               // Final destination
      obstacles,                        // Obstacles (empty for now)
      moveConstraints                   // Advanced-style constraints
    );
  } else {
    // Multiple waypoints: Use backward-compatible method
    double max_vel = moveConstraints.getVelMax();
    double max_acc = moveConstraints.getAccMax();
    double max_omega = moveConstraints.getVelMaxW();
    double max_omega_acc = moveConstraints.getAccMaxW();
    
    advanced_motion_planner.planSmoothTrajectory(path_fWorld, max_vel, max_acc, max_omega, max_omega_acc);
  }
  
  if (advanced_motion_planner.isValid()) {
    // Set the smooth TrajPath trajectory for the TrajectoryTracker
    trajectory_tracker.setTrajectory(std::make_shared<ctrl::AdvancedMotionPlanner>(advanced_motion_planner));
    
    std::unique_lock<std::mutex> lock(robot_state_mutex);
    robot_state = RobotState::TRAJECTORY_FOLLOWING;
    trajectory_manager_type_ = TrajectoryManagerType::AdvancedTrajectory;
    
    std::cout << "[rob::RobotManager::SetSmoothPathTrackerPath] Successfully created smooth advanced trajectory! Duration: " 
              << advanced_motion_planner.getTotalTime() << "s" << std::endl;
  } else {
    std::cout << "[rob::RobotManager::SetSmoothPathTrackerPath] Failed to create smooth trajectory" << std::endl;
  }
}

void rob::RobotManager::SetAdvancedTrajectory(const ctrl::AdvancedMotionPlanner& advanced_planner) {
  std::cout << "[rob::RobotManager::SetAdvancedTrajectory] Directly setting advanced trajectory..." << std::endl;
  
  // Copy the advanced planner (it already has the complete trajectory)
  advanced_motion_planner = advanced_planner;
  
  if (advanced_motion_planner.isValid()) {
    // Set the advanced trajectory for the TrajectoryTracker
    trajectory_tracker.setTrajectory(std::make_shared<ctrl::AdvancedMotionPlanner>(advanced_motion_planner));
    
    std::unique_lock<std::mutex> lock(robot_state_mutex);
    robot_state = RobotState::TRAJECTORY_FOLLOWING;
    trajectory_manager_type_ = TrajectoryManagerType::AdvancedTrajectory;
    
    std::cout << "[rob::RobotManager::SetAdvancedTrajectory] Successfully set advanced trajectory! Duration: " 
              << advanced_motion_planner.getTotalTime() << "s" << std::endl;
  } else {
    std::cout << "[rob::RobotManager::SetAdvancedTrajectory] Invalid advanced trajectory!" << std::endl;
  }
}


void rob::RobotManager::SetTrajectoryManagerType(TrajectoryManagerType type) {
  trajectory_manager_type_ = type;
  std::string type_name;
  switch (type) {
    case TrajectoryManagerType::AdvancedTrajectory:
      type_name = "ADVANCED_TRAJECTORY";
      break;
  }
  std::cout << "[rob::RobotManager::SetTrajectoryManagerType] Set to " << type_name << std::endl;
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

Eigen::Vector3d rob::RobotManager::GetBodyVelocity() const {
  return velocity_fBody;
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

// void rob::RobotManager::GoHome() {
//   if (!initialized_pose_home) {
//     std::cout << "[rob::RobotManager::GoHome] Can't go home. It is uninitialized" << std::endl;
//     return;
//   }
//   std::unique_lock<std::mutex> lock(robot_state_mutex);
//   std::vector<Eigen::Vector3d> path;
//   path.push_back(pose_fWorld);
//   path.push_back(pose_home_fWorld);
//   //bool is_path_valid = trajectory_manager.CreateTrajectoriesFromPath(path);
//   if (!is_path_valid) {
//     std::cout << "[rob::RobotManager::GoHome] Can't go home. Path invalid\n";
//   }
//   robot_state = RobotState::GOING_HOME;
// }

std::string rob::RobotManager::GetRobotState() {
  std::unique_lock<std::mutex> lock(robot_state_mutex);
  switch (robot_state) {
    case RobotState::IDLE:
      return "IDLE";
<<<<<<< HEAD
<<<<<<< HEAD
    case RobotState::CALIBRATING:
      return "CALIBRATING";
    case RobotState::DRIVING_TO_POINT:
      return "DRIVING_TO_POINT";
=======
    case RobotState::BSPLINE_DRIVING:
      return "BSPLINE_DRIVING";
    case RobotState::UNIFORM_BSPLINE_DRIVING:
      return "UNIFORM_BSPLINE_DRIVING";
<<<<<<< HEAD
>>>>>>> 97084d4f (Added Smooth uniform BSpline Trajectory Planner)
=======
    case RobotState::BEZIER_TRAJECTORY_DRIVING:
      return "BEZIER_TRAJECTORY_DRIVING";
<<<<<<< HEAD
>>>>>>> 05fb426c (Addded Bizzare Trjactory Planner)
=======
    case RobotState::DBRRT_DRIVING:
      return "DBRRT_DRIVING";
>>>>>>> 35d39075 (Added DBRT Planner)
=======
    case RobotState::DRIVING_TO_POINT:
      return "DRIVING_TO_POINT";
    case RobotState::INTERPOLATING_TO_POINT:
      return "INTERPOLATING_TO_POINT";
>>>>>>> 5eb85243 (Tiger Manim BangBang2D Trajectory Planner)
    case RobotState::MANUAL_DRIVING:
      return "MANUAL_DRIVING";
    case RobotState::AUTONOMOUS_DRIVING:
      return "AUTONOMOUS_DRIVING";
    case RobotState::CALIBRATING:
      return "CALIBRATING";
    case RobotState::TRAJECTORY_FOLLOWING:
      return "TRAJECTORY_FOLLOWING";
    case RobotState::REPLANNING_CONTROL:
      return "REPLANNING_CONTROL";
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
  state_estimator.SetPose(pose_fWorld);  // Also set current pose
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

// Replanning controller methods (Advanced-style)
void rob::RobotManager::SetReplanningGoal(const Eigen::Vector3d& goal) {
  std::unique_lock<std::mutex> lock(robot_state_mutex);
  
  replanning_controller_.setDestination(goal);
  
  // Create movement constraints from system config
  ctrl::MoveConstraints constraints;
  constraints.setVelMax(1.0)        // m/s
             .setAccMax(0.8)        // m/s²
             .setVelMaxW(3.0)       // rad/s
             .setAccMaxW(2.5);      // rad/s²
  
  replanning_controller_.setMoveConstraints(constraints);
  
  robot_state = RobotState::REPLANNING_CONTROL;
  
  std::cout << "[rob::RobotManager::SetReplanningGoal] Set replanning goal: " 
            << goal.transpose() << std::endl;
}

void rob::RobotManager::SetReplanningEnabled(bool enabled) {
  replanning_controller_.setReplanningEnabled(enabled);
  std::cout << "[rob::RobotManager::SetReplanningEnabled] Replanning " 
            << (enabled ? "enabled" : "disabled") << std::endl;
}

void rob::RobotManager::AddObstacles(const std::vector<std::shared_ptr<ctrl::IObstacle>>& obstacles) {
  replanning_controller_.setObstacles(obstacles);
  std::cout << "[rob::RobotManager::AddObstacles] Added " << obstacles.size() 
            << " obstacles to replanning controller" << std::endl;
}

void rob::RobotManager::ClearObstacles() {
  std::vector<std::shared_ptr<ctrl::IObstacle>> empty_obstacles;
  replanning_controller_.setObstacles(empty_obstacles);
  std::cout << "[rob::RobotManager::ClearObstacles] Cleared all obstacles" << std::endl;
}

ctrl::ReplanningController::ReplanningStats rob::RobotManager::GetReplanningStats() const {
  return replanning_controller_.getStats();
}

int rob::RobotManager::GetReplanCount() const {
  return replanning_controller_.getStats().total_replans;
}
