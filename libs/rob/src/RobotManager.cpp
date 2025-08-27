#include <iostream>
#include <tuple>

#include "SystemConfig.h"
#include "MotionController.h"
#include "RobotManager.h"
#include "Utils.h"
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
  trajectory_manager_type_ = TrajectoryManagerType::UniformBSpline;  // Default to uniform B-spline

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
    case RobotState::BSPLINE_DRIVING:
      std::tie(finished_motion, velocity_fBody_) = bspline_manager.Update(pose_fWorld);
      break;
    case RobotState::UNIFORM_BSPLINE_DRIVING:
      velocity_fBody_ = uniform_bspline_planner.Update(pose_fWorld, util::GetCurrentTime());
      finished_motion = uniform_bspline_planner.IsFinished();
      break;
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
  if (w_radps.has_value()) state_estimator.NewGyroData(w_radps.value());
  if (pose_from_camera.has_value()) state_estimator.NewCameraData(pose_from_camera.value());

  {
    // Pose shall not be used in control while it is being updated
    std::unique_lock<std::mutex> lock(robot_state_mutex);
    pose_fWorld = state_estimator.GetPose();
  }

  if (!disable_gyro_checks && !hardware_manager.IsGyroCalibrated()) {
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


void rob::RobotManager::SetBSplinePath(std::vector<Eigen::Vector3d> path_fWorld, double t_start_s) {
  bool is_path_valid;
  {
    // Print the path
    std::cout << "[rob::RobotManager::SetBSplinePath] B-spline trajectory for smooth waypoint following: ";
    for (int i = 0; i < path_fWorld.size() - 1; ++i) {
      std::cout << path_fWorld[i].transpose() << " -> ";
    }
    std::cout << path_fWorld[path_fWorld.size() - 1].transpose() << std::endl;

    // Initialize trajectory manager with current robot state
    pose_fWorld = state_estimator.GetPose();
    bspline_manager.InitializeFromRobotManager(this);
    
    // Create trajectories using B-spline Manager
    is_path_valid = bspline_manager.CreateTrajectoriesFromPath(path_fWorld, t_start_s);
    std::cout << "[rob::RobotManager::SetBSplinePath] Finish creating B-spline trajectories" << std::endl;
  }

  if (is_path_valid) {
    std::unique_lock<std::mutex> lock(robot_state_mutex);
    robot_state = RobotState::BSPLINE_DRIVING;
    trajectory_manager_type_ = TrajectoryManagerType::BSpline;
  } else {
    std::cout << "[rob::RobotManager::SetBSplinePath] Given path is invalid. Failed to create "
                 "B-spline trajectories\nPath: ";
  }
}

void rob::RobotManager::SetUniformBSplinePath(std::vector<Eigen::Vector3d> path_fWorld, double t_start_s) {
  bool is_path_valid;
  {
    // Print the path
    std::cout << "[rob::RobotManager::SetUniformBSplinePath] Uniform B-spline trajectory for smooth waypoint following: ";
    for (int i = 0; i < path_fWorld.size() - 1; ++i) {
      std::cout << path_fWorld[i].transpose() << " -> ";
    }
    std::cout << path_fWorld[path_fWorld.size() - 1].transpose() << std::endl;

    // Initialize trajectory manager with current robot state
    pose_fWorld = state_estimator.GetPose();
    uniform_bspline_planner.InitializeFromRobotManager(this);
    
    // Create trajectories using Uniform B-spline Planner
    is_path_valid = uniform_bspline_planner.SetPath(path_fWorld, t_start_s);
    std::cout << "[rob::RobotManager::SetUniformBSplinePath] Finish creating Uniform B-spline trajectories" << std::endl;
  }

  if (is_path_valid) {
    std::unique_lock<std::mutex> lock(robot_state_mutex);
    robot_state = RobotState::UNIFORM_BSPLINE_DRIVING;
    trajectory_manager_type_ = TrajectoryManagerType::UniformBSpline;
  } else {
    std::cout << "[rob::RobotManager::SetUniformBSplinePath] Given path is invalid. Failed to create "
                 "Uniform B-spline trajectories\nPath: ";
  }
}

void rob::RobotManager::SetBezierTrajectoryPath(std::vector<Eigen::Vector3d> path_fWorld, double t_start_s) {
  bool is_path_valid;
  {
    // Print the path
    std::cout << "[rob::RobotManager::SetBezierTrajectoryPath] Bezier trajectory for accurate waypoint following: ";
    for (int i = 0; i < path_fWorld.size() - 1; ++i) {
      std::cout << path_fWorld[i].transpose() << " -> ";
    }
    std::cout << path_fWorld[path_fWorld.size() - 1].transpose() << std::endl;
    // Initialize trajectory manager with current robot state
    pose_fWorld = state_estimator.GetPose();
    bezier_trajectory_planner.InitializeFromRobotManager(this);
    
    // Create trajectories using Bezier Trajectory Planner
    is_path_valid = bezier_trajectory_planner.SetPath(path_fWorld, t_start_s);
    std::cout << "[rob::RobotManager::SetBezierTrajectoryPath] Finish creating Bezier trajectories" << std::endl;
  }
  if (is_path_valid) {
    std::unique_lock<std::mutex> lock(robot_state_mutex);
    robot_state = RobotState::BEZIER_TRAJECTORY_DRIVING;
    trajectory_manager_type_ = TrajectoryManagerType::BezierTrajectory;
  } else {
    std::cout << "[rob::RobotManager::SetBezierTrajectoryPath] Given path is invalid. Failed to create "
                 "Bezier trajectories\nPath: ";
  }
}

void rob::RobotManager::SetBangBangPath(std::vector<Eigen::Vector3d> path_fWorld, double t_start_s) {
  bool is_path_valid;
  {
    // Print the path
    std::cout << "[rob::RobotManager::SetBangBangPath] Bang-bang trajectory for time-optimal motion: ";
    for (int i = 0; i < path_fWorld.size() - 1; ++i) {
      std::cout << path_fWorld[i].transpose() << " -> ";
    }
    std::cout << path_fWorld[path_fWorld.size() - 1].transpose() << std::endl;
    
    // Initialize trajectory planner with current robot state
    pose_fWorld = state_estimator.GetPose();
    
    // Set robot parameters
    bangbang_planner.SetRobotRadius(0.09);  // 90mm robot radius
    bangbang_planner.SetLimits(0.8, 2.0);   // 0.8 m/s max vel, 2.0 m/s^2 max acc (more reasonable)
    bangbang_planner.SetFeedbackGains(2.0, 0.5);  // Reduced gains for stability
    bangbang_planner.SetFieldBoundaries(-4.5, 4.5, -3.0, 3.0);  // SSL field size
    
    // Create trajectory using Bang-bang planner
    is_path_valid = bangbang_planner.SetPath(path_fWorld, t_start_s);
    std::cout << "[rob::RobotManager::SetBangBangPath] Finish creating Bang-bang trajectory" << std::endl;
  }
  
  if (is_path_valid) {
    std::unique_lock<std::mutex> lock(robot_state_mutex);
    robot_state = RobotState::BANGBANG_DRIVING;
    trajectory_manager_type_ = TrajectoryManagerType::BangBang;
  } else {
    std::cout << "[rob::RobotManager::SetBangBangPath] Failed to create Bang-bang trajectory" << std::endl;
  }
}

void rob::RobotManager::GoHome(){
  
}
void rob::RobotManager::SetDBRRTGoal(const Eigen::Vector3d& goal) {
  std::cout << "[rob::RobotManager::SetDBRRTGoal] DB-RRT planning to goal: " << goal.transpose() << std::endl;
  
  // Initialize planner
  pose_fWorld = state_estimator.GetPose();
  dbrrt_planner.InitializeFromRobotManager(this);
  
  // Plan trajectory from current pose to goal
  bool planning_success = dbrrt_planner.PlanTrajectory(pose_fWorld, goal, 1.0);
  
  if (planning_success) {
    std::unique_lock<std::mutex> lock(robot_state_mutex);
    robot_state = RobotState::DBRRT_DRIVING;
    trajectory_manager_type_ = TrajectoryManagerType::DBRRT;
    std::cout << "[rob::RobotManager::SetDBRRTGoal] Planning successful! Trajectory duration: " 
              << dbrrt_planner.GetTrajectoryDuration() << "s" << std::endl;
  } else {
    std::cout << "[rob::RobotManager::SetDBRRTGoal] Planning failed!" << std::endl;
  }
}

void rob::RobotManager::SetTrajectoryManagerType(TrajectoryManagerType type) {
  trajectory_manager_type_ = type;
  std::string type_name;
  switch (type) {
    case TrajectoryManagerType::BSpline:
      type_name = "B_SPLINE";
      break;
    case TrajectoryManagerType::UniformBSpline:
      type_name = "UNIFORM_B_SPLINE";
      break;
    case TrajectoryManagerType::BezierTrajectory:
      type_name = "BEZIER_TRAJECTORY";
      break;
    case TrajectoryManagerType::DBRRT:
      type_name = "DB_RRT";
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
    case RobotState::BSPLINE_DRIVING:
      return "BSPLINE_DRIVING";
    case RobotState::UNIFORM_BSPLINE_DRIVING:
      return "UNIFORM_BSPLINE_DRIVING";
    case RobotState::BEZIER_TRAJECTORY_DRIVING:
      return "BEZIER_TRAJECTORY_DRIVING";
    case RobotState::DBRRT_DRIVING:
      return "DBRRT_DRIVING";
    case RobotState::MANUAL_DRIVING:
      return "MANUAL_DRIVING";
    case RobotState::CALIBRATING:
      return "CALIBRATING";
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