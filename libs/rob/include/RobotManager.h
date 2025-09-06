#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H

#include <queue>
#include <mutex>
#include <atomic>
#include <vector>
#include <random>

#include "StateEstimator.h"
#include "HardwareManager.h"
<<<<<<< HEAD
#include "MotionController.h"
<<<<<<< HEAD
#include "TrajectoryManager.h"
<<<<<<< HEAD
#include "RobotPositions.h"
=======
#include "M_TrajectoryController.h"
<<<<<<< HEAD
>>>>>>> 47173921 (New Trajactory Planner Manager with BangBang Trajactory Planner)
=======
#include "PurePursuitTrajectoryManager.h"
#include "HermiteSplineTrajectoryManager.h"
<<<<<<< HEAD
>>>>>>> 9a94022d (Add Sline Trajactory for RRTX WayPoints)
=======
#include "BSplineTrajectoryManager.h"
>>>>>>> 924fc1b6 (BSpline Trajactory for Path Planning)
=======
#include "BSplineTrajectoryManager.h"
#include "UniformBSplineTrajectoryPlanner.h"
<<<<<<< HEAD
>>>>>>> 97084d4f (Added Smooth uniform BSpline Trajectory Planner)
=======
#include "BezierTrajectoryPlanner.h"
<<<<<<< HEAD
>>>>>>> 05fb426c (Addded Bizzare Trjactory Planner)
=======
#include "DBRRTTrajectoryPlanner.h"
<<<<<<< HEAD
>>>>>>> 35d39075 (Added DBRT Planner)
=======
#include "BangBangTrajectoryPlanner.h"
>>>>>>> 08bf10ed (Fix Ideal position in Kalman Filter Logging)
=======
#include "AdvancedMotionPlanner.h"
#include "TrajectoryTracker.h"
>>>>>>> 5eb85243 (Tiger Manim BangBang2D Trajectory Planner)

// Forward declarations
namespace state {
  class SoccerObject;
}

namespace rob {

enum class RobotState {
  IDLE,
  DRIVING_TO_POINT,
  INTERPOLATING_TO_POINT,
  MANUAL_DRIVING,
  AUTONOMOUS_DRIVING,
  CALIBRATING,
  TRAJECTORY_FOLLOWING   // TIGERs-style trajectory following
};

enum class TrajectoryManagerType {
  TIGERsTrajectory // Use TIGERs-style AdvancedMotionPlanner + TrajectoryTracker
};

enum class RobotAction {
  KICK_BALL,
  PASS_BALL,
  DRIBBLE_BALL,
  MOVE,  // Default action when no specific action is set
};
class RobotManager {
 public:
  RobotManager();

  void ControlLoop();
  void SenseLoop();
  void ControlLogic();
  void SenseLogic();

  // Actions
  void KickBall();
  void PassBall();

  // Used by the outside world
  void SetBodyVelocity(Eigen::Vector3d& velocity_fBody);
  void AddGoal(const Eigen::Vector3d& goal);
  void GoHome();
  void InitializeHome(Eigen::Vector3d pose_home);
  void SetBSplinePath(std::vector<Eigen::Vector3d> path, double t_start_s = util::GetCurrentTime()); // B-spline path
  void SetUniformBSplinePath(std::vector<Eigen::Vector3d> path, double t_start_s = util::GetCurrentTime()); // Uniform B-spline path
  void SetBezierTrajectoryPath(std::vector<Eigen::Vector3d> path, double t_start_s = util::GetCurrentTime()); // Bezier trajectory path
  void SetBangBangPath(std::vector<Eigen::Vector3d> path, double t_start_s = util::GetCurrentTime()); // Bang-bang trajectory path
  void SetSmoothPathTrackerPath(std::vector<Eigen::Vector3d> path, double t_start_s = util::GetCurrentTime()); // Smooth path tracker with Pure Pursuit
  void SetDBRRTGoal(const Eigen::Vector3d& goal); // DB-RRT goal-based planning
  RobotAction GetRobotAction();
  void SetRobotAction(RobotAction action);
  
  // Trajectory manager selection
  void SetTrajectoryManagerType(TrajectoryManagerType type);
  TrajectoryManagerType GetTrajectoryManagerType() const { return trajectory_manager_type_; }
  
  // Get access to TIGERs trajectory system components
  ctrl::AdvancedMotionPlanner& GetAdvancedMotionPlanner() { return advanced_motion_planner; }
  ctrl::TrajectoryTracker& GetTrajectoryTracker() { return trajectory_tracker; }

  bool BodyVelocityIsInLimits(Eigen::Vector3d& velocity_fBody);

  Eigen::Vector3d GetPoseInWorldFrame() const;
  void InitializePose(Eigen::Vector3d& pose_fWorld);
  Eigen::Vector3d GetVelocityInWorldFrame() const;
  Eigen::Vector3d GetBodyVelocity() const;
  Eigen::Vector3d GetStateEstimationPose() const;
  void TryAssignNextGoal();
  
  // Set state estimation noise parameters (for simulation)
  void SetStateEstimationNoise(double position_noise, double angle_noise);
  
  // Get replanning statistics
  int GetReplanCount() const;

  std::string GetRobotState();
  void CalibrateGyro();
  bool IsGyroCalibrated();
  void NewCameraData(Eigen::Vector3d pose_from_camera);
  void NewGyroData(double w_radps);
  void NewMotorsData(const Eigen::Vector4d& motors_rpms);

  ~RobotManager();

 protected:  // Changed from private to protected for demo inheritance
  RobotState previous_robot_state;
  RobotState robot_state;

  RobotAction robot_action;

  Eigen::Vector3d pose_fWorld;
  Eigen::Vector3d velocity_fBody;

  est::StateEstimator state_estimator;
  hw::HardwareManager hardware_manager;
  ctrl::AdvancedMotionPlanner advanced_motion_planner;  // TIGERs-style advanced motion planner
  ctrl::TrajectoryTracker trajectory_tracker;  // TIGERs-style PID trajectory tracker
  
  TrajectoryManagerType trajectory_manager_type_;

  std::thread control_thread;
  std::thread sense_thread;
  std::mutex robot_state_mutex;

  std::atomic<bool> rob_manager_running = false;
  double control_loop_frequency_hz = 50;
  double sense_loop_frequency_hz = 100;

  bool finished_motion;

  // For driving to point
  Eigen::Vector3d pose_destination;
  std::queue<Eigen::Vector3d> goal_queue;
  std::mutex goal_queue_mutex;

  // Home position
  bool initialized_pose_home;
  Eigen::Vector3d pose_home_fWorld;
  cfg::RobotHomePosition home_position;
  bool start_from_home;

  // Idle
  double start_time_idle_s;
  double elapsed_time_idle_s;
  double sleep_time_while_idle_s;

  // Error cases
  int num_sensor_readings_failed;

  // Flag to disable gyro functionality when not connected (for demo/simulation mode)
  bool disable_gyro_checks;
  
  // State estimation simulation parameters
  double state_estimation_position_noise_ = 0.02;  // 2cm standard deviation
  double state_estimation_angle_noise_ = 0.05;     // 0.05 rad standard deviation
  mutable std::mt19937 rng_{std::random_device{}()};  // Random number generator
};
}  // namespace rob

#endif  // ROBOT_MANAGER_H