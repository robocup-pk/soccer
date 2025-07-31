#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H

#include <queue>
#include <mutex>
#include <atomic>
#include <vector>

#include "StateEstimator.h"
#include "HardwareManager.h"
#include "MotionController.h"
#include "TrajectoryManager.h"
#include "M_TrajectoryController.h"
#include "PurePursuitTrajectoryManager.h"
#include "HermiteSplineTrajectoryManager.h"
#include "BSplineTrajectoryManager.h"

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
  GOING_HOME,
  CALIBRATING,
  M_AUTONOMOUS_DRIVING,  // New state for M_TrajectoryManager
  PURE_PURSUIT_DRIVING,  // New state for Pure Pursuit following
  HERMITE_SPLINE_DRIVING, // New state for Hermite Spline trajectory following
  BSPLINE_DRIVING        // New state for B-spline trajectory following
};

enum class TrajectoryManagerType {
  ORIGINAL,      // Use original TrajectoryManager
  BangBang,      // Use M_TrajectoryManager (BangBang-based)
  PurePursuit,   // Use Pure Pursuit for multi-waypoint following
  HermiteSpline, // Use Cubic Hermite Spline for RRT* waypoints
  BSpline        // Use B-spline for smoother trajectories
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
  void SetPath(std::vector<Eigen::Vector3d> path, double t_start_s = util::GetCurrentTime());
  void SetMPath(std::vector<Eigen::Vector3d> path, double t_start_s = util::GetCurrentTime()); // Paper-based path
  void SetPurePursuitPath(std::vector<Eigen::Vector3d> path, double t_start_s = util::GetCurrentTime()); // Pure Pursuit path
  void SetHermiteSplinePath(std::vector<Eigen::Vector3d> path, double t_start_s = util::GetCurrentTime()); // Hermite Spline path
  void SetBSplinePath(std::vector<Eigen::Vector3d> path, double t_start_s = util::GetCurrentTime()); // B-spline path
  RobotAction GetRobotAction();
  void SetRobotAction(RobotAction action);
  
  // Trajectory manager selection
  void SetTrajectoryManagerType(TrajectoryManagerType type);
  TrajectoryManagerType GetTrajectoryManagerType() const { return trajectory_manager_type_; }

  bool BodyVelocityIsInLimits(Eigen::Vector3d& velocity_fBody);

  Eigen::Vector3d GetPoseInWorldFrame() const;
  void InitializePose(Eigen::Vector3d& pose_fWorld);
  Eigen::Vector3d GetVelocityInWorldFrame() const;
  void TryAssignNextGoal();

  std::string GetRobotState();
  void CalibrateGyro();
  bool IsGyroCalibrated();
  void NewCameraData(Eigen::Vector3d pose_from_camera);

  ~RobotManager();

 private:
  RobotState previous_robot_state;
  RobotState robot_state;

  RobotAction robot_action;

  Eigen::Vector3d pose_fWorld;
  Eigen::Vector3d velocity_fBody;

  est::StateEstimator state_estimator;
  hw::HardwareManager hardware_manager;
  ctrl::MotionController motion_controller;
  ctrl::TrajectoryManager trajectory_manager;
  ctrl::M_TrajectoryManager m_trajectory_manager;  // Paper-based trajectory manager
  ctrl::PurePursuitTrajectoryManager pure_pursuit_manager;  // Pure Pursuit trajectory manager
  ctrl::HermiteSplineTrajectoryManager hermite_spline_manager;  // Hermite Spline trajectory manager
  ctrl::BSplineTrajectoryManager bspline_manager;  // B-spline trajectory manager
  
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

  // Idle
  double start_time_idle_s;
  double elapsed_time_idle_s;
  double sleep_time_while_idle_s;

  // Error cases
  int num_sensor_readings_failed;

  // Flag to disable gyro functionality when not connected (for demo/simulation mode)
  bool disable_gyro_checks;
};
}  // namespace rob

#endif  // ROBOT_MANAGER_H