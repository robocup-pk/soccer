#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H

#include <queue>
#include <mutex>
#include <atomic>
#include <vector>

#include "StateEstimator.h"
#include "HardwareManager.h"
#include "MotionController.h"
#include "AdaptiveTrajectory3D.h"
#include "RobotPositions.h"

namespace rob {

enum class RobotState {
  IDLE,
  DRIVING_TO_POINT,
  INTERPOLATING_TO_POINT,
  MANUAL_DRIVING,
  AUTONOMOUS_DRIVING,
  GOING_HOME,
  CALIBRATING
};

enum class RobotAction {
  KICK_BALL,
  PASS_BALL,
  MOVE  // Normal Action State
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
  RobotAction GetRobotAction();
  void SetRobotAction(RobotAction action);
  void NewCameraData(Eigen::Vector3d pose_from_camera);

  bool BodyVelocityIsInLimits(Eigen::Vector3d& velocity_fBody);

  Eigen::Vector3d GetPoseInWorldFrame() const;
  void InitializePose(Eigen::Vector3d& pose_fWorld);
  Eigen::Vector3d GetVelocityInWorldFrame() const;
  void TryAssignNextGoal();

  std::string GetRobotState();
  void CalibrateGyro();
  bool IsGyroCalibrated();

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
  ctrl::AdaptiveTrajectory trajectory_manager;

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
};
}  // namespace rob

#endif  // ROBOT_MANAGER_H