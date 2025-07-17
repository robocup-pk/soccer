#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H

#include <queue>
#include <mutex>
#include <atomic>

#include "StateEstimator.h"
#include "HardwareManager.h"
#include "MotionController.h"
#include "TrajectoryManager.h"
#include "Utils.h"

namespace rob {

enum class RobotState {
  IDLE,
  DRIVING_TO_POINT,
  INTERPOLATING_TO_POINT,
  MANUAL_DRIVING,
  AUTONOMOUS_DRIVING,
  GOING_HOME
};

class RobotManager {
 public:
  RobotManager();

  void ControlLoop();
  void SenseLoop();
  void ControlLogic();
  void SenseLogic();

  // Used by the outside world
  void SetBodyVelocity(Eigen::Vector3d& velocity_fBody);
  void AddGoal(const Eigen::Vector3d& goal);
  void GoHome();
  void InitializeHome(Eigen::Vector3d pose_home);
  void SetPath(std::vector<Eigen::Vector3d> path, double t_start_s = util::GetCurrentTime());

  bool BodyVelocityIsInLimits(Eigen::Vector3d& velocity_fBody);

  Eigen::Vector3d GetPoseInWorldFrame();
  void TryAssignNextGoal();

  std::string GetRobotState();

  ~RobotManager();

 private:
  RobotState previous_robot_state;
  RobotState robot_state;

  Eigen::Vector3d pose_fWorld;
  Eigen::Vector3d velocity_fBody;

  est::StateEstimator state_estimator;
  hw::HardwareManager hardware_manager;
  ctrl::MotionController motion_controller;
  ctrl::TrajectoryManager trajectory_manager;

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
};
}  // namespace rob

#endif  // ROBOT_MANAGER_H