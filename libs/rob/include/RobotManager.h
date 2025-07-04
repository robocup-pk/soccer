#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H

#include <queue>
#include <mutex>

#include "HardwareManager.h"
#include "Estimator.h"
#include "MotionController.h"

#include <queue>

namespace rob {

enum class RobotState { IDLE, DRIVING_TO_POINT, MANUAL_DRIVING, AUTONOMOUS_DRIVING, GOING_HOME };

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

  Eigen::Vector3d GetPoseInWorldFrame();
  void TryAssignNextGoal();

  std::string GetRobotState();

 private:
  RobotState previous_robot_state;
  RobotState robot_state;

  Eigen::Vector3d pose_fWorld;
  Eigen::Vector3d velocity_fBody;

  hw::HardwareManager hardware_manager;
  est::Estimator estimator;
  ctrl::MotionController motion_controller;

  std::mutex robot_manager_threads_mutex;
  std::thread control_thread;
  std::thread sense_thread;

  bool rob_manager_running = false;
  double control_loop_frequency_hz = 50;
  double sense_loop_frequency_hz = 100;

  bool finished_motion;

  // For driving to point
  Eigen::Vector3d pose_destination;
  std::queue<Eigen::Vector3d> goal_queue;

  // Home position
  bool initialized_pose_home;
  Eigen::Vector3d pose_home_fWorld;

  // Idle
  double start_time_idle_s;
  double elapsed_time_idle_s;
  double sleep_time_while_idle_s;
};
}  // namespace rob

#endif  // ROBOT_MANAGER_H