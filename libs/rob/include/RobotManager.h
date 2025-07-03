#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H

#include "HardwareManager.h"
#include "Estimator.h"

namespace rob {

enum class RobotState { IDLE, DRIVING_TO_POINT, MANUAL_DRIVING, AUTONOMOUS_DRIVING };

class RobotManager {
 public:
  RobotManager();

  void ControlLoop();
  void SenseLoop();
  void ControlLogic();
  void SenseLogic();

  void StartDrivingToPoint(Eigen::Vector3d pose_dest);
  void SetBodyVelocity(Eigen::Vector3d& velocity_fBody);

  Eigen::Vector3d GetPoseInWorldFrame();

 private:
  RobotState robot_state;

  Eigen::Vector3d pose_fWorld;
  Eigen::Vector3d velocity_fBody;

  hw::HardwareManager hardware_manager;
  est::Estimator estimator;
  ctrl::MotionController motion_controller;

  std::thread control_thread;
  std::thread sense_thread;

  bool rob_manager_running = false;
  double control_loop_frequency_hz = 50;
  double sense_loop_frequency_hz = 100;

  // For driving to point
  Eigen::Vector3d pose_destination;
};
}  // namespace rob

#endif  // ROBOT_MANAGER_H