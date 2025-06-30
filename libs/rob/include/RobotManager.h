#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H

#include "HardwareManager.h"
#include "Estimator.h"

namespace rob {
class RobotManager {
 public:
  RobotManager(std::shared_ptr<kin::RobotModel> robot_model);

  void ControlLoop();
  void SenseLoop();
  void ControlLogic();
  void SenseLogic();

  void SetBodyVelocity(Eigen::Vector3d& velocity_fBody);

  Eigen::Vector3d GetPose();

 private:
  Eigen::Vector3d pose;
  Eigen::Vector3d velocity_fBody;

  hw::HardwareManager hardware_manager;
  est::Estimator estimator;

  std::thread control_thread;
  std::thread sense_thread;

  bool rob_manager_running = false;
  double control_loop_frequency_hz = 50;
  double sense_loop_frequency_hz = 100;
};
}  // namespace rob

#endif  // ROBOT_MANAGER_H