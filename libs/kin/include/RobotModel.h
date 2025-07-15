#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include <vector>

#include <Eigen/Dense>

#include "RobotDescription.h"

namespace kin {

/*
  This struct describes the robot kinematic model.
*/
class RobotModel {
 public:
  RobotModel();
  RobotModel(RobotDescription& robot_desc);

  // Compute robot velocity in body frame (x_dot_b, y_dot_b, theta_dot_b) from wheel speeds (for
  // simulating)
  Eigen::VectorXd WheelSpeedsRadpsToRobotVelocity(const std::vector<double>& wheel_speeds_radps);
  Eigen::VectorXd WheelSpeedsRpmToRobotVelocity(
      const Eigen::Vector4d& wheel_speeds_rpm);

  // Compute wheel speeds (radps) from robot velocity (for actuation)
  Eigen::VectorXd RobotVelocityToWheelSpeedsRps(const Eigen::Vector3d& robot_velocity_mps_radps);
  Eigen::VectorXd RobotVelocityToWheelSpeedsRpm(const Eigen::Vector3d& robot_velocity_mps_radps);

  // Getters for jacobians
  Eigen::MatrixXd InverseMapping() const;  // u = J * v
  Eigen::MatrixXd ForwardMapping() const;  // v = J+ * u

  RobotDescription robot_description;

 private:
  // Inverse mapping: u = J * v
  // Converts robot velocity (x_dot, y_dot, w_dot) in body frame to motor speeds (radps)
  void ComputeInverseMapping();

  // Forward mapping: v = J+ * u
  // Converts motor speeds to robot velocity
  void ComputeForwardMapping();

  Eigen::MatrixXd inverse_mapping;
  Eigen::MatrixXd forward_mapping;
};

}  // namespace kin

#endif  // ROBOT_MODEL_H