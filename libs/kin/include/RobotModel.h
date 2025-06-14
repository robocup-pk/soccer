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
  RobotModel(const RobotDescription robot_description);

  // Compute robot velocity in body frame (x_dot_b, y_dot_b, theta_dot_b) from wheel speeds (for
  // simulating)
  Eigen::VectorXd WheelSpeedsToRobotVelocity(const std::vector<double>& wheel_speeds_radps);

  // Compute wheel speeds (radps) from robot velocity (for actuation)
  Eigen::VectorXd RobotVelocityToWheelSpeeds(const Eigen::Vector3d& robot_velocity_mps_radps);

  // Getters for jacobians
  Eigen::MatrixXd InverseJacobian() const;  // u = J * v
  Eigen::MatrixXd ForwardJacobian() const;  // v = J+ * u

 private:
  // Inverse Jacobian: u = J * v
  // Converts robot velocity (x_dot, y_dot, w_dot) in body frame to motor speeds (radps)
  void ComputeInverseJacobian();

  // Forward Jacobian: v = J+ * u
  // Converts motor speeds to robot velocity
  void ComputeForwardJacobian();

  Eigen::MatrixXd inverse_jacobian;
  Eigen::MatrixXd forward_jacobian;
  RobotDescription robot_description;
};

}  // namespace kin

#endif  // ROBOT_MODEL_H