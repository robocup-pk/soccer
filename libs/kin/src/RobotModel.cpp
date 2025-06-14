#include "RobotModel.h"

#include <iostream>

kin::RobotModel::RobotModel(const RobotDescription robot_description)
    : robot_description(robot_description) {
  ComputeInverseJacobian();
  ComputeForwardJacobian();
}

Eigen::VectorXd kin::RobotModel::WheelSpeedsToRobotVelocity(
    const std::vector<double>& wheel_speeds_radps) {
  // Convert std::vector to Eigen::VectorXd
  Eigen::VectorXd wheel_speeds(wheel_speeds_radps.size());
  for (size_t i = 0; i < wheel_speeds_radps.size(); ++i) {
    wheel_speeds(i) = wheel_speeds_radps[i];
  }
  // Now use forward kinematics:
  return forward_jacobian * wheel_speeds;
}

Eigen::VectorXd kin::RobotModel::RobotVelocityToWheelSpeeds(
    const Eigen::Vector3d& robot_velocity_mps_radps) {
  // Inverse kinematics
  return inverse_jacobian * robot_velocity_mps_radps;
}

Eigen::MatrixXd kin::RobotModel::InverseJacobian() const { return inverse_jacobian; }

Eigen::MatrixXd kin::RobotModel::ForwardJacobian() const { return forward_jacobian; }

void kin::RobotModel::ComputeInverseJacobian() {
  size_t num_wheels = robot_description.wheel_positions_m.size();
  double r = robot_description.wheel_radius_m;

  // Initialize Jacobian matrix (num_wheels x 3)
  // This will be the complete inverse jacobian: ω = J * v
  inverse_jacobian = Eigen::MatrixXd(num_wheels, 3);

  for (size_t i = 0; i < num_wheels; ++i) {
    double x_i = robot_description.wheel_positions_m[i].first;
    double y_i = robot_description.wheel_positions_m[i].second;
    double beta_i = robot_description.wheel_angles_rad[i];

    // Jacobian row: ω = (1/r) * [geometric_jacobian] * v
    // J_row = (1/r) * [cos(β_i), sin(β_i), x_i*sin(β_i) - y_i*cos(β_i)]
    inverse_jacobian(i, 0) = (1.0 / r) * std::cos(beta_i);
    inverse_jacobian(i, 1) = (1.0 / r) * std::sin(beta_i);
    inverse_jacobian(i, 2) = (1.0 / r) * (x_i * std::sin(beta_i) - y_i * std::cos(beta_i));
  }
}

void kin::RobotModel::ComputeForwardJacobian() {
  // Compute Moore-Penrose pseudoinverse: J+ = (J^T * J)^(-1) * J^T
  // This will give us the complete forward jacobian: v = J+ * ω
  Eigen::MatrixXd J_transpose = inverse_jacobian.transpose();
  Eigen::MatrixXd JtJ = J_transpose * inverse_jacobian;

  // Check if JtJ is invertible
  Eigen::FullPivLU<Eigen::MatrixXd> lu(JtJ);
  if (!lu.isInvertible()) {
    std::cerr << "Warning: Jacobian is not invertible. Singular configuration detected!"
              << std::endl;
    // Use pseudo-inverse with SVD for numerical stability
    forward_jacobian = inverse_jacobian.completeOrthogonalDecomposition().pseudoInverse();
  } else {
    // Standard Moore-Penrose pseudoinverse
    forward_jacobian = JtJ.inverse() * J_transpose;
  }
}
