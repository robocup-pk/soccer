#include "RobotModel.h"

#include <iostream>

kin::RobotModel::RobotModel() {
  robot_description = kin::GetRobotDescription();
  ComputeInverseMapping();
  ComputeForwardMapping();
}

kin::RobotModel::RobotModel(kin::RobotDescription& robot_dec) {
  robot_description = robot_dec;
  ComputeInverseMapping();
  ComputeForwardMapping();
}

Eigen::VectorXd kin::RobotModel::WheelSpeedsRadpsToRobotVelocity(
    const std::vector<double>& wheel_speeds_radps) {
  // Convert std::vector to Eigen::VectorXd
  Eigen::VectorXd wheel_speeds(wheel_speeds_radps.size());
  for (size_t i = 0; i < wheel_speeds_radps.size(); ++i) {
    wheel_speeds(i) = wheel_speeds_radps[i];
  }
  // Now use forward kinematics:
  return forward_mapping * wheel_speeds;
}

Eigen::VectorXd kin::RobotModel::WheelSpeedsRpmToRobotVelocity(
    const Eigen::Vector4d& wheel_speeds_rpm) {
  // rpm to radps
  Eigen::Vector4d wheel_speeds_radps = (2 * M_PI * wheel_speeds_rpm) / 60;

  return forward_mapping * wheel_speeds_radps;
}

Eigen::VectorXd kin::RobotModel::RobotVelocityToWheelSpeedsMps(
    const Eigen::Vector3d& robot_velocity_mps_radps) {
  // Inverse kinematics
  return inverse_mapping * robot_velocity_mps_radps;
}

Eigen::VectorXd kin::RobotModel::RobotVelocityToWheelSpeedsRpm(
    const Eigen::Vector3d& robot_velocity_mps_radps) {
  Eigen::Vector4d wheel_speeds_mps = RobotVelocityToWheelSpeedsMps(robot_velocity_mps_radps);
  Eigen::Vector4d wheel_speeds_radps = wheel_speeds_mps / robot_description.wheel_radius_m;
  return 60 * wheel_speeds_radps / (2 * M_PI);
}

Eigen::MatrixXd kin::RobotModel::InverseMapping() const { return inverse_mapping; }

Eigen::MatrixXd kin::RobotModel::ForwardMapping() const { return forward_mapping; }

void kin::RobotModel::ComputeInverseMapping() {
  size_t num_wheels = robot_description.wheel_positions_m.size();
  double r = robot_description.wheel_radius_m;

  // Initialize Jacobian matrix (num_wheels x 3)
  // This will be the complete inverse jacobian: ω = J * v
  inverse_mapping = Eigen::MatrixXd(num_wheels, 3);

  for (size_t i = 0; i < num_wheels; ++i) {
    double x_i = robot_description.wheel_positions_m[i].first;
    double y_i = robot_description.wheel_positions_m[i].second;
    double beta_i = robot_description.wheel_angles_rad[i];

    // Jacobian row: ω = (1/r) * [geometric_jacobian] * v
    // J_row = (1/r) * [cos(β_i), sin(β_i), x_i*sin(β_i) - y_i*cos(β_i)]
    inverse_mapping(i, 0) = (1.0 / r) * std::cos(beta_i);
    inverse_mapping(i, 1) = (1.0 / r) * std::sin(beta_i);
    inverse_mapping(i, 2) = (1.0 / r) * (x_i * std::sin(beta_i) - y_i * std::cos(beta_i));
  }
}

void kin::RobotModel::ComputeForwardMapping() {
  // Compute Moore-Penrose pseudoinverse: J+ = (J^T * J)^(-1) * J^T
  // This will give us the complete forward jacobian: v = J+ * ω
  Eigen::MatrixXd J_transpose = inverse_mapping.transpose();
  Eigen::MatrixXd JtJ = J_transpose * inverse_mapping;

  // Check if JtJ is invertible
  Eigen::FullPivLU<Eigen::MatrixXd> lu(JtJ);
  if (!lu.isInvertible()) {
    std::cerr << "Warning: Jacobian is not invertible. Singular configuration detected!"
              << std::endl;
    // Use pseudo-inverse with SVD for numerical stability
    forward_mapping = inverse_mapping.completeOrthogonalDecomposition().pseudoInverse();
  } else {
    // Standard Moore-Penrose pseudoinverse
    forward_mapping = JtJ.inverse() * J_transpose;
  }
}
