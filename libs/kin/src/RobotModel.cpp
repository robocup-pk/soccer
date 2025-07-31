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

Eigen::Vector3d kin::RobotModel::WheelSpeedsRpmToRobotVelocity(
    const Eigen::Vector4d& wheel_speeds_rpm) {
  // Hard conversion from RPM to rad/s: rad/s = RPM * 2*π / 60
  Eigen::Vector4d wheel_speeds_radps;
  wheel_speeds_radps[0] = wheel_speeds_rpm[0] * 2.0 * M_PI / 60.0;
  wheel_speeds_radps[1] = wheel_speeds_rpm[1] * 2.0 * M_PI / 60.0;
  wheel_speeds_radps[2] = wheel_speeds_rpm[2] * 2.0 * M_PI / 60.0;
  wheel_speeds_radps[3] = wheel_speeds_rpm[3] * 2.0 * M_PI / 60.0;

  Eigen::VectorXd result = forward_mapping * wheel_speeds_radps;
  return result.head<3>();
}

Eigen::VectorXd kin::RobotModel::RobotVelocityToWheelSpeedsRps(
    const Eigen::Vector3d& robot_velocity_mps) {
  // Inverse kinematics
  return inverse_mapping * robot_velocity_mps;
}

Eigen::Vector4d kin::RobotModel::RobotVelocityToWheelSpeedsRpm(
    const Eigen::Vector3d& robot_velocity_mps) {
  Eigen::Vector4d wheel_speeds_radps = RobotVelocityToWheelSpeedsRps(robot_velocity_mps).head<4>();

  // Hard conversion from rad/s to RPM: RPM = rad/s * 60 / (2*π)
  Eigen::Vector4d wheel_speeds_rpm;
  wheel_speeds_rpm[0] = wheel_speeds_radps[0] * 60.0 / (2.0 * M_PI);
  wheel_speeds_rpm[1] = wheel_speeds_radps[1] * 60.0 / (2.0 * M_PI);
  wheel_speeds_rpm[2] = wheel_speeds_radps[2] * 60.0 / (2.0 * M_PI);
  wheel_speeds_rpm[3] = wheel_speeds_radps[3] * 60.0 / (2.0 * M_PI);

  return wheel_speeds_rpm;
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
