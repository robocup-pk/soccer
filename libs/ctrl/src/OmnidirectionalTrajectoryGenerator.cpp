#include "OmnidirectionalTrajectoryGenerator.h"
#include "SystemConfig.h"
#include "RobotDescription.h"
#include "RobotModel.h"
#include "Utils.h"

#include <iostream>
#include <cmath>

ctrl::OmnidirectionalTrajectoryGenerator::OmnidirectionalTrajectoryGenerator() {
  // Initialize with your system's parameters
  setVehicleParameters(mu, m, J, h);

  // Initialize wheel configuration based on your robot description
  initializeWheelConfiguration();

  // Initialize robot model for kinematics
  robot_model = std::make_shared<kin::RobotModel>();

  // Very slow limits for testing and visibility
  double v_max = 0.5;     // Very slow max velocity for testing
  double a_max = 0.2;     // Very slow acceleration for testing
  a_max_y = 0.2;          // Very slow acceleration for testing
  theta_dot_max = 1.0;    // Very slow angular velocity for testing
  theta_ddot_max = 0.5;   // Very slow angular acceleration for testing

  // Calculate envelope and velocity limits
  calculateEllipticalEnvelope();
  calculateVelocityLimits();

  std::cout << "[OmnidirectionalTrajectoryGenerator] Initialized with limits:" << std::endl;
  std::cout << "  Max velocity: " << v_max << " m/s" << std::endl;
  std::cout << "  Max acceleration: (" << a_max_x << ", " << a_max_y << ", " << theta_dot_max
            << ")" << std::endl;
}

void ctrl::OmnidirectionalTrajectoryGenerator::setVehicleParameters(double friction_coeff,
                                                                    double mass, double inertia,
                                                                    double cm_height) {
  mu = friction_coeff;
  m = mass;
  J = inertia;
  h = cm_height;

  // Recalculate envelope
  calculateEllipticalEnvelope();
}

void ctrl::OmnidirectionalTrajectoryGenerator::setMotorLimits(double max_speed_radps,
                                                              double max_acc, double min_speed) {
  max_wheel_speed = max_speed_radps;
  max_wheel_acceleration = max_acc;
  min_wheel_speed = min_speed;

  // Recalculate velocity limits based on motor constraints
  calculateVelocityLimits();
}

void ctrl::OmnidirectionalTrajectoryGenerator::setVelocityLimits(double max_velocity) {
  v_max = max_velocity;
}

void ctrl::OmnidirectionalTrajectoryGenerator::setTolerances(double pos_tol, double angle_tol) {
  position_tolerance = pos_tol;
  angle_tolerance = angle_tol;
}

void ctrl::OmnidirectionalTrajectoryGenerator::initializeWheelConfiguration() {
  wheels.clear();

  // Get robot description from your system
  const auto& robot_desc = kin::GetRobotDescription();

  for (size_t i = 0; i < robot_desc.wheel_positions_m.size(); ++i) {
    WheelConfig wheel;

    // Get position from robot description
    double x = robot_desc.wheel_positions_m[i].first;
    double y = robot_desc.wheel_positions_m[i].second;
    double angle = robot_desc.wheel_angles_rad[i];

    // Normalize position vector
    double mag = std::sqrt(x * x + y * y);
    if (mag > 1e-6) {
      wheel.px = x / mag;
      wheel.py = y / mag;
    } else {
      wheel.px = 0.0;
      wheel.py = 0.0;
    }

    // Force direction from wheel angle (direction wheel can apply force)
    wheel.fx = std::cos(angle);
    wheel.fy = std::sin(angle);

    wheel.angle = angle;

    wheels.push_back(wheel);
  }

  std::cout << "[OmnidirectionalTrajectoryGenerator] Initialized " << wheels.size() << " wheels"
            << std::endl;
  verifyWheelAngles();
}

void ctrl::OmnidirectionalTrajectoryGenerator::verifyWheelAngles() {
  if (wheels.size() < 4) return;

  // Print wheel configuration for verification
  std::cout << "Wheel configuration verification:" << std::endl;
  for (size_t i = 0; i < wheels.size(); ++i) {
    std::cout << "  Wheel " << (i + 1) << ": pos=(" << wheels[i].px << ", " << wheels[i].py
              << "), force=(" << wheels[i].fx << ", " << wheels[i].fy
              << "), angle=" << wheels[i].angle * 180 / M_PI << "°" << std::endl;
  }
}

void ctrl::OmnidirectionalTrajectoryGenerator::calculateEllipticalEnvelope() {
  // Build the constraint matrix A from wheel configurations
  std::vector<std::vector<double>> A(wheels.size(), std::vector<double>(3));
  std::vector<double> b(wheels.size());

  for (size_t i = 0; i < wheels.size(); ++i) {
    // A[i] = [f_i^T, P_i × f_i] where × is cross product in 2D
    A[i][0] = wheels[i].fx;                                               // f_ix
    A[i][1] = wheels[i].fy;                                               // f_iy
    A[i][2] = wheels[i].px * wheels[i].fy - wheels[i].py * wheels[i].fx;  // P_i × f_i

    // Maximum force from wheel (based on motor torque and wheel radius)
    double wheel_radius = kin::RobotDescription::wheel_radius_m;
    b[i] = max_wheel_acceleration * wheel_radius;  // Convert to force
  }

  // Calculate maximum accelerations in each direction
  double a_x_max = 0, a_y_max = 0, alpha_max = 0;

  for (size_t i = 0; i < wheels.size(); ++i) {
    a_x_max += std::abs(A[i][0]) * b[i] / m;
    a_y_max += std::abs(A[i][1]) * b[i] / m;
    alpha_max += std::abs(A[i][2]) * b[i] / J;
  }

  // Use conservative scaling since we have proper velocity limits
  a_x_max *= 0.8;    // Mild scaling to respect 0.5 m/s² limit
  a_y_max *= 0.8;    // Mild scaling to respect 0.5 m/s² limit
  alpha_max *= 0.8;  // Mild scaling for angular acceleration

  // Set envelope parameters (limited by friction and system constraints)
  envelope_a = std::min({a_x_max, mu * g, a_max_x});
  envelope_b = std::min({a_y_max, mu * g, a_max_y});
  theta_dot_max = std::min({alpha_max, theta_dot_max});

  // Update maximum accelerations
  a_max_x = envelope_a;
  a_max_y = envelope_b;

  std::cout << "Elliptical envelope: a=" << envelope_a << ", b=" << envelope_b
            << ", alpha_max=" << theta_dot_max << std::endl;
}

void ctrl::OmnidirectionalTrajectoryGenerator::calculateVelocityLimits() {
  // Use the system's configured maximum velocity
  double system_max_vel = cfg::SystemConfig::max_velocity_fBody_mps.minCoeff();
  v_max = std::min(v_max, system_max_vel);

  std::cout << "Maximum velocity set to: " << v_max << " m/s" << std::endl;
}

bool ctrl::OmnidirectionalTrajectoryGenerator::isGoalReached(const State& current,
                                                             const State& goal) {
  double dx = current.x - goal.x;
  double dy = current.y - goal.y;
  double dtheta = normalizeAngle(current.theta - goal.theta);

  return (std::sqrt(dx * dx + dy * dy) < position_tolerance && std::abs(dtheta) < angle_tolerance);
}

bool ctrl::OmnidirectionalTrajectoryGenerator::isGoalReached(const Eigen::Vector3d& current_pose,
                                                             const Eigen::Vector3d& goal_pose) {
  double dx = current_pose[0] - goal_pose[0];
  double dy = current_pose[1] - goal_pose[1];
  double dtheta = normalizeAngle(current_pose[2] - goal_pose[2]);

  return (std::sqrt(dx * dx + dy * dy) < position_tolerance && std::abs(dtheta) < angle_tolerance);
}

double ctrl::OmnidirectionalTrajectoryGenerator::normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

// Main trajectory generation function
Eigen::Vector3d ctrl::OmnidirectionalTrajectoryGenerator::generateTrajectory(const State& current,
                                                                             const State& goal) {
  // Check if we've reached the goal
  if (isGoalReached(current, goal)) {
    return Eigen::Vector3d::Zero();  // Stop
  }

  // Calculate desired accelerations using the elliptical envelope approach
  auto [a_x, a_y, a_theta] = calculateDesiredAccelerations(current, goal);

  // Convert to body velocities (compatible with your system)
  return accelerationsToBodyVelocity(current, a_x, a_y, a_theta);
}

// Alternative interface compatible with your current system
std::pair<bool, Eigen::Vector3d> ctrl::OmnidirectionalTrajectoryGenerator::Update(
    const Eigen::Vector3d& current_pose, const Eigen::Vector3d& goal_pose) {
  // Convert to State format
  State current(current_pose[0], current_pose[1], current_pose[2], 0, 0, 0);
  State goal(goal_pose[0], goal_pose[1], goal_pose[2], 0, 0, 0);

  // Set goal in current state for the simplified controller
  current.goal_x = goal_pose[0];
  current.goal_y = goal_pose[1];
  current.goal_theta = goal_pose[2];

  // Check if goal is reached
  if (isGoalReached(current, goal)) {
    return std::make_pair(true, Eigen::Vector3d::Zero());
  }

  // Generate trajectory using the simplified proportional controller
  Eigen::Vector3d velocity =
      accelerationsToBodyVelocity(current, 0, 0, 0);  // We don't use accelerations anymore

  return std::make_pair(false, velocity);
}

// Calculate desired accelerations using elliptical envelope
std::tuple<double, double, double>
ctrl::OmnidirectionalTrajectoryGenerator::calculateDesiredAccelerations(const State& current,
                                                                        const State& goal) {
  // Calculate position and velocity errors
  double dx = goal.x - current.x;
  double dy = goal.y - current.y;
  double dtheta = normalizeAngle(goal.theta - current.theta);

  // For this implementation, assume goal velocities are zero
  double dvx = 0 - current.x_dot;
  double dvy = 0 - current.y_dot;
  double dvtheta = 0 - current.theta_dot;

  // Solve 1D optimal control problems for x and y independently
  double a_x = solveOptimalControl1D(dx, current.x_dot, 0, a_max_x, v_max);
  double a_y = solveOptimalControl1D(dy, current.y_dot, 0, a_max_y, v_max);
  double a_theta =
      solveOptimalControl1D(dtheta, current.theta_dot, 0, theta_ddot_max, theta_dot_max);

  // Apply elliptical constraint
  double acceleration_magnitude =
      std::sqrt((a_x * a_x) / (envelope_a * envelope_a) + (a_y * a_y) / (envelope_b * envelope_b));

  if (acceleration_magnitude > 1.0) {
    // Scale to stay within elliptical envelope
    a_x /= acceleration_magnitude;
    a_y /= acceleration_magnitude;
  }

  // Limit angular acceleration
  a_theta = std::max(-theta_ddot_max, std::min(theta_ddot_max, a_theta));

  return {a_x, a_y, a_theta};
}

// Solve the 1D optimal control problem from Section 4
double ctrl::OmnidirectionalTrajectoryGenerator::solveOptimalControl1D(double position_error,
                                                                       double initial_velocity,
                                                                       double final_velocity,
                                                                       double max_acceleration,
                                                                       double max_velocity) {
  // Normalize problem (handle negative distances)
  double sign = (position_error >= 0) ? 1.0 : -1.0;
  double w_f = std::abs(position_error);
  double w_dot_0 = sign * initial_velocity;
  double w_dot_f = sign * final_velocity;

  if (w_f < 1e-6) return 0.0;  // Already at destination

  // Determine which case applies and calculate control effort
  TrajectorySegment segment =
      determineTrajectoryCase(w_f, w_dot_0, max_acceleration, max_velocity);

  return sign * segment.control_effort;
}

ctrl::OmnidirectionalTrajectoryGenerator::TrajectorySegment
ctrl::OmnidirectionalTrajectoryGenerator::determineTrajectoryCase(double w_f, double w_dot_0,
                                                                  double a_max, double v_max) {
  TrajectorySegment segment;

  // Case 1: w_dot_0 < 0 (moving away from destination)
  if (w_dot_0 < 0) {
    segment.case_type = CASE_1;
    segment.control_effort = a_max;
    segment.duration = -w_dot_0 / a_max;
    segment.distance = -w_dot_0 * w_dot_0 / (2 * a_max);
    segment.final_velocity = 0;
    return segment;
  }

  // Case 3: w_dot_0 > v_max (moving too fast)
  if (w_dot_0 > v_max) {
    segment.case_type = CASE_3;
    segment.control_effort = -a_max;
    segment.duration = (w_dot_0 - v_max) / a_max;
    segment.distance = (w_dot_0 * w_dot_0 - v_max * v_max) / (2 * a_max);
    segment.final_velocity = v_max;
    return segment;
  }

  // Cases 2.1, 2.2, 2.3: Normal operation
  double stopping_distance = w_dot_0 * w_dot_0 / (2 * a_max);

  if (w_f <= stopping_distance) {
    // Case 2.3: Just decelerate
    segment.case_type = CASE_2_3;
    segment.control_effort = -a_max;
    segment.duration = w_dot_0 / a_max;
    segment.distance = stopping_distance;
    segment.final_velocity = 0;
  } else {
    // Case 2.1: Accelerate first
    segment.case_type = CASE_2_1;
    segment.control_effort = a_max;

    // Check if we reach v_max or need to start braking
    double w_1 = std::sqrt(w_f * a_max + w_dot_0 * w_dot_0 / 2);

    if (w_1 <= v_max) {
      // Accelerate then immediately decelerate
      segment.duration = (w_1 - w_dot_0) / a_max;
      segment.distance = w_f / 2 + w_dot_0 * w_dot_0 / (2 * a_max);
      segment.final_velocity = w_1;
    } else {
      // Accelerate to v_max
      segment.duration = (v_max - w_dot_0) / a_max;
      segment.distance = (v_max * v_max - w_dot_0 * w_dot_0) / (2 * a_max);
      segment.final_velocity = v_max;
    }
  }

  return segment;
}

// Convert accelerations to body velocities (compatible with your motion controller)
Eigen::Vector3d ctrl::OmnidirectionalTrajectoryGenerator::accelerationsToBodyVelocity(
    const State& current, double a_x, double a_y, double a_theta) {
  // TEMPORARY: Simple high-speed proportional controller instead of complex acceleration
  // integration This should give us much higher speeds immediately

  double goal_x = current.goal_x;  // These should be set from Update() function
  double goal_y = current.goal_y;
  double goal_theta = current.goal_theta;

  // Simple proportional gains - aggressive for high speed
  double kp_pos = 3.0;    // Aggressive position gain
  double kp_theta = 8.0;  // Aggressive angular gain

  // Calculate position errors
  double dx = goal_x - current.x;
  double dy = goal_y - current.y;
  double dtheta = normalizeAngle(goal_theta - current.theta);

  // Generate velocities directly (skip acceleration integration)
  double vx_desired = kp_pos * dx;
  double vy_desired = kp_pos * dy;
  double vtheta_desired = kp_theta * dtheta;

  // Transform to local frame
  double cos_theta = std::cos(current.theta);
  double sin_theta = std::sin(current.theta);

  double vx_local = cos_theta * vx_desired + sin_theta * vy_desired;
  double vy_local = -sin_theta * vx_desired + cos_theta * vy_desired;

  // Apply aggressive velocity limits (much higher than before)
  Eigen::Vector3d velocity_fBody;
  velocity_fBody[0] = std::clamp(vx_local, -3.0, 3.0);  // High speed limits
  velocity_fBody[1] = std::clamp(vy_local, -3.0, 3.0);
  velocity_fBody[2] = std::clamp(vtheta_desired, -10.0, 10.0);

  // Debug output to see what we're commanding
  static int debug_counter = 0;
  if (++debug_counter % 50 == 0) {  // Every 1 second
    std::cout << "[OmniTrajGen] POS ERROR: (" << dx << ", " << dy << ", " << dtheta << ")"
              << std::endl;
    std::cout << "[OmniTrajGen] COMMANDING: (" << velocity_fBody[0] << ", " << velocity_fBody[1]
              << ", " << velocity_fBody[2] << ")" << std::endl;
  }

  return velocity_fBody;
}

// Get current envelope parameters for debugging
void ctrl::OmnidirectionalTrajectoryGenerator::getEnvelopeParameters(double& a, double& b,
                                                                     double& angle) const {
  a = envelope_a;
  b = envelope_b;
  angle = envelope_angle;
}
