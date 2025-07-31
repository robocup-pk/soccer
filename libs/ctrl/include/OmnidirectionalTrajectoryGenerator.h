#ifndef OMNIDIRECTIONAL_TRAJECTORY_GENERATOR_H
#define OMNIDIRECTIONAL_TRAJECTORY_GENERATOR_H

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <tuple>
#include <memory>
#include <Eigen/Dense>

#include "SystemConfig.h"
#include "RobotDescription.h"
#include "RobotModel.h"

namespace ctrl {

class OmnidirectionalTrajectoryGenerator {
 public:
  // Vehicle state structure
  struct State {
    double x, y, theta;
    double x_dot, y_dot, theta_dot;
    double goal_x, goal_y, goal_theta;  // Goal position for simplified controller

    State(double x = 0, double y = 0, double theta = 0, double x_dot = 0, double y_dot = 0,
          double theta_dot = 0)
        : x(x),
          y(y),
          theta(theta),
          x_dot(x_dot),
          y_dot(y_dot),
          theta_dot(theta_dot),
          goal_x(0),
          goal_y(0),
          goal_theta(0) {}
  };

  // Wheel velocities structure (compatible with your system)
  struct WheelVelocities {
    Eigen::Vector4d velocities;  // wheel velocities in rad/s

    WheelVelocities(double v1 = 0, double v2 = 0, double v3 = 0, double v4 = 0)
        : velocities(v1, v2, v3, v4) {}

    WheelVelocities(const Eigen::Vector4d& vels) : velocities(vels) {}
  };

 private:
  // Vehicle parameters (from research paper)
  double mu = 0.8;    // friction coefficient
  double g = 9.81;    // gravity
  double m = 2.7;     // mass (kg)
  double J = 0.0085;  // moment of inertia (m²kg)
  double h = 0.05;    // height of center of mass (m)

  // Wheel configuration based on your robot description
  struct WheelConfig {
    double px, py;  // Position vector components (normalized)
    double fx, fy;  // Force direction vector components (unit vectors)
    double angle;   // Wheel angle in radians
  };

  std::vector<WheelConfig> wheels;

  // Motor limitations (from your system)
  double max_wheel_speed = 500.0;         // rad/s (converted from RPM)
  double max_wheel_acceleration = 350.0;  // rad/s² (increased for 1.5 m/s capability)
  double min_wheel_speed = 0.105;         // rad/s (motor dead zone)

  // Derived envelope parameters
  double a_max_x = 4.0;         // maximum linear acceleration in x (m/s²) - increased for 1 m/s²
  double a_max_y = 4.0;         // maximum linear acceleration in y (m/s²) - increased for 1 m/s²
  double theta_dot_max = 10.0;  // maximum angular velocity (rad/s) - increased
  double theta_ddot_max = 6.0;  // maximum angular acceleration (rad/s²) - increased
  double v_max = 2.0;           // maximum velocity (m/s) - keep for 1.5 m/s target

  // Elliptical envelope parameters
  double envelope_a = 1.0;      // semi-major axis
  double envelope_b = 1.0;      // semi-minor axis
  double envelope_angle = 0.0;  // orientation angle

  // Control tolerance
  double position_tolerance = 0.01;  // 1cm
  double angle_tolerance = 0.05;     // ~3 degrees

  // Single DOF trajectory case types
  enum TrajectoryCase {
    CASE_1,    // initial velocity < 0
    CASE_2_1,  // accelerate phase
    CASE_2_2,  // cruise phase
    CASE_2_3,  // decelerate phase
    CASE_3     // initial velocity > v_max
  };

  struct TrajectorySegment {
    TrajectoryCase case_type;
    double control_effort;  // q_w(t)
    double duration;        // t'
    double distance;        // w'
    double final_velocity;  // w_dot'
  };

  // Robot kinematics interface
  std::shared_ptr<kin::RobotModel> robot_model;

 public:
  OmnidirectionalTrajectoryGenerator();

  // Set vehicle parameters
  void setVehicleParameters(double friction_coeff, double mass, double inertia, double cm_height);
  void setMotorLimits(double max_speed_radps, double max_acc, double min_speed);
  void setVelocityLimits(double max_velocity);
  void setTolerances(double pos_tol, double angle_tol);

  // Main trajectory generation function
  Eigen::Vector3d generateTrajectory(const State& current, const State& goal);

  // Alternative interface compatible with your current system
  std::pair<bool, Eigen::Vector3d> Update(const Eigen::Vector3d& current_pose,
                                          const Eigen::Vector3d& goal_pose);

  // Check if goal is reached
  bool isGoalReached(const State& current, const State& goal);
  bool isGoalReached(const Eigen::Vector3d& current_pose, const Eigen::Vector3d& goal_pose);

  // Get current envelope parameters for debugging
  void getEnvelopeParameters(double& a, double& b, double& angle) const;

 private:
  void initializeWheelConfiguration();
  void verifyWheelAngles();
  void calculateEllipticalEnvelope();
  void calculateVelocityLimits();

  double normalizeAngle(double angle);

  // Calculate desired accelerations using elliptical envelope
  std::tuple<double, double, double> calculateDesiredAccelerations(const State& current,
                                                                   const State& goal);

  // Solve the 1D optimal control problem from Section 4
  double solveOptimalControl1D(double position_error, double initial_velocity,
                               double final_velocity, double max_acceleration,
                               double max_velocity);

  TrajectorySegment determineTrajectoryCase(double w_f, double w_dot_0, double a_max,
                                            double v_max);

  // Convert accelerations to body velocities (compatible with your motion controller)
  Eigen::Vector3d accelerationsToBodyVelocity(const State& current, double a_x, double a_y,
                                              double a_theta);
};

}  // namespace ctrl

#endif  // OMNIDIRECTIONAL_TRAJECTORY_GENERATOR_H
