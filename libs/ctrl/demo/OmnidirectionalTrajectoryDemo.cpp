#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "OmnidirectionalTrajectoryGenerator.h"
#include "SystemConfig.h"

int main() {
  std::cout << "=== Omnidirectional Trajectory Generator Demo ===" << std::endl;

  // Create the trajectory generator
  ctrl::OmnidirectionalTrajectoryGenerator generator;

  // Display current parameters
  double a, b, angle;
  generator.getEnvelopeParameters(a, b, angle);
  std::cout << "\nElliptical Envelope Parameters:" << std::endl;
  std::cout << "  Semi-major axis: " << a << " m/s²" << std::endl;
  std::cout << "  Semi-minor axis: " << b << " m/s²" << std::endl;
  std::cout << "  Orientation: " << angle * 180 / M_PI << "°" << std::endl;

  // Test scenarios
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> test_cases = {
      {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0)},       // Forward
      {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0)},       // Left
      {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 1.0, 0.0)},       // Diagonal
      {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, M_PI / 2)},  // Rotation only
      {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 1.0, M_PI / 4)},  // Combined motion
  };

  std::vector<std::string> case_names = {"Forward Motion", "Left Motion", "Diagonal Motion",
                                         "Pure Rotation", "Combined Motion"};

  for (size_t i = 0; i < test_cases.size(); ++i) {
    std::cout << "\n--- " << case_names[i] << " ---" << std::endl;

    Eigen::Vector3d current = test_cases[i].first;
    Eigen::Vector3d goal = test_cases[i].second;

    std::cout << "Current: (" << current.transpose() << ")" << std::endl;
    std::cout << "Goal: (" << goal.transpose() << ")" << std::endl;

    // Test a few iterations
    for (int step = 0; step < 5; ++step) {
      auto [finished, velocity] = generator.Update(current, goal);

      std::cout << "Step " << step << ": ";
      if (finished) {
        std::cout << "GOAL REACHED!" << std::endl;
        break;
      } else {
        std::cout << "Velocity = (" << velocity.transpose() << ")" << std::endl;

        // Simple integration for demonstration
        double dt = 0.02;  // 50Hz
        current[0] += velocity[0] * dt;
        current[1] += velocity[1] * dt;
        current[2] += velocity[2] * dt;
      }
    }
  }

  // Test goal reaching functionality
  std::cout << "\n--- Goal Reaching Test ---" << std::endl;
  Eigen::Vector3d near_goal(0.005, 0.005, 0.01);  // Within tolerance
  Eigen::Vector3d origin(0.0, 0.0, 0.0);

  bool reached = generator.isGoalReached(origin, near_goal);
  std::cout << "Goal within tolerance reached: " << (reached ? "YES" : "NO") << std::endl;

  Eigen::Vector3d far_goal(1.0, 1.0, 1.0);  // Outside tolerance
  reached = generator.isGoalReached(origin, far_goal);
  std::cout << "Goal outside tolerance reached: " << (reached ? "YES" : "NO") << std::endl;

  std::cout << "\n=== Demo Complete ===" << std::endl;

  return 0;
}
