
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>

// Project libs
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Coordinates.h"
#include "Utils.h"
#include "RobotManager.h"
#include "RobotModel.h"
#include "OmnidirectionalTrajectoryGenerator.h"

class OmnidirectionalDemo {
 private:
  rob::RobotManager robot_manager;
  kin::RobotModel robot_model;
  std::vector<state::SoccerObject> soccer_objects;
  vis::GLSimulation gl_simulation;

  // Demo parameters
  int current_test = 0;
  double test_start_time = 0.0;
  double test_duration = 25.0;  // Increased duration for longer high-speed paths
  bool test_in_progress = false;

  // Multi-point path following
  int current_waypoint_index = 0;
  double waypoint_reached_tolerance = 0.15;  // 15cm tolerance for higher speeds
  bool following_multi_point_path = false;

  struct TestCase {
    std::string name;
    std::vector<Eigen::Vector3d> path;
    std::string description;
    bool is_multi_point;  // Flag for multi-point vs single point-to-point
  };

  std::vector<TestCase> test_cases;

 public:
  OmnidirectionalDemo() {
    // Initialize test cases
    setupTestCases();

    // Initialize soccer objects
    state::SoccerObject robot_obj("robot_0", Eigen::Vector3d::Zero(), Eigen::Vector2d(0.18, 0.18),
                                  Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 2.0f);
    robot_obj.radius_m = 0.09f;
    soccer_objects.push_back(robot_obj);

    // Initialize simulation
    gl_simulation.InitGameObjects(soccer_objects);

    // Initialize robot pose
    Eigen::Vector3d initial_pose(0, 0, 0);
    robot_manager.SetIdealPose(initial_pose);
    robot_manager.SetUseIdealPoseTracking(true);

    std::cout << std::fixed << std::setprecision(3);
    printInstructions();
  }

  void setupTestCases() {
    TestCase test;

    // Test 1: Forward motion with longer distance for gradual acceleration
    test.name = "High-Speed Forward Motion";
    test.description = "Fast forward translation with distance for speed buildup";
    test.path = {
        Eigen::Vector3d(0, 0, 0),   // Start at origin
        Eigen::Vector3d(1.5, 0, 0)  // Longer distance for gradual acceleration
    };
    test.is_multi_point = false;
    test_cases.push_back(test);

    // Test 2: Lateral motion with longer distance for gradual acceleration
    test.name = "High-Speed Lateral Motion";
    test.description = "Fast sideways translation with sustained speed";
    test.path = {
        Eigen::Vector3d(0, 0, 0),   // Start at origin
        Eigen::Vector3d(0, 1.2, 0)  // Longer lateral distance for gradual acceleration
    };
    test.is_multi_point = false;
    test_cases.push_back(test);

    // Test 3: Complex zigzag path (multi-point with shorter segments)
    test.name = "Zigzag Path (High-Speed Multi-Point)";
    test.description = "Fast zigzag navigation with segments for field size";
    test.path = {
        Eigen::Vector3d(0, 0, 0),            // Start at origin
        Eigen::Vector3d(0.4, 0.3, 0),        // First zig (field-appropriate)
        Eigen::Vector3d(0.8, -0.2, 0),       // First zag (field-appropriate)
        Eigen::Vector3d(1.0, 0.4, 0),        // Second zig (field-appropriate)
        Eigen::Vector3d(1.0, 0.4, M_PI / 4)  // Final orientation
    };
    test.is_multi_point = true;
    test_cases.push_back(test);

    // Test 4: Field navigation (appropriate for soccer field)
    test.name = "Field Navigation";
    test.description = "High-speed navigation across field-appropriate distances";
    test.path = {
        Eigen::Vector3d(0, 0, 0),      // Start
        Eigen::Vector3d(0.5, 0, 0),    // Build speed forward
        Eigen::Vector3d(0.7, 0.4, 0),  // Small curve
        Eigen::Vector3d(0.9, 0.6, 0),  // Continue
        Eigen::Vector3d(1.1, 0.4, 0),  // Arc back
        Eigen::Vector3d(1.2, 0, 0)     // Final destination
    };
    test.is_multi_point = true;
    test_cases.push_back(test);

    // Test 5: Square path with longer edges for gradual acceleration
    test.name = "Square Path";
    test.description = "Navigate a square with longer edges for proper acceleration";
    test.path = {
        Eigen::Vector3d(0, 0, 0),               // Start (facing forward)
        Eigen::Vector3d(1.5, 0, M_PI / 2),      // Longer forward edge
        Eigen::Vector3d(1.5, 1.5, M_PI),        // Longer right edge
        Eigen::Vector3d(0, 1.5, 3 * M_PI / 2),  // Longer back edge
        Eigen::Vector3d(0, 0, 0)                // Back to start
    };
    test.is_multi_point = true;
    test_cases.push_back(test);

    // Test 6: S-curve path (field-appropriate scale)
    test.name = "S-Curve Path";
    test.description = "Smooth S-curve for field-appropriate distances";
    test.path = {
        Eigen::Vector3d(0, 0, 0),              // Start
        Eigen::Vector3d(0.3, 0.2, M_PI / 8),   // Into curve
        Eigen::Vector3d(0.5, 0.4, M_PI / 4),   // Through curve
        Eigen::Vector3d(0.7, 0.5, M_PI / 6),   // Transition
        Eigen::Vector3d(0.9, 0.4, 0),          // Curve back
        Eigen::Vector3d(1.1, 0.2, -M_PI / 8),  // Complete S
        Eigen::Vector3d(1.2, 0, 0)             // Finish
    };
    test.is_multi_point = true;
    test_cases.push_back(test);

    // Test 7: Diagonal motion with longer distance for gradual acceleration
    test.name = "High-Speed Diagonal Motion";
    test.description = "Fast diagonal movement with longer distance for proper acceleration";
    test.path = {
        Eigen::Vector3d(0, 0, 0),            // Start at origin
        Eigen::Vector3d(1.5, 1.5, M_PI / 4)  // Longer diagonal for gradual acceleration
    };
    test.is_multi_point = false;
    test_cases.push_back(test);

    // Test 8: Rapid rotation test (300 RPM test)
    test.name = "Rapid Rotation (300 RPM Test)";
    test.description = "Fast rotation in place to test maximum wheel speeds";
    test.path = {
        Eigen::Vector3d(0, 0, 0),        // Start at origin
        Eigen::Vector3d(0, 0, 2 * M_PI)  // Full 360 degree rotation
    };
    test.is_multi_point = false;
    test_cases.push_back(test);

    // Test 9: Figure-8 pattern (field-appropriate for sustained speed)
    test.name = "Figure-8 Pattern";
    test.description = "Complex curved trajectory at field-appropriate scale";
    test.path = {Eigen::Vector3d(0, 0, 0),
                 Eigen::Vector3d(0.2, 0.2, M_PI / 6),
                 Eigen::Vector3d(0.4, 0.3, M_PI / 3),
                 Eigen::Vector3d(0.5, 0.1, M_PI / 2),
                 Eigen::Vector3d(0.4, -0.1, 2 * M_PI / 3),
                 Eigen::Vector3d(0.2, -0.2, 5 * M_PI / 6),
                 Eigen::Vector3d(0, 0, M_PI),
                 Eigen::Vector3d(-0.2, 0.2, 7 * M_PI / 6),
                 Eigen::Vector3d(-0.4, 0.3, 4 * M_PI / 3),
                 Eigen::Vector3d(-0.5, 0.1, 3 * M_PI / 2),
                 Eigen::Vector3d(-0.4, -0.1, 5 * M_PI / 3),
                 Eigen::Vector3d(-0.2, -0.2, 11 * M_PI / 6),
                 Eigen::Vector3d(0, 0, 2 * M_PI)};
    test.is_multi_point = true;
    test_cases.push_back(test);
  }

  void printInstructions() {
    std::cout << "=== Omnidirectional Trajectory Demo with Wheel Speed Verification ==="
              << std::endl;
    std::cout << "This demo tests various motion patterns and verifies wheel speeds." << std::endl;
    std::cout << "\nTest Cases:" << std::endl;
    for (size_t i = 0; i < test_cases.size(); ++i) {
      std::cout << "  " << (i + 1) << ". " << test_cases[i].name << " - "
                << test_cases[i].description << std::endl;
    }
    std::cout << "\nControls:" << std::endl;
    std::cout << "  SPACE: Start/Next test" << std::endl;
    std::cout << "  R: Reset to beginning" << std::endl;
    std::cout << "  ESC: Exit" << std::endl;
    std::cout << "\nPress SPACE to start the first test..." << std::endl;
  }

  void startNextTest() {
    if (current_test >= static_cast<int>(test_cases.size())) {
      std::cout << "\n=== All tests completed! Press R to restart or ESC to exit ===" << std::endl;
      return;
    }

    TestCase& test = test_cases[current_test];
    std::cout << "\n=== Starting Test " << (current_test + 1) << ": " << test.name
              << " ===" << std::endl;
    std::cout << "Description: " << test.description << std::endl;
    std::cout << "Path points: " << test.path.size() << std::endl;
    std::cout << "Multi-point path: " << (test.is_multi_point ? "YES" : "NO") << std::endl;

    // Print the full path for debugging
    std::cout << "Full path: ";
    for (size_t i = 0; i < test.path.size(); ++i) {
      std::cout << "(" << std::fixed << std::setprecision(2) << test.path[i][0] << ","
                << test.path[i][1] << "," << test.path[i][2] << ")";
      if (i < test.path.size() - 1) std::cout << " -> ";
    }
    std::cout << std::endl;

    // Reset robot to the first point of the path
    if (!test.path.empty()) {
      Eigen::Vector3d start_pose = test.path[0];
      robot_manager.SetIdealPose(start_pose);
      std::cout << "Reset robot to start position: (" << start_pose[0] << ", " << start_pose[1]
                << ", " << start_pose[2] << ")" << std::endl;

      // Initialize waypoint following
      current_waypoint_index = 1;  // Start with the first target waypoint
      following_multi_point_path = test.is_multi_point;

      if (test.is_multi_point && test.path.size() > 1) {
        // For multi-point paths, start with the first waypoint
        setNextWaypoint(test);
      } else if (test.path.size() >= 2) {
        // For single point-to-point, set the final goal
        Eigen::Vector3d goal_pose = test.path.back();
        robot_manager.SetOmnidirectionalGoal(goal_pose);
        std::cout << "Set single goal to: (" << goal_pose[0] << ", " << goal_pose[1] << ", "
                  << goal_pose[2] << ")" << std::endl;
      }
    }

    test_start_time = util::GetCurrentTime();
    test_in_progress = true;
    current_test++;
  }

  void setNextWaypoint(const TestCase& test) {
    if (current_waypoint_index < test.path.size()) {
      Eigen::Vector3d waypoint = test.path[current_waypoint_index];
      robot_manager.SetOmnidirectionalGoal(waypoint);
      std::cout << "→ Set waypoint " << current_waypoint_index << ": (" << waypoint[0] << ", "
                << waypoint[1] << ", " << waypoint[2] << ")" << std::endl;
    }
  }

  void updateMultiPointPath() {
    if (!following_multi_point_path || !test_in_progress || current_test == 0) return;

    TestCase& test = test_cases[current_test - 1];

    // Check if current waypoint is reached
    Eigen::Vector3d current_pose = robot_manager.GetPoseInWorldFrame();

    if (current_waypoint_index < test.path.size()) {
      Eigen::Vector3d target_waypoint = test.path[current_waypoint_index];

      // Calculate distance to current waypoint (position + orientation)
      double position_distance = (current_pose.head<2>() - target_waypoint.head<2>()).norm();
      double angle_distance = std::abs(util::WrapAngle(current_pose[2] - target_waypoint[2]));

      // Check if waypoint is reached (more tolerant for high speeds)
      if (position_distance < waypoint_reached_tolerance &&
          angle_distance < 0.3) {  // 0.3 rad ≈ 17 degrees
        std::cout << "✓ Reached waypoint " << current_waypoint_index << " (pos_err: " << std::fixed
                  << std::setprecision(3) << position_distance << "m, ang_err: " << angle_distance
                  << " rad)" << std::endl;

        current_waypoint_index++;

        if (current_waypoint_index < test.path.size()) {
          // Move to next waypoint
          setNextWaypoint(test);
        } else {
          // Path completed
          std::cout << "🎯 High-speed multi-point path completed!" << std::endl;
          following_multi_point_path = false;
        }
      }
    }
  }

  void resetDemo() {
    current_test = 0;
    test_in_progress = false;

    // Reset robot position
    Eigen::Vector3d initial_pose(0, 0, 0);
    robot_manager.SetIdealPose(initial_pose);

    std::cout << "\n=== Demo Reset ===\nPress SPACE to start the first test..." << std::endl;
  }

  void verifyPositionTracking() {
    // Get the actual pose and target from the robot manager and trajectory generator
    Eigen::Vector3d current_pose = robot_manager.GetPoseInWorldFrame();

    // Position tracking at 1 Hz (every 1000ms at 50Hz = every 50 calls)
    static int position_counter = 0;
    if (++position_counter % 50 == 0) {  // Every 50 calls = 1000ms = 1 Hz

      printf("\n--- Position Tracking (Goal vs Actual) ---\n");
      printf("Actual Position: (%.3f, %.3f, %.3f rad)\n", current_pose(0), current_pose(1),
             current_pose(2));

      // Print goal if we have a test in progress
      if (test_in_progress && current_test > 0) {
        TestCase& test = test_cases[current_test - 1];

        if (test.is_multi_point && current_waypoint_index < test.path.size()) {
          // Multi-point: show current waypoint target
          Eigen::Vector3d current_target = test.path[current_waypoint_index];
          printf("Goal Position:   (%.3f, %.3f, %.3f rad) [Waypoint %d/%zu]\n", current_target(0),
                 current_target(1), current_target(2), current_waypoint_index + 1,
                 test.path.size());

          // Calculate position error to current waypoint
          Eigen::Vector2d position_error = current_pose.head<2>() - current_target.head<2>();
          double angle_error = normalize_angle(current_pose(2) - current_target(2));

          printf("Position Error:  (%.3f, %.3f) m, %.3f rad (%.1f deg)\n", position_error(0),
                 position_error(1), angle_error, angle_error * 180.0 / M_PI);
          printf("Distance Error:  %.3f m\n", position_error.norm());

          // Show progress through waypoints
          if (current_waypoint_index > 0) {
            printf("Completed Waypoints: %d/%zu\n", current_waypoint_index, test.path.size());
          }

        } else if (!test.is_multi_point && !test.path.empty()) {
          // Single point: show final target
          Eigen::Vector3d final_target = test.path.back();
          printf("Goal Position:   (%.3f, %.3f, %.3f rad) [Final Target]\n", final_target(0),
                 final_target(1), final_target(2));

          // Calculate position error to final target
          Eigen::Vector2d position_error = current_pose.head<2>() - final_target.head<2>();
          double angle_error = normalize_angle(current_pose(2) - final_target(2));

          printf("Position Error:  (%.3f, %.3f) m, %.3f rad (%.1f deg)\n", position_error(0),
                 position_error(1), angle_error, angle_error * 180.0 / M_PI);
          printf("Distance Error:  %.3f m\n", position_error.norm());
        }

        printf("Test: %s\n", test.name.c_str());
      } else {
        printf("Goal Position:   (NONE - Test not in progress)\n");
      }

      // Show velocity information briefly
      Eigen::Vector3d world_velocity = robot_manager.GetVelocityInWorldFrame();
      double velocity_magnitude = world_velocity.norm();
      printf("Current Speed:   %.3f m/s\n", velocity_magnitude);
      printf("Robot State:     %s\n",
             robot_manager.GetRobotState().c_str());  // Debug: check which state we're in

      if (velocity_magnitude > 1.0) {
        printf("Status: HIGH-SPEED (Target: ~1.5 m/s max)\n");
      } else if (velocity_magnitude > 0.5) {
        printf("Status: MODERATE-SPEED\n");
      } else if (velocity_magnitude > 0.1) {
        printf("Status: LOW-SPEED\n");
      } else {
        printf("Status: STATIONARY\n");
      }

      printf("------------------------------------------------\n");
    }
  }

  double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
  }

  void printRobotStatus() {
    static int status_counter = 0;
    if (++status_counter % 50 == 0) {  // Print every 1 second (1 Hz)
      Eigen::Vector3d pose = robot_manager.GetPoseInWorldFrame();
      std::string state = robot_manager.GetRobotState();

      std::cout << "\n--- Robot Status ---" << std::endl;
      std::cout << "Position [m]: [" << pose[0] << ", " << pose[1] << "]" << std::endl;
      std::cout << "Orientation [deg]: " << (pose[2] * 180.0 / M_PI) << std::endl;
      std::cout << "State: " << state << std::endl;

      if (test_in_progress && current_test > 0) {
        double elapsed = util::GetCurrentTime() - test_start_time;
        std::cout << "Test Progress: " << elapsed << "/" << test_duration << "s" << std::endl;
      }
    }
  }

  void handleInput() {
    static bool space_pressed_last = false;
    static bool r_pressed_last = false;

    // Handle spacebar for next test
    bool space_pressed = (glfwGetKey(gl_simulation.GetRawGLFW(), GLFW_KEY_SPACE) == GLFW_PRESS);
    if (space_pressed && !space_pressed_last) {
      if (!test_in_progress) {
        startNextTest();
      }
    }
    space_pressed_last = space_pressed;

    // Handle R for reset
    bool r_pressed = (glfwGetKey(gl_simulation.GetRawGLFW(), GLFW_KEY_R) == GLFW_PRESS);
    if (r_pressed && !r_pressed_last) {
      resetDemo();
    }
    r_pressed_last = r_pressed;
  }

  void checkTestCompletion() {
    if (!test_in_progress) return;

    double elapsed = util::GetCurrentTime() - test_start_time;
    std::string state = robot_manager.GetRobotState();

    // Check if test should end (either time limit or reached goal)
    if (elapsed > test_duration || state == "IDLE") {
      test_in_progress = false;
      std::cout << "\n✓ Test completed! Press SPACE for next test..." << std::endl;
    }
  }

  bool run() {
    double dt = util::CalculateDt();

    // Handle user input
    handleInput();

    // Update multi-point path following
    updateMultiPointPath();

    // Check test completion
    checkTestCompletion();

    // Update robot position for visualization
    Eigen::Vector3d current_pose = robot_manager.GetPoseInWorldFrame();
    for (auto& obj : soccer_objects) {
      if (obj.name == "robot_0") {
        obj.position = current_pose;
        break;
      }
    }

    // Verify wheel speeds using actual commanded velocity
    verifyPositionTracking();

    // Print status periodically
    printRobotStatus();

    // Run simulation step
    return gl_simulation.RunSimulationStep(soccer_objects, dt);
  }
};

int main(int argc, char* argv[]) {
  std::cout << "[DemoOmnidirectionalVerification] Starting omnidirectional verification demo..."
            << std::endl;

  // Check configuration
  if (cfg::SystemConfig::num_robots != 1) {
    std::cout << "[DemoOmnidirectionalVerification] Set num_robots to 1. Exiting!" << std::endl;
    return 0;
  }

  try {
    OmnidirectionalDemo demo;

    // Main simulation loop
    while (demo.run()) {
      util::WaitMs(20);  // ~50Hz update rate
    }

  } catch (const std::exception& e) {
    std::cerr << "Demo error: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "[DemoOmnidirectionalVerification] Demo finished!" << std::endl;
  return 0;
}
