// Demo application to test ball physics and kicking functionality
#include <iostream>
#include <chrono>
#include <thread>

// Project libs
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Kinematics.h"
#include "Coordinates.h"
#include "Utils.h"
#include "RobotManager.h"
#include "BallModel.h"
#include "Kick.h"

int main(int argc, char* argv[]) {
  std::cout << "[DemoBallPhysicsAndKicking] Starting ball physics and kicking demo..." << std::endl;
  
  // Check configuration
  if (cfg::SystemConfig::num_robots != 1) {
    std::cout << "[DemoBallPhysicsAndKicking] Set num_robots to 1. Exiting!" << std::endl;
    return 0;
  }

  // Initialize soccer objects (robot and ball)
  std::vector<state::SoccerObject> soccer_objects;
  
  // Manually create robot and ball objects
  state::SoccerObject robot_obj("robot_0", Eigen::Vector3d::Zero(), Eigen::Vector2d(0.18, 0.18), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 2.0f);
  robot_obj.radius_m = 0.09f;
  soccer_objects.push_back(robot_obj);

  kin::BallModel ball_obj;
  soccer_objects.push_back(ball_obj);
  
  // Initialize OpenGL simulation
  vis::GLSimulation gl_simulation;
  gl_simulation.InitGameObjects(soccer_objects);
  
  // Initialize robot manager
  // rob::RobotManager robot_manager;
  // Eigen::Vector3d initial_pose(0, 0, 0);
  // robot_manager.InitializePose(initial_pose);  // Start at origin
  
  // Demo control variables
  bool demo_started = false;
  bool ball_kicked = false;
  double demo_start_time = 0.0;
  double kick_time = 3.0;  // Kick after 3 seconds
  bool space_pressed_last_frame = false;
  bool enter_pressed_last_frame = false;
  
  std::cout << "Demo Instructions:" << std::endl;
  std::cout << "1. The robot is positioned to face the ball." << std::endl;
  std::cout << "2. After 3 seconds, the robot will kick the ball." << std::endl;
  std::cout << "3. Watch the ball move with realistic physics!" << std::endl;
  std::cout << "4. Press SPACE or 1 to kick again, ENTER to reset ball position." << std::endl;
  std::cout << "5. Press ESC to exit." << std::endl;

  // Main simulation loop
  while (true) {
    double current_time = util::GetCurrentTime();
    double dt = util::CalculateDt();
    
    if (!demo_started) {
      demo_start_time = current_time;
      demo_started = true;
      
      // Position robot and ball
    Eigen::Vector3d robot_start_pos(-0.1, 0, 0); // Robot 10cm from ball
    Eigen::Vector3d ball_start_pos(0, 0, 0);     // Ball at origin
    double angle = atan2(ball_start_pos.y() - robot_start_pos.y(), ball_start_pos.x() - robot_start_pos.x());

    for (auto& obj : soccer_objects) {
        if (obj.name == "robot_0") {
            obj.position = robot_start_pos;
            obj.position.z() = angle;
            // robot_manager.InitializePose(obj.position);
        } else if (obj.name == "ball") {
            obj.position = ball_start_pos;
            obj.velocity = Eigen::Vector3d::Zero();
        }
    }
    }
    
    // Execute kick after specified time
    if (!ball_kicked && (current_time - demo_start_time) >= kick_time) {
      std::cout << "[Demo] Robot is kicking the ball now!" << std::endl;
      for (auto& obj : soccer_objects) {
        if (obj.name == "robot_0") {
            for (auto& ball_obj : soccer_objects) {
                if (ball_obj.name == "ball") {
                    kin::Kick(obj, ball_obj);
                    break;
                }
            }
            break;
        }
      }
      ball_kicked = true;
    }
    
    // Handle input
    vis::ProcessInput(gl_simulation.GetRawGLFW(), soccer_objects);
    
    // Check for manual kick (spacebar or 1 key)
    if (glfwGetKey(gl_simulation.GetRawGLFW(), GLFW_KEY_SPACE) == GLFW_PRESS || 
        glfwGetKey(gl_simulation.GetRawGLFW(), GLFW_KEY_1) == GLFW_PRESS) {
      if (!space_pressed_last_frame) {
        std::cout << "[Demo] Manual kick triggered!" << std::endl;
        for (auto& obj : soccer_objects) {
            if (obj.name == "robot_0") {
                for (auto& ball_obj : soccer_objects) {
                    if (ball_obj.name == "ball") {
                        kin::Kick(obj, ball_obj);
                        break;
                    }
                }
                break;
            }
        }
      }
      space_pressed_last_frame = true;
    } else {
      space_pressed_last_frame = false;
    }
    
    // Check for ball reset (enter key)
    if (glfwGetKey(gl_simulation.GetRawGLFW(), GLFW_KEY_ENTER) == GLFW_PRESS) {
      if (!enter_pressed_last_frame) {
      std::cout << "[Demo] Resetting ball position" << std::endl;
      for (auto& obj : soccer_objects) {
        if (obj.name == "ball") {
          obj.position = Eigen::Vector3d(0, 0, 0);
          obj.velocity = Eigen::Vector3d::Zero();
          obj.acceleration = Eigen::Vector3d::Zero();
        }
      }
      ball_kicked = false;
      demo_start_time = current_time;
      }
      enter_pressed_last_frame = true;
    } else {
      enter_pressed_last_frame = false;
    }
    
    // Update physics (including advanced ball physics)
    kin::UpdateKinematics(soccer_objects, dt);
    kin::CheckAndResolveCollisions(soccer_objects);
    
    // Update robot state from soccer objects (robot_manager.GetPoseInWorldFrame() is not used in this simplified demo)
    for (auto& obj : soccer_objects) {
      if (obj.name == "robot_0") {
        // obj.position = robot_manager.GetPoseInWorldFrame(); // Keep this for visualization
        break;
      }
    }
    
    // Check if simulation should continue and render
    if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
      std::cout << "[Demo] Simulation window closed" << std::endl;
      break;
    }
    
    // Print ball info periodically
    static double last_print_time = 0;
    if (current_time - last_print_time > 1.0) {  // Print every second
      for (const auto& obj : soccer_objects) {
        if (obj.name == "ball") {
          double speed = std::sqrt(obj.velocity[0]*obj.velocity[0] + obj.velocity[1]*obj.velocity[1]);
          std::cout << "[Demo] Ball position: (" << obj.position[0] << ", " << obj.position[1] 
                    << "), Speed: " << speed << " m/s, Velocity: (" << obj.velocity[0] << ", " 
                    << obj.velocity[1] << ", " << obj.velocity[2] << ")" << std::endl;
        }
      }
      last_print_time = current_time;
    }
    
  }

  std::cout << "[DemoBallPhysicsAndKicking] Demo finished!" << std::endl;
  return 0;
}