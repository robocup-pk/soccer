// Demo application to test ball physics and dribbling functionality
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

int main(int argc, char* argv[]) {
  std::cout << "[DemoBallPhysicsAndDribbling] Starting ball physics and dribbling demo..." << std::endl;
  
  // Check configuration
  if (cfg::SystemConfig::num_robots != 1) {
    std::cout << "[DemoBallPhysicsAndDribbling] Set num_robots to 1. Exiting!" << std::endl;
    return 0;
  }

  // Initialize soccer objects (robot and ball)
  std::vector<state::SoccerObject> soccer_objects;
  state::InitSoccerObjects(soccer_objects);
  
  // Initialize OpenGL simulation
  vis::GLSimulation gl_simulation;
  gl_simulation.InitGameObjects(soccer_objects);
  
  // Initialize robot manager
  rob::RobotManager robot_manager;
  Eigen::Vector3d initial_pose(0, 0, 0);
  robot_manager.InitializePose(initial_pose);  // Start at origin
  
  // Demo control variables
  bool demo_started = false;
  bool ball_approached = false;
  bool is_dribbling = false;
  double demo_start_time = 0.0;
  double approach_time = 3.0;  // Approach ball for 3 seconds
  double dribble_time = 8.0;   // Dribble for 8 seconds
  bool space_pressed_last_frame = false;
  bool enter_pressed_last_frame = false;
  bool d_pressed_last_frame = false;
  
  std::cout << "Demo Instructions:" << std::endl;
  std::cout << "1. The robot will move close to the ball" << std::endl;
  std::cout << "2. After 3 seconds, the robot will start dribbling" << std::endl;
  std::cout << "3. Robot will dribble for 8 seconds with ball control!" << std::endl;
  std::cout << "4. Press SPACE or 1 to kick, D to toggle dribble, ENTER to reset ball position" << std::endl;
  std::cout << "5. Press ESC to exit" << std::endl;

  // Main simulation loop
  while (true) {
    double current_time = util::GetCurrentTime();
    double dt = util::CalculateDt();
    
    if (!demo_started) {
      demo_start_time = current_time;
      demo_started = true;
      
      // Position robot near the ball for demonstration
      for (auto& obj : soccer_objects) {
        if (obj.name == "robot_0") {
          obj.position = Eigen::Vector3d(-0.4, 0, 0);  // Position robot 0.4m to the left of ball
        }
        if (obj.name == "ball") {
          obj.position = Eigen::Vector3d(0, 0, 0);     // Ball at origin
          obj.velocity = Eigen::Vector3d::Zero();      // Ball stationary
        }
      }
    }
    
    // Move robot toward ball for the first few seconds
    if (!ball_approached && (current_time - demo_start_time) < approach_time) {
      // Move robot close to ball
      robot_manager.AddGoal(Eigen::Vector3d(-0.15, 0, 0));  // Get close to ball for dribbling
    }
    
    // Start dribbling after approach time
    if (!ball_approached && (current_time - demo_start_time) >= approach_time) {
      std::cout << "[Demo] Robot is now starting to dribble the ball!" << std::endl;
      robot_manager.DribbleBall();
      ball_approached = true;
      is_dribbling = true;
    }
    
    // Stop dribbling after dribble time
    if (is_dribbling && (current_time - demo_start_time) >= (approach_time + dribble_time)) {
      std::cout << "[Demo] Dribbling demo phase completed!" << std::endl;
      robot_manager.SetRobotAction(rob::RobotAction::MOVE);
      is_dribbling = false;
    }
    
    // Handle input
    vis::ProcessInput(gl_simulation.GetRawGLFW(), soccer_objects);
    
    // Check for manual kick (spacebar or 1 key)
    if (glfwGetKey(gl_simulation.GetRawGLFW(), GLFW_KEY_SPACE) == GLFW_PRESS || 
        glfwGetKey(gl_simulation.GetRawGLFW(), GLFW_KEY_1) == GLFW_PRESS) {
      if (!space_pressed_last_frame) {
        std::cout << "[Demo] Manual kick triggered!" << std::endl;
        robot_manager.KickBall();
        is_dribbling = false;
      }
      space_pressed_last_frame = true;
    } else {
      space_pressed_last_frame = false;
    }
    
    // Check for manual dribble toggle (D key)
    if (glfwGetKey(gl_simulation.GetRawGLFW(), GLFW_KEY_D) == GLFW_PRESS) {
      if (!d_pressed_last_frame) {
        if (is_dribbling) {
          std::cout << "[Demo] Stopping dribble!" << std::endl;
          robot_manager.SetRobotAction(rob::RobotAction::MOVE);
          is_dribbling = false;
        } else {
          std::cout << "[Demo] Starting dribble!" << std::endl;
          robot_manager.DribbleBall();
          is_dribbling = true;
        }
      }
      d_pressed_last_frame = true;
    } else {
      d_pressed_last_frame = false;
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
      ball_approached = false;
      is_dribbling = false;
      demo_start_time = current_time;
      robot_manager.SetRobotAction(rob::RobotAction::MOVE);
      }
      enter_pressed_last_frame = true;
    } else {
      enter_pressed_last_frame = false;
    }
    
    // Execute robot actions (including kicks and dribbles)
    if (robot_manager.GetRobotAction() == rob::RobotAction::KICK_BALL) {
      std::cout << "[Demo] Executing kick action!" << std::endl;
      robot_manager.ExecuteKickAction(soccer_objects);
    } else if (robot_manager.GetRobotAction() == rob::RobotAction::DRIBBLE_BALL) {
      // Execute dribble action continuously
      robot_manager.ExecuteDribbleAction(soccer_objects);
    }
    
    // Update robot logic
    robot_manager.ControlLogic();
    robot_manager.SenseLogic();
    
    // Update physics (including advanced ball physics)
    kin::UpdateKinematics(soccer_objects, dt);
    kin::CheckAndResolveCollisions(soccer_objects);
    
    // Update robot state from soccer objects
    for (const auto& obj : soccer_objects) {
      if (obj.name == "robot_0") {
        soccer_objects[0].position = robot_manager.GetPoseInWorldFrame();
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
    if (current_time - last_print_time > 2.0) {  // Print every 2 seconds
      std::cout << "[Debug] Soccer objects found:" << std::endl;
      for (const auto& obj : soccer_objects) {
        std::cout << "  - Name: '" << obj.name << "', Position: (" << obj.position[0] << ", " << obj.position[1] << ")" << std::endl;
        if (obj.name == "ball") {
          double speed = std::sqrt(obj.velocity[0]*obj.velocity[0] + obj.velocity[1]*obj.velocity[1]);
          std::cout << "[Demo] Ball position: (" << obj.position[0] << ", " << obj.position[1] 
                    << "), Speed: " << speed << " m/s, Dribbling: " << (is_dribbling ? "YES" : "NO") << std::endl;
        }
      }
      last_print_time = current_time;
    }
    
  }

  std::cout << "[DemoBallPhysicsAndDribbling] Demo finished!" << std::endl;
  return 0;
}