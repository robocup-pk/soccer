#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>

#include "GLSimulation.h"
#include "SoccerObject.h"
#include "BallModel.h"
#include "RobotManager.h"
#include "SystemConfig.h"

int main(int argc, char *argv[]) {
  std::cout << "=== Ball Physics Demo ===" << std::endl;
  std::cout << "This demo shows realistic ball physics using BallModel" << std::endl;
  std::cout << "Use arrow keys to move the robot" << std::endl;
  std::cout << "Press ENTER to reset ball position" << std::endl;
  std::cout << "Press ESC to exit" << std::endl;
  
  // Initialize soccer objects
  std::vector<state::SoccerObject> soccer_objects;
  state::InitSoccerObjects(soccer_objects);
  
  // Create robot manager for robot control
  rob::RobotManager robot_manager;
  robot_manager.InitializePose(Eigen::Vector3d(0.5, 0, 0));
  
  // Initialize visualization
  vis::GLSimulation gl_simulation;
  gl_simulation.Init();
  
  // Main simulation loop
  while (!gl_simulation.ShouldClose()) {
    auto current_time = std::chrono::steady_clock::now();
    
    // Handle input
    vis::ProcessInput(gl_simulation.GetRawGLFW(), soccer_objects);
    
    // Check for ball reset (enter key)
    static bool enter_pressed_last_frame = false;
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
      }
      enter_pressed_last_frame = true;
    } else {
      enter_pressed_last_frame = false;
    }
    
    // Update robot position in soccer objects
    for (auto& obj : soccer_objects) {
      if (obj.name == "robot0") {
        obj.position = robot_manager.GetPoseInWorldFrame();
        obj.velocity = robot_manager.GetBodyVelocity();
      }
    }
    
    // Update ball physics
    for (auto& obj : soccer_objects) {
      if (obj.name == "ball") {
        // Apply basic physics simulation
        const float dt = 0.016f; // 60 FPS
        obj.Move(dt);
        
        // Apply friction to slow down the ball
        obj.velocity *= 0.98f;
        
        // Boundary checking
        const float field_width = 6.0f;
        const float field_height = 4.0f;
        if (std::abs(obj.position.x()) > field_width/2) {
          obj.position.x() = (obj.position.x() > 0) ? field_width/2 : -field_width/2;
          obj.velocity.x() *= -0.8f; // Bounce with energy loss
        }
        if (std::abs(obj.position.y()) > field_height/2) {
          obj.position.y() = (obj.position.y() > 0) ? field_height/2 : -field_height/2;
          obj.velocity.y() *= -0.8f; // Bounce with energy loss
        }
      }
    }
    
    // Update robot logic
    robot_manager.ControlLogic();
    robot_manager.SenseLogic();
    
    // Render the scene
    gl_simulation.Render(soccer_objects);
    
    // Control frame rate (60 FPS)
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }
  
  std::cout << "[Demo] Shutting down..." << std::endl;
  return 0;
}