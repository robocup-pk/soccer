#include <iostream>
#include <vector>
#include "BallModel.h"
#include "Kinematics.h"
#include "SoccerObject.h"

// Test case 1: Basic ball physics with no external forces
void TestBasicBallPhysics() {
    std::cout << "\n=== Test 1: Basic Ball Physics ===" << std::endl;
    
    kin::BallModel ball_model;
    
    // Create a ball with initial velocity
    Eigen::Vector3d position(0, 0, 0);
    Eigen::Vector3d velocity(3, 2, 0);  // 3 m/s in x, 2 m/s in y
    Eigen::Vector3d acceleration(0, 0, 0);
    
    std::cout << "Initial: pos(" << position.x() << ", " << position.y() 
              << "), vel(" << velocity.x() << ", " << velocity.y() << ")" << std::endl;
    
    // Update physics for 5 steps (0.1s each)
    double dt = 0.1;
    for (int i = 0; i < 5; i++) {
        ball_model.UpdatePhysics(position, velocity, acceleration, dt);
        std::cout << "Step " << i+1 << ": pos(" << position.x() << ", " << position.y() 
                  << "), vel(" << velocity.x() << ", " << velocity.y() << ")" << std::endl;
    }
}

// Test case 2: Apply kick to stationary ball
void TestKickApplication() {
    std::cout << "\n=== Test 2: Kick Application ===" << std::endl;
    
    kin::BallModel ball_model;
    
    // Create stationary ball
    Eigen::Vector3d velocity(0, 0, 0);
    Eigen::Vector2d kick_direction(1, 0);  // Kick to the right
    double kick_power = 5.0;  // 5 m/s
    
    std::cout << "Before kick: vel(" << velocity.x() << ", " << velocity.y() << ")" << std::endl;
    
    ball_model.ApplyKick(velocity, kick_direction, kick_power);
    
    std::cout << "After kick: vel(" << velocity.x() << ", " << velocity.y() << ")" << std::endl;
    std::cout << "Expected: vel(5, 0)" << std::endl;
    
    // Verify kick was applied correctly
    if (std::abs(velocity.x() - 5.0) < 0.001 && std::abs(velocity.y()) < 0.001) {
        std::cout << "✅ Kick applied correctly!" << std::endl;
    } else {
        std::cout << "❌ Kick application failed!" << std::endl;
    }
}

// Test case 3: Integration with soccer objects
void TestSoccerObjectIntegration() {
    std::cout << "\n=== Test 3: Soccer Object Integration ===" << std::endl;
    
    // Create soccer objects
    std::vector<state::SoccerObject> soccer_objects;
    
    // Create ball
    state::SoccerObject ball;
    ball.name = "ball";
    ball.position = Eigen::Vector3d(0, 0, 0);
    ball.velocity = Eigen::Vector3d(0, 0, 0);
    ball.acceleration = Eigen::Vector3d(0, 0, 0);
    ball.size = Eigen::Vector3d(0.043, 0.043, 0.043);  // FIFA ball diameter
    ball.mass_kg = 0.43;  // FIFA ball mass
    soccer_objects.push_back(ball);
    
    // Create robot
    state::SoccerObject robot;
    robot.name = "robot_0";
    robot.position = Eigen::Vector3d(-0.1, 0, 0);  // 10cm from ball
    robot.velocity = Eigen::Vector3d(0, 0, 0);
    robot.size = Eigen::Vector3d(0.2, 0.2, 0.1);
    robot.mass_kg = 2.5;
    soccer_objects.push_back(robot);
    
    std::cout << "Initial ball pos: (" << soccer_objects[0].position.x() << ", " 
              << soccer_objects[0].position.y() << ")" << std::endl;
    std::cout << "Initial robot pos: (" << soccer_objects[1].position.x() << ", " 
              << soccer_objects[1].position.y() << ")" << std::endl;
    
    // Test ApplyKickToBall function directly
    Eigen::Vector2d kick_direction(1, 0);
    double kick_power = 5.0;
    
    std::cout << "Applying kick..." << std::endl;
    kin::ApplyKickToBall(soccer_objects[0], kick_direction, kick_power);
    
    std::cout << "Ball velocity after kick: (" << soccer_objects[0].velocity.x() << ", " 
              << soccer_objects[0].velocity.y() << ")" << std::endl;
    
    // Update physics for a few steps
    for (int i = 0; i < 3; i++) {
        kin::UpdateBallPhysics(soccer_objects, 0.1);
        std::cout << "Step " << i+1 << " ball pos: (" << soccer_objects[0].position.x() << ", " 
                  << soccer_objects[0].position.y() << "), vel: (" << soccer_objects[0].velocity.x() 
                  << ", " << soccer_objects[0].velocity.y() << ")" << std::endl;
    }
}

// Test case 4: Test with exact demo scenario
void TestDemoScenario() {
    std::cout << "\n=== Test 4: Demo Scenario ===" << std::endl;
    
    // Replicate exact demo conditions
    std::vector<state::SoccerObject> soccer_objects;
    state::InitSoccerObjects(soccer_objects);
    
    // Position robot close to ball (like in demo)
    for (auto& obj : soccer_objects) {
        if (obj.name == "robot_0") {
            obj.position = Eigen::Vector3d(-0.2, 0, 0);  // 20cm from ball
        }
        if (obj.name == "ball") {
            obj.position = Eigen::Vector3d(0, 0, 0);     // Ball at origin
            obj.velocity = Eigen::Vector3d::Zero();      // Ball stationary
        }
    }
    
    std::cout << "Demo setup complete" << std::endl;
    
    // Find objects
    state::SoccerObject* ball = nullptr;
    state::SoccerObject* robot = nullptr;
    
    for (auto& obj : soccer_objects) {
        if (obj.name == "ball") ball = &obj;
        else if (obj.name == "robot_0") robot = &obj;
    }
    
    if (!ball || !robot) {
        std::cout << "❌ Could not find ball or robot!" << std::endl;
        return;
    }
    
    // Calculate distance (same logic as ExecuteKickAction)
    Eigen::Vector3d ball_center = ball->GetCenterPosition();
    Eigen::Vector3d robot_center = robot->GetCenterPosition();
    double distance = std::sqrt(std::pow(ball_center.x() - robot_center.x(), 2) + 
                               std::pow(ball_center.y() - robot_center.y(), 2));
    
    std::cout << "Robot center: (" << robot_center.x() << ", " << robot_center.y() << ")" << std::endl;
    std::cout << "Ball center: (" << ball_center.x() << ", " << ball_center.y() << ")" << std::endl;
    std::cout << "Distance: " << distance << "m" << std::endl;
    
    if (distance < 0.3) {
        std::cout << "✅ Ball is within kicking range!" << std::endl;
        
        // Apply kick
        double robot_angle = robot->position[2];
        Eigen::Vector2d kick_direction(std::cos(robot_angle), std::sin(robot_angle));
        
        std::cout << "Robot angle: " << robot_angle << " rad" << std::endl;
        std::cout << "Kick direction: (" << kick_direction.x() << ", " << kick_direction.y() << ")" << std::endl;
        
        kin::ApplyKickToBall(*ball, kick_direction, 5.0);
        
        std::cout << "Ball velocity after kick: (" << ball->velocity.x() << ", " << ball->velocity.y() << ")" << std::endl;
    } else {
        std::cout << "❌ Ball is NOT within kicking range!" << std::endl;
    }
}

int main() {
    std::cout << "Ball Physics Test Suite" << std::endl;
    std::cout << "======================" << std::endl;
    
    TestBasicBallPhysics();
    TestKickApplication();
    TestSoccerObjectIntegration();
    TestDemoScenario();
    
    std::cout << "\n=== Test Suite Complete ===" << std::endl;
    return 0;
}