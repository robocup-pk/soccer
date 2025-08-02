// cpp std libs
#include <iostream>
#include <chrono>
#include <cmath>
// self libs
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Kinematics.h"
#include "Coordinates.h"
#include "Utils.h"
#include "RobotManager.h"

int main(int argc, char* argv[]) {
// Need two robots for this demo to run
if (cfg::SystemConfig::num_robots != 2) {
std::cout << "[DemoMotionController::Main] Set num_robots to 2. Exiting!" << std::endl;
return 0;
}

// std::cout << "Random Test 1 " << std::endl;
// START SIMULATION
std::vector<state::SoccerObject> soccer_objects;
state::InitSoccerObjects(soccer_objects);
vis::GLSimulation gl_simulation;
gl_simulation.InitGameObjects(soccer_objects);

std::vector<Eigen::Vector3d> path1, path2;
path1.push_back(Eigen::Vector3d(0, 0, 0));
path1.push_back(Eigen::Vector3d(0, 1, 0));
path1.push_back(Eigen::Vector3d(0.7, 0, M_PI / 1.2));
path2.push_back(Eigen::Vector3d(0, 0, 0));
path2.push_back(Eigen::Vector3d(1, 0, 0));
path2.push_back(Eigen::Vector3d(0, 0.7, -M_PI / 3.5));

// path.push_back(EigGetPoseInWorldFrameen::Vector3d(0, 0, 90 * M_PI / 180));
// path.push_back(Eigen::Vector3d(0, 1, 90 * M_PI / 180));
// path.push_back(Eigen::Vector3d(0, 1, 0 * M_PI / 180));
// path.push_back(Eigen::Vector3d(1, 1, 0 * M_PI / 180));
// path.push_back(Eigen::Vector3d(1, 1, -90 * M_PI / 180));
// path.push_back(Eigen::Vector3d(1, 0, -90 * M_PI / 180));
// path.push_back(Eigen::Vector3d(1, 0, -180 * M_PI / 180));
// path.push_back(Eigen::Vector3d(0, 0, -180 * M_PI / 180));
// path.push_back(Eigen::Vector3d(0, 0, 270 * M_PI / 180));
// path.push_back(Eigen::Vector3d(0, 0, 360 * M_PI / 180));

rob::RobotManager robot_manager1;
robot_manager1.SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
robot_manager1.SetBSplinePath(path1, util::GetCurrentTime());

rob::RobotManager robot_manager2;
robot_manager2.SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
robot_manager2.SetBSplinePath(path2, util::GetCurrentTime());
// ball
state::SoccerObject& ball = soccer_objects[2]; // assuming index 2 is the ball
ball.velocity = Eigen::Vector3d(1.0, 0.0, 0.0); // Move along +x at 1 m/s
ball.acceleration = Eigen::Vector3d::Zero(); // No acceleration
ball.is_attached = true;
ball.attached_to = &soccer_objects[0];

Eigen::Vector3d goal = Eigen::Vector3d(0.7, 0, 0.0);
while (1) {
// std::cout << "Object 2 name: " << soccer_objects[2].name << std::endl;
robot_manager1.ControlLogic();

// Sense logic for RobotManager
robot_manager1.SenseLogic();
soccer_objects[0].position = robot_manager1.GetPoseInWorldFrame();
robot_manager2.ControlLogic();

// Sense logic for RobotManager
robot_manager2.SenseLogic();
soccer_objects[1].position = robot_manager2.GetPoseInWorldFrame();

if (ball.is_attached && ball.attached_to) {
double angle = ball.attached_to->position[2];
double offset = 0.1; // distance in front of robot face
ball.position[0] = ball.attached_to->position[0] + offset * cos(angle);
ball.position[1] = ball.attached_to->position[1] + offset * sin(angle);
} else {
ball.Move(util::CalculateDt());
}
// Debug output
Eigen::Vector3d robot_pos = soccer_objects[0].position;
double distance_to_goal = (robot_pos.head<2>() - goal.head<2>()).norm();
std::cout << "Robot pos: " << robot_pos.transpose() 
          << ", Goal: " << goal.transpose() 
          << ", Distance: " << distance_to_goal << std::endl;

// Check if robot is close to goal (using 2D distance, 5cm tolerance)
if (distance_to_goal < 0.05 && ball.is_attached && ball.attached_to == &soccer_objects[0]) {
    std::cout << "Detaching ball at goal!" << std::endl;
    kin::DetachBall(ball, 1.5f);
}
float x = soccer_objects[1].position[0] - ball.position[0];
float y = soccer_objects[1].position[1] - ball.position[1];
float dis = sqrt(x * x + y * y);
if (!ball.is_attached && kin::IsBallInFrontOfRobot(soccer_objects[1], ball) && dis < 0.1f) {
kin::HandleBallSticking(soccer_objects[1], ball);
}

// Update physics and kinematics
kin::UpdateKinematics(soccer_objects, util::CalculateDt());

// simulation
if (!gl_simulation.RunSimulationStep(soccer_objects, util::CalculateDt())) {
std::cout << "[main] Simulation finished" << std::endl;
break;
}
}

return 0;
}