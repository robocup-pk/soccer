#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>

#include "GLSimulation.h"
#include "SoccerObject.h" 
#include "RRTX.h"
#include "Waypoint.h"
#include "RobotManager.h"
#include "Kinematics.h"
#include "Kick.h"
#include "Utils.h"
using namespace std;
int main(int argc, char* argv[]) {
    std::cout << "[KickDemo] Simple RRTX + TrajectoryManager + Kick Demo" << std::endl;
    
    if (cfg::SystemConfig::num_robots != 1) {
        std::cout << "[KickDemo] Set num_robots to 1. Exiting!" << std::endl;
        return 0;
    }

    // Initialize objects
    std::vector<state::SoccerObject> soccer_objects;
    state::InitSoccerObjects(soccer_objects);
    vis::GLSimulation gl_simulation;
    gl_simulation.InitGameObjects(soccer_objects);
    rob::RobotManager robot_manager;
    
    // Set positions
    Eigen::Vector3d robot_start_pose(0.0, 0.0, 0.0);     // Robot starts at origin
    Eigen::Vector3d ball_position(1.5, 0.5, 0.0);
    
    // Initialize robot pose
    Eigen::Vector3d robot_pose_ref = robot_start_pose;
    robot_manager.InitializePose(robot_pose_ref);
    
    for (auto& obj : soccer_objects) {
        if (obj.name == "robot0") {
            obj.position = robot_start_pose;
            obj.velocity = Eigen::Vector3d::Zero();
        } else if (obj.name == "ball") {
            obj.position = ball_position;
            obj.velocity = Eigen::Vector3d::Zero();
        }
    }
    //Give source and destination for RRTX
    state::Waypoint source(robot_start_pose[0], robot_start_pose[1], 0.0);
    state::Waypoint destination(ball_position[0], ball_position[1], 0.0);
    // Create RRTX instance
    state::Path path;
    // Find path from robot to ball
    path = algo::FindSinglePath_RRTX(source, destination);

    if (path.empty()) {
        std::cout << "[KickDemo] No path found from robot to ball!" << std::endl;
        return 1;
    }
    vector<Eigen::Vector3d> targetPath;
    for(int i = 0; i < path.size(); ++i){
        cout << "Waypoint " << i << ": (" << path[i].x << ", " << path[i].y << ", " << path[i].angle << ")" << std::endl;
    }
    // Convert path to Eigen::Vector3d format for RobotManager
    for(int i = 0; i < path.size(); ++i) {
        targetPath.push_back(Eigen::Vector3d(path[i].x * 1000, path[i].y * 1000, path[i].angle ));
    }
    // Add ball to robot manager's goal queue
    robot_manager.SetPath(targetPath);

    while (true) {

        // Check distance between robot and ball
        Eigen::Vector3d robot_pos = robot_manager.GetPoseInWorldFrame();
        double dist = sqrt((ball_position - robot_pos).head<2>().squaredNorm());
        cout<<"Distance: "<<dist<<endl;
        
        // Set kick action when close to ball - robot manager will execute it
        if (dist < 0.3) {  // Use same distance as ExecuteKickAction (30cm)
            cout<<"Setting kick action"<<endl;
            //robot_manager.KickBall();  // Set the action, let robot manager execute it
            robot_manager.ExecuteKickAction(soccer_objects);
            //Now after kick exit the screen
            std::cout << "Robot kicked the ball. Exiting demo." << std::endl;
        }
        
        double current_time = util::GetCurrentTime();
        double dt = util::CalculateDt();
        
        vis::ProcessInput(gl_simulation.GetRawGLFW(), soccer_objects); 
        // Update robot and physics
        robot_manager.ControlLogic();
        robot_manager.SenseLogic();
        
        // Sync robot position to soccer objects
        for (auto& obj : soccer_objects) {
            if (obj.name == "robot0") {
                obj.position = robot_manager.GetPoseInWorldFrame();
                obj.velocity = robot_manager.GetVelocityInWorldFrame();
                break;
            }
        }
        
        kin::UpdateKinematics(soccer_objects, dt);
        kin::CheckAndResolveCollisions(soccer_objects);
        
        if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
            break;
        }
    }

    std::cout << "[KickDemo] Demo finished!" << std::endl;
    return 0;
}