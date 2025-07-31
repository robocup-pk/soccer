#include <iostream>
#include <vector>
#include <fstream>
#include "Waypoint.h"
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "RobotManager.h"
#include "Utils.h"
#include "RRTX.h"

using namespace std;

void LogTrajectoryData(const string& filename, double time, const Eigen::Vector3d& pose, 
                      const Eigen::Vector3d& velocity) {
    static ofstream file(filename);
    if (file.is_open()) {
        file << time << "," 
             << pose[0] << "," << pose[1] << "," << pose[2] << ","
             << velocity[0] << "," << velocity[1] << "," << velocity[2] << ","
             << velocity.head<2>().norm() << endl;
    }
}

int main(int argc, char* argv[]) {
    std::cout << "[Demo] Trajectory Comparison Demo" << std::endl;
    std::cout << "Comparing different trajectory planners with same waypoints" << std::endl;

    // Choose trajectory type
    int traj_type = 1;
    if (argc > 1) {
        traj_type = std::atoi(argv[1]);
    }
    
    bool headless = false;
    if (argc > 2 && std::string(argv[2]) == "headless") {
        headless = true;
    }

    // Initialize objects
    vector<state::SoccerObject> soccer_objects;
    state::InitSoccerObjects(soccer_objects);
    vis::GLSimulation gl_simulation;
    if (!headless) {
        gl_simulation.InitGameObjects(soccer_objects);
    }
    
    // Initialize RobotManager
    rob::RobotManager robot_manager;
    
    // Set initial robot pose
    Eigen::Vector3d robot_start_pose(0.0, 0.0, 0.0);
    robot_manager.InitializePose(robot_start_pose);
    
    // Create RRT*-like waypoints (typical output)
    vector<Eigen::Vector3d> waypoints;
    waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    waypoints.push_back(Eigen::Vector3d(0.15, 0.05, 0.2));
    waypoints.push_back(Eigen::Vector3d(0.35, 0.08, 0.3));
    waypoints.push_back(Eigen::Vector3d(0.48, 0.22, 0.6));
    waypoints.push_back(Eigen::Vector3d(0.65, 0.45, 0.9));
    waypoints.push_back(Eigen::Vector3d(0.8, 0.6, 0.7));
    waypoints.push_back(Eigen::Vector3d(1.0, 0.8, 0.5));
    
    // Print waypoints
    std::cout << "RRT*-like Waypoints:" << std::endl;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "  " << i << ": (" << waypoints[i][0] << ", " 
                  << waypoints[i][1] << ", " << waypoints[i][2] << ")" << std::endl;
    }
    
    // Set trajectory based on type
    string filename;
    string planner_name;
    
    switch (traj_type) {
        case 1:
            planner_name = "B-spline (Smooth)";
            filename = "bspline_trajectory.csv";
            std::cout << "\nUsing B-spline trajectory for maximum smoothness" << std::endl;
            robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
            robot_manager.SetBSplinePath(waypoints, util::GetCurrentTime());
            break;
        case 2:
            planner_name = "Hermite Spline";
            filename = "hermite_trajectory.csv";
            std::cout << "\nUsing Hermite spline trajectory" << std::endl;
            robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::HermiteSpline);
            robot_manager.SetHermiteSplinePath(waypoints, util::GetCurrentTime());
            break;
        case 3:
            planner_name = "Pure Pursuit";
            filename = "purepursuit_trajectory.csv";
            std::cout << "\nUsing Pure Pursuit trajectory" << std::endl;
            robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::PurePursuit);
            robot_manager.SetPurePursuitPath(waypoints, util::GetCurrentTime());
            break;
        case 4:
            planner_name = "BangBang";
            filename = "bangbang_trajectory.csv";
            std::cout << "\nUsing BangBang trajectory" << std::endl;
            robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::BangBang);
            robot_manager.SetMPath(waypoints, util::GetCurrentTime());
            break;
        default:
            planner_name = "Original";
            filename = "original_trajectory.csv";
            std::cout << "\nUsing original trajectory manager" << std::endl;
            robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::ORIGINAL);
            robot_manager.SetPath(waypoints, util::GetCurrentTime());
            break;
    }
    
    // Write header to file
    ofstream header_file(filename);
    header_file << "time,x,y,theta,vx,vy,vtheta,linear_speed" << endl;
    header_file.close();
    
    // Variables for analysis
    double max_velocity = 0.0;
    double max_acceleration = 0.0;
    double total_jerk = 0.0;
    int velocity_violations = 0;
    double start_time = util::GetCurrentTime();
    
    Eigen::Vector3d prev_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d prev_acceleration = Eigen::Vector3d::Zero();
    
    // Run simulation
    while (true) {
        double current_time = util::GetCurrentTime();
        double elapsed_time = current_time - start_time;
        
        // Stop after 20 seconds or if trajectory is finished
        if (elapsed_time > 20.0 || robot_manager.GetRobotState() == "IDLE") {
            if (robot_manager.GetRobotState() == "IDLE") {
                std::cout << "\nTrajectory completed at t=" << elapsed_time << "s" << std::endl;
            } else {
                std::cout << "\nTime limit reached" << std::endl;
            }
            break;
        }
        
        // Run simulation step
        if (!headless) {
            if (!gl_simulation.RunSimulationStep(soccer_objects, util::CalculateDt())) {
                std::cout << "[Demo] Simulation window closed" << std::endl;
                break;
            }
            vis::ProcessInput(gl_simulation.GetRawGLFW(), soccer_objects);
        }
        
        // Control logic for RobotManager
        robot_manager.ControlLogic();
        robot_manager.SenseLogic();
        
        // Get current state
        Eigen::Vector3d pose = robot_manager.GetPoseInWorldFrame();
        Eigen::Vector3d velocity = robot_manager.GetVelocityInWorldFrame();
        
        // Log data
        LogTrajectoryData(filename, elapsed_time, pose, velocity);
        
        // Calculate metrics
        double linear_speed = velocity.head<2>().norm();
        if (linear_speed > max_velocity) {
            max_velocity = linear_speed;
        }
        
        if (linear_speed > 1.0 || std::abs(velocity[2]) > 5.0) {
            velocity_violations++;
        }
        
        // Calculate acceleration (numerical differentiation)
        double dt = util::CalculateDt();
        if (dt > 0) {
            Eigen::Vector3d acceleration = (velocity - prev_velocity) / dt;
            double linear_accel = acceleration.head<2>().norm();
            if (linear_accel > max_acceleration) {
                max_acceleration = linear_accel;
            }
            
            // Calculate jerk
            if (elapsed_time > dt) {
                Eigen::Vector3d jerk = (acceleration - prev_acceleration) / dt;
                total_jerk += jerk.norm() * dt;
            }
            prev_acceleration = acceleration;
        }
        prev_velocity = velocity;
        
        // Update soccer objects
        soccer_objects[0].position = pose;
        soccer_objects[0].velocity = velocity;
        
        // Print status every second
        static double last_print_time = 0;
        if (elapsed_time - last_print_time > 1.0) {
            std::cout << "[" << planner_name << "] t=" << elapsed_time 
                      << "s, pos=(" << pose[0] << ", " << pose[1] 
                      << "), speed=" << linear_speed << " m/s" << std::endl;
            last_print_time = elapsed_time;
        }
        
        if (headless) {
            util::WaitMs(20);  // 50Hz update
        }
    }
    
    // Print analysis results
    std::cout << "\n========== Trajectory Analysis ==========" << std::endl;
    std::cout << "Planner: " << planner_name << std::endl;
    std::cout << "Max velocity: " << max_velocity << " m/s" << std::endl;
    std::cout << "Max acceleration: " << max_acceleration << " m/s²" << std::endl;
    std::cout << "Total jerk integral: " << total_jerk << " m/s³·s" << std::endl;
    std::cout << "Velocity violations: " << velocity_violations << std::endl;
    std::cout << "Data saved to: " << filename << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}