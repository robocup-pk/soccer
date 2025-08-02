#include <iostream>
#include <vector>
#include <fstream>
#include <memory>
#include <chrono>
#include <thread>
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "RobotManager.h"
#include "Utils.h"
#include "BSplineTrajectory.h"
#include "BSplineTrajectoryManager.h"

// Function to save trajectory data for plotting
void SaveTrajectoryData(const std::vector<Eigen::Vector3d>& rrt_waypoints,
                        const std::vector<Eigen::Vector3d>& bspline_points,
                        const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    
    // Write RRT* waypoints
    file << "# RRT* Waypoints\n";
    for (const auto& wp : rrt_waypoints) {
        file << "RRT," << wp[0] << "," << wp[1] << "," << wp[2] << "\n";
    }
    
    // Write B-spline points
    file << "# B-spline Trajectory\n";
    for (const auto& pt : bspline_points) {
        file << "BSPLINE," << pt[0] << "," << pt[1] << "," << pt[2] << "\n";
    }
    
    file.close();
    std::cout << "Saved trajectory data to " << filename << std::endl;
}

// Sample B-spline trajectory at regular intervals
std::vector<Eigen::Vector3d> SampleBSplineTrajectory(ctrl::BSplineTrajectory& bspline, 
                                                     double duration, 
                                                     double dt = 0.01) {
    std::vector<Eigen::Vector3d> points;
    double start_time = util::GetCurrentTime();
    
    // Create a dummy pose for sampling
    Eigen::Vector3d current_pose(0, 0, 0);
    
    for (double t = 0; t <= duration; t += dt) {
        double sample_time = start_time + t;
        
        // Get the position at this time
        // We'll use the velocity to integrate position
        if (t == 0) {
            points.push_back(current_pose);
        } else {
            Eigen::Vector3d vel = bspline.Update(current_pose, sample_time);
            current_pose += vel * dt;
            points.push_back(current_pose);
        }
        
        // Stop if trajectory is finished
        if (bspline.IsFinished()) {
            break;
        }
    }
    
    return points;
}

int main(int argc, char* argv[]) {
    std::cout << "[RRT* + B-Spline Visualization Demo]" << std::endl;
    std::cout << "This demo shows how B-spline handles corners from RRT* waypoints" << std::endl;
    
    // Initialize visualization
    std::vector<state::SoccerObject> soccer_objects;
    
    // Robot
    soccer_objects.push_back(
        state::SoccerObject("robot0", Eigen::Vector3d(0, 0, 0), 
                           Eigen::Vector2d(0.18, 0.18),
                           Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(), 10));
    
    // Goal
    soccer_objects.push_back(
        state::SoccerObject("ball", Eigen::Vector3d(2, 2, 0),
                           Eigen::Vector2d(0.086, 0.086),
                           Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(), 0.046f));
    
    // Obstacles for more interesting RRT* paths
    soccer_objects.push_back(
        state::SoccerObject("robot1", Eigen::Vector3d(1, 0.5, 0),
                           Eigen::Vector2d(0.18, 0.18),
                           Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(), 10));
    
    soccer_objects.push_back(
        state::SoccerObject("robot2", Eigen::Vector3d(0.5, 1.5, 0),
                           Eigen::Vector2d(0.18, 0.18),
                           Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(), 10));
    
    soccer_objects.push_back(
        state::SoccerObject("robot3", Eigen::Vector3d(1.5, 1, 0),
                           Eigen::Vector2d(0.18, 0.18),
                           Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(), 10));
    
    auto gl_sim = std::make_unique<vis::GLSimulation>();
    gl_sim->InitGameObjects(soccer_objects);
    
    // Test case selection
    int test_case = 1;
    if (argc > 1) {
        test_case = std::atoi(argv[1]);
    }
    
    std::vector<Eigen::Vector3d> waypoints;
    
    switch (test_case) {
        case 1: {
            // Sharp 90-degree turns
            std::cout << "\nTest 1: Sharp 90-degree corners" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 1.0, M_PI/2));
            waypoints.push_back(Eigen::Vector3d(0.0, 1.0, M_PI));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, -M_PI/2));
            break;
        }
        case 2: {
            // Zigzag path
            std::cout << "\nTest 2: Zigzag path with sharp corners" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.5, M_PI/4));
            waypoints.push_back(Eigen::Vector3d(1.5, 0.0, -M_PI/4));
            waypoints.push_back(Eigen::Vector3d(2.0, 0.5, M_PI/4));
            waypoints.push_back(Eigen::Vector3d(2.5, 0.0, 0.0));
            break;
        }
        case 3: {
            // Simulated RRT* path with obstacle avoidance
            std::cout << "\nTest 3: Simulated RRT* path around obstacles" << std::endl;
            
            // Create a path that would be typical of RRT* avoiding obstacles
            // This path has many sharp turns typical of RRT* output
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.3, 0.3, M_PI/4));
            waypoints.push_back(Eigen::Vector3d(0.0, 1.0, M_PI/2));
            waypoints.push_back(Eigen::Vector3d(0.3, 1.8, M_PI/4));
            waypoints.push_back(Eigen::Vector3d(1.0, 2.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(2.0, 2.0, 0.0));
            break;
        }
        case 4: {
            // Very sharp hairpin turns
            std::cout << "\nTest 4: Hairpin turns (180-degree corners)" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.2, M_PI));  // Sharp U-turn
            waypoints.push_back(Eigen::Vector3d(0.0, 0.2, M_PI));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.4, 0.0));   // Another U-turn
            waypoints.push_back(Eigen::Vector3d(1.0, 0.4, 0.0));
            break;
        }
    }
    
    // Print waypoints
    std::cout << "\nRRT*/Manual Waypoints:" << std::endl;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "  Waypoint " << i << ": [" 
                  << waypoints[i][0] << ", " 
                  << waypoints[i][1] << ", " 
                  << waypoints[i][2] << " rad]" << std::endl;
    }
    
    // Create B-spline trajectory
    auto bspline_trajectory = std::make_unique<ctrl::BSplineTrajectory>();
    double start_time = util::GetCurrentTime();
    bspline_trajectory->SetPath(waypoints, start_time);
    
    // Sample the B-spline trajectory
    std::cout << "\nSampling B-spline trajectory..." << std::endl;
    double trajectory_duration = 10.0;  // Assume 10 seconds max
    std::vector<Eigen::Vector3d> bspline_points = SampleBSplineTrajectory(*bspline_trajectory, 
                                                                          trajectory_duration);
    
    std::cout << "Generated " << bspline_points.size() << " B-spline points" << std::endl;
    
    // Save data for external plotting
    std::string filename = "rrt_bspline_trajectory_test" + std::to_string(test_case) + ".csv";
    SaveTrajectoryData(waypoints, bspline_points, filename);
    
    // Create robot manager for visualization
    auto robot = std::make_unique<rob::RobotManager>();
    robot->SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
    robot->SetBSplinePath(waypoints, util::GetCurrentTime());
    
    // Add visual markers for waypoints
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::string name = "waypoint" + std::to_string(i);
        soccer_objects.push_back(
            state::SoccerObject(name, waypoints[i],
                               Eigen::Vector2d(0.05, 0.05),  // Small markers
                               Eigen::Vector3d::Zero(),
                               Eigen::Vector3d::Zero(), 0.01f));
    }
    
    // Visualization info
    std::cout << "\n=== VISUALIZATION ===" << std::endl;
    std::cout << "Red dots: RRT*/Manual waypoints (sharp corners)" << std::endl;
    std::cout << "Robot trajectory: B-spline smoothed path" << std::endl;
    std::cout << "Notice how B-spline cuts corners to maintain smoothness" << std::endl;
    std::cout << "\nPress ESC to exit visualization" << std::endl;
    
    // Main visualization loop
    int frame = 0;
    double last_print_time = util::GetCurrentTime();
    
    while (gl_sim) {
        // Update robot
        robot->ControlLogic();
        robot->SenseLogic();
        
        // Update robot position in visualization
        soccer_objects[0].position = robot->GetPoseInWorldFrame();
        
        // Print status periodically
        double current_time = util::GetCurrentTime();
        if (current_time - last_print_time > 1.0) {
            Eigen::Vector3d pose = robot->GetPoseInWorldFrame();
            std::cout << "Robot at: [" << pose[0] << ", " << pose[1] 
                      << ", " << pose[2] << " rad]" << std::endl;
            last_print_time = current_time;
        }
        
        // Run visualization
        if (!gl_sim->RunSimulationStep(soccer_objects, 0.016)) {
            break;
        }
        
        frame++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    
    std::cout << "\n[Demo Complete]" << std::endl;
    std::cout << "Check " << filename << " for trajectory data" << std::endl;
    
    return 0;
}