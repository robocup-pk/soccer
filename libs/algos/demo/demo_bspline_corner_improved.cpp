#include <iostream>
#include <vector>
#include <fstream>
#include <memory>
#include <chrono>
#include <thread>
#include <cmath>
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "RobotManager.h"
#include "Utils.h"
#include "BSplineTrajectory.h"
#include "BSplineTrajectoryManager.h"

// Improved waypoint preprocessing for better corner handling
std::vector<Eigen::Vector3d> PreprocessWaypointsForCorners(const std::vector<Eigen::Vector3d>& waypoints, 
                                                           double corner_radius = 0.15) {
    std::vector<Eigen::Vector3d> processed;
    
    if (waypoints.size() < 3) {
        return waypoints; // Not enough points for corner processing
    }
    
    // Always add the first waypoint
    processed.push_back(waypoints[0]);
    
    // Process interior waypoints
    for (size_t i = 1; i < waypoints.size() - 1; ++i) {
        Eigen::Vector3d prev = waypoints[i-1];
        Eigen::Vector3d curr = waypoints[i];
        Eigen::Vector3d next = waypoints[i+1];
        
        // Calculate direction vectors
        Eigen::Vector2d v1 = (curr - prev).head<2>();
        Eigen::Vector2d v2 = (next - curr).head<2>();
        
        double len1 = v1.norm();
        double len2 = v2.norm();
        
        if (len1 > 2*corner_radius && len2 > 2*corner_radius) {
            // Normalize directions
            v1 /= len1;
            v2 /= len2;
            
            // Calculate angle between segments
            double dot = v1.dot(v2);
            double cross = v1[0]*v2[1] - v1[1]*v2[0];
            double angle = std::atan2(cross, dot);
            
            // If significant angle change, add corner waypoints
            if (std::abs(angle) > M_PI/6) { // More than 30 degrees
                // Add point before corner
                Eigen::Vector3d before = curr;
                before.head<2>() -= v1 * corner_radius;
                before[2] = prev[2]; // Keep previous orientation
                processed.push_back(before);
                
                // Add corner midpoint with interpolated angle
                if (std::abs(angle) > M_PI/2) { // Sharp corner
                    // Add extra point for very sharp corners
                    Eigen::Vector3d mid = curr;
                    mid[2] = prev[2] + angle/2; // Halfway rotation
                    processed.push_back(mid);
                }
                
                // Add point after corner
                Eigen::Vector3d after = curr;
                after.head<2>() += v2 * corner_radius;
                after[2] = next[2]; // Target orientation
                processed.push_back(after);
            } else {
                // Small angle, just add the waypoint
                processed.push_back(curr);
            }
        } else {
            // Segments too short, add waypoint as is
            processed.push_back(curr);
        }
    }
    
    // Always add the last waypoint
    processed.push_back(waypoints.back());
    
    return processed;
}

// Save trajectory data for plotting
void SaveImprovedTrajectoryData(const std::vector<Eigen::Vector3d>& original_waypoints,
                                const std::vector<Eigen::Vector3d>& processed_waypoints,
                                const std::vector<Eigen::Vector3d>& bspline_points,
                                const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    
    // Write original waypoints
    file << "# Original RRT* Waypoints\n";
    for (const auto& wp : original_waypoints) {
        file << "ORIGINAL," << wp[0] << "," << wp[1] << "," << wp[2] << "\n";
    }
    
    // Write processed waypoints
    file << "# Processed Waypoints (with corner handling)\n";
    for (const auto& wp : processed_waypoints) {
        file << "PROCESSED," << wp[0] << "," << wp[1] << "," << wp[2] << "\n";
    }
    
    // Write B-spline points
    file << "# B-spline Trajectory\n";
    for (const auto& pt : bspline_points) {
        file << "BSPLINE," << pt[0] << "," << pt[1] << "," << pt[2] << "\n";
    }
    
    file.close();
    std::cout << "Saved improved trajectory data to " << filename << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "[Improved B-Spline Corner Handling Demo]" << std::endl;
    std::cout << "This demo shows improved B-spline behavior at corners" << std::endl;
    
    // Test case selection
    int test_case = 1;
    if (argc > 1) {
        test_case = std::atoi(argv[1]);
    }
    
    std::vector<Eigen::Vector3d> waypoints;
    
    switch (test_case) {
        case 1: {
            // Sharp 90-degree turns
            std::cout << "\nTest 1: Improved handling of 90-degree corners" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 1.0, M_PI/2));
            waypoints.push_back(Eigen::Vector3d(0.0, 1.0, M_PI));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, -M_PI/2));
            break;
        }
        case 2: {
            // Single 90-degree corner for clear visualization
            std::cout << "\nTest 2: Single corner analysis" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 1.0, M_PI/2));
            break;
        }
        case 3: {
            // Multiple corner types
            std::cout << "\nTest 3: Various corner angles" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.3, M_PI/6));  // 30 degree
            waypoints.push_back(Eigen::Vector3d(1.5, 0.0, -M_PI/6));
            waypoints.push_back(Eigen::Vector3d(2.0, 0.5, M_PI/4));  // 45 degree
            waypoints.push_back(Eigen::Vector3d(2.5, 0.0, -M_PI/4));
            waypoints.push_back(Eigen::Vector3d(3.0, 1.0, M_PI/2));  // 90 degree
            break;
        }
    }
    
    // Print original waypoints
    std::cout << "\nOriginal waypoints:" << std::endl;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "  " << i << ": [" << waypoints[i][0] << ", " 
                  << waypoints[i][1] << ", " << waypoints[i][2] << " rad]" << std::endl;
    }
    
    // Preprocess waypoints for better corner handling
    std::vector<Eigen::Vector3d> processed_waypoints = PreprocessWaypointsForCorners(waypoints);
    
    std::cout << "\nProcessed waypoints (with corner handling):" << std::endl;
    for (size_t i = 0; i < processed_waypoints.size(); ++i) {
        std::cout << "  " << i << ": [" << processed_waypoints[i][0] << ", " 
                  << processed_waypoints[i][1] << ", " << processed_waypoints[i][2] << " rad]" << std::endl;
    }
    
    // Create B-spline with processed waypoints
    auto bspline_trajectory = std::make_unique<ctrl::BSplineTrajectory>();
    
    // Use lower degree for tighter corner following
    bspline_trajectory->SetSplineDegree(2); // Quadratic B-spline
    
    double start_time = util::GetCurrentTime();
    bspline_trajectory->SetPath(processed_waypoints, start_time);
    
    // Sample the B-spline
    std::vector<Eigen::Vector3d> bspline_points;
    double dt = 0.01;
    double max_time = 10.0;
    Eigen::Vector3d current_pose = waypoints[0];
    
    for (double t = 0; t <= max_time; t += dt) {
        double sample_time = start_time + t;
        Eigen::Vector3d vel = bspline_trajectory->Update(current_pose, sample_time);
        
        if (bspline_trajectory->IsFinished()) {
            break;
        }
        
        // Integrate to get position
        current_pose += vel * dt;
        bspline_points.push_back(current_pose);
    }
    
    std::cout << "\nGenerated " << bspline_points.size() << " B-spline points" << std::endl;
    
    // Save data
    std::string filename = "improved_bspline_test" + std::to_string(test_case) + ".csv";
    SaveImprovedTrajectoryData(waypoints, processed_waypoints, bspline_points, filename);
    
    // Create visualization
    std::vector<state::SoccerObject> soccer_objects;
    
    // Robot
    soccer_objects.push_back(
        state::SoccerObject("robot0", waypoints[0], 
                           Eigen::Vector2d(0.18, 0.18),
                           Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(), 10));
    
    // Add markers for original waypoints (red)
    for (size_t i = 0; i < waypoints.size(); ++i) {
        soccer_objects.push_back(
            state::SoccerObject("orig" + std::to_string(i), waypoints[i],
                               Eigen::Vector2d(0.06, 0.06),
                               Eigen::Vector3d::Zero(),
                               Eigen::Vector3d::Zero(), 0.01f));
    }
    
    // Add markers for processed waypoints (green) - offset slightly for visibility
    for (size_t i = 0; i < processed_waypoints.size(); ++i) {
        Eigen::Vector3d pos = processed_waypoints[i];
        pos[0] += 0.02;
        pos[1] += 0.02;
        soccer_objects.push_back(
            state::SoccerObject("proc" + std::to_string(i), pos,
                               Eigen::Vector2d(0.04, 0.04),
                               Eigen::Vector3d::Zero(),
                               Eigen::Vector3d::Zero(), 0.01f));
    }
    
    auto gl_sim = std::make_unique<vis::GLSimulation>();
    gl_sim->InitGameObjects(soccer_objects);
    
    // Create robot for animation
    auto robot = std::make_unique<rob::RobotManager>();
    robot->SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
    
    // Use processed waypoints
    robot->SetBSplinePath(processed_waypoints, util::GetCurrentTime());
    
    std::cout << "\n=== VISUALIZATION ===" << std::endl;
    std::cout << "Large red dots: Original waypoints (sharp corners)" << std::endl;
    std::cout << "Small green dots: Processed waypoints (corner handling)" << std::endl;
    std::cout << "Robot path: Improved B-spline trajectory" << std::endl;
    std::cout << "\nPress ESC to exit" << std::endl;
    
    // Run visualization
    while (gl_sim) {
        robot->ControlLogic();
        robot->SenseLogic();
        
        soccer_objects[0].position = robot->GetPoseInWorldFrame();
        
        if (!gl_sim->RunSimulationStep(soccer_objects, 0.016)) {
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    
    std::cout << "\n[Demo Complete] - Check " << filename << " for data" << std::endl;
    
    return 0;
}