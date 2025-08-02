#include <iostream>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <algorithm>
#include "BSplineTrajectory.h"
#include "Utils.h"

int main() {
    // Original waypoints - square path
    std::vector<Eigen::Vector3d> waypoints;
    waypoints.push_back(Eigen::Vector3d(0, 0, 0));
    waypoints.push_back(Eigen::Vector3d(1, 0, 0));
    waypoints.push_back(Eigen::Vector3d(1, 1, M_PI/2));
    waypoints.push_back(Eigen::Vector3d(0, 1, M_PI));
    waypoints.push_back(Eigen::Vector3d(0, 0, -M_PI/2));
    
    std::cout << "Testing B-spline with original waypoints (no preprocessing)\n";
    std::cout << "Original waypoints:\n";
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "  " << i << ": [" << waypoints[i][0] << ", " 
                  << waypoints[i][1] << ", " << waypoints[i][2] << " rad]\n";
    }
    
    // Create B-spline trajectory
    ctrl::BSplineTrajectory bspline_trajectory;
    bspline_trajectory.SetSplineDegree(3); // Cubic B-spline
    
    double start_time = util::GetCurrentTime();
    if (!bspline_trajectory.SetPath(waypoints, start_time)) {
        std::cerr << "Failed to set B-spline path\n";
        return 1;
    }
    
    // Sample the trajectory
    std::vector<Eigen::Vector3d> trajectory_points;
    Eigen::Vector3d current_pose = waypoints[0];
    double dt = 0.01;
    
    while (!bspline_trajectory.IsFinished()) {
        double current_time = util::GetCurrentTime();
        Eigen::Vector3d vel = bspline_trajectory.Update(current_pose, current_time);
        
        if (bspline_trajectory.IsFinished()) {
            break;
        }
        
        current_pose += vel * dt;
        trajectory_points.push_back(current_pose);
        
        // Simulate time passing
        usleep(dt * 1000000); // Use POSIX usleep instead
    }
    
    // Save to file
    std::ofstream file("original_waypoints_trajectory.csv");
    file << "# Original waypoints\n";
    for (const auto& wp : waypoints) {
        file << "WAYPOINT," << wp[0] << "," << wp[1] << "," << wp[2] << "\n";
    }
    file << "# B-spline trajectory\n";
    for (const auto& pt : trajectory_points) {
        file << "TRAJECTORY," << pt[0] << "," << pt[1] << "," << pt[2] << "\n";
    }
    file.close();
    
    std::cout << "\nGenerated " << trajectory_points.size() << " trajectory points\n";
    std::cout << "Saved to original_waypoints_trajectory.csv\n";
    
    // Check max deviation from square
    double max_x_overshoot = 0, max_y_overshoot = 0;
    for (const auto& pt : trajectory_points) {
        if (pt[0] > 1) max_x_overshoot = std::max(max_x_overshoot, pt[0] - 1);
        if (pt[0] < 0) max_x_overshoot = std::max(max_x_overshoot, -pt[0]);
        if (pt[1] > 1) max_y_overshoot = std::max(max_y_overshoot, pt[1] - 1);
        if (pt[1] < 0) max_y_overshoot = std::max(max_y_overshoot, -pt[1]);
    }
    
    std::cout << "\nMax overshoot outside [0,1]x[0,1] square:\n";
    std::cout << "  X: " << max_x_overshoot << " m\n";
    std::cout << "  Y: " << max_y_overshoot << " m\n";
    
    return 0;
}