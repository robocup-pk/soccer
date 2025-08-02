#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "RobotManager.h"
#include "BSplineTrajectory.h"
#include "Utils.h"

int main() {
    std::cout << "Testing B-spline smoothness for regular trajectories\n" << std::endl;
    
    // Test 1: Simple forward movement (should use original method)
    {
        std::cout << "=== Test 1: Forward movement to (1,0,0) ===" << std::endl;
        std::vector<Eigen::Vector3d> waypoints;
        waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
        
        ctrl::BSplineTrajectory bspline;
        bspline.SetPath(waypoints, 0.0);
        
        // Sample the trajectory at multiple points
        std::cout << "Time\tPosition\t\tVelocity" << std::endl;
        for (double t = 0.0; t <= 2.0; t += 0.2) {
            Eigen::Vector3d pose(0, 0, 0);  // Current pose (for feedback)
            Eigen::Vector3d vel = bspline.Update(pose, t);
            std::cout << t << "\t" << "---" << "\t\t" 
                      << vel[0] << ", " << vel[1] << ", " << vel[2] << std::endl;
        }
    }
    
    // Test 2: Pure rotation (should use rotation fix)
    {
        std::cout << "\n=== Test 2: Pure 180-degree rotation ===" << std::endl;
        std::vector<Eigen::Vector3d> waypoints;
        waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        waypoints.push_back(Eigen::Vector3d(0.0, 0.0, M_PI));
        
        ctrl::BSplineTrajectory bspline;
        bspline.SetPath(waypoints, 0.0);
        
        // Sample the trajectory
        std::cout << "Time\tPosition\t\tVelocity" << std::endl;
        for (double t = 0.0; t <= 2.0; t += 0.2) {
            Eigen::Vector3d pose(0, 0, 0);
            Eigen::Vector3d vel = bspline.Update(pose, t);
            std::cout << t << "\t" << "---" << "\t\t" 
                      << vel[0] << ", " << vel[1] << ", " << vel[2] << std::endl;
        }
    }
    
    // Test 3: Mixed movement and rotation
    {
        std::cout << "\n=== Test 3: Movement to (1,0) with 45-degree rotation ===" << std::endl;
        std::vector<Eigen::Vector3d> waypoints;
        waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        waypoints.push_back(Eigen::Vector3d(1.0, 0.0, M_PI/4));
        
        ctrl::BSplineTrajectory bspline;
        bspline.SetPath(waypoints, 0.0);
        
        // Sample the trajectory
        std::cout << "Time\tPosition\t\tVelocity" << std::endl;
        for (double t = 0.0; t <= 2.0; t += 0.2) {
            Eigen::Vector3d pose(0, 0, 0);
            Eigen::Vector3d vel = bspline.Update(pose, t);
            std::cout << t << "\t" << "---" << "\t\t" 
                      << vel[0] << ", " << vel[1] << ", " << vel[2] << std::endl;
        }
    }
    
    return 0;
}