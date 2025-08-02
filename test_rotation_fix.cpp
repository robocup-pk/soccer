#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "RobotManager.h"
#include "Utils.h"

int main() {
    std::cout << "Testing B-spline rotation fix" << std::endl;
    
    // Test different rotation angles
    std::vector<double> test_angles = {M_PI/2, M_PI, 3*M_PI/2, 2*M_PI};
    std::vector<std::string> angle_names = {"90°", "180°", "270°", "360°"};
    
    for (size_t i = 0; i < test_angles.size(); ++i) {
        std::cout << "\n=== Testing " << angle_names[i] << " rotation ===" << std::endl;
        
        rob::RobotManager robot_manager;
        robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
        
        // Create waypoints for pure rotation
        std::vector<Eigen::Vector3d> waypoints;
        waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        waypoints.push_back(Eigen::Vector3d(0.0, 0.0, test_angles[i]));
        
        std::cout << "Requested rotation: " << test_angles[i] << " rad (" 
                  << test_angles[i] * 180 / M_PI << " deg)" << std::endl;
        
        // Set path (this will print the achieved rotation)
        robot_manager.SetBSplinePath(waypoints, util::GetCurrentTime());
        
        // Let the trajectory run for a moment
        for (int j = 0; j < 10; ++j) {
            robot_manager.ControlLogic();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    return 0;
}