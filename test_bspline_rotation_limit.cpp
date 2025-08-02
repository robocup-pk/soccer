#include <iostream>
#include <Eigen/Dense>
#include <cmath>

// Simple B-spline evaluation to test rotation limits
Eigen::Vector3d EvaluateBSpline(const std::vector<Eigen::Vector3d>& control_points, double t) {
    // Cubic B-spline (degree 3)
    int n = control_points.size() - 1;
    int k = 4; // degree + 1
    
    // For simplicity, assume uniform knot vector
    std::vector<double> knots;
    for (int i = 0; i < n + k + 1; ++i) {
        if (i < k) knots.push_back(0.0);
        else if (i > n) knots.push_back(1.0);
        else knots.push_back((double)(i - k + 1) / (n - k + 2));
    }
    
    // Find knot span
    int span = k - 1;
    while (span < n && t >= knots[span + 1]) span++;
    
    // Compute basis functions
    std::vector<double> N(k, 0.0);
    N[0] = 1.0;
    
    for (int j = 1; j < k; ++j) {
        for (int i = k - 1; i >= j; --i) {
            double alpha = 0.0;
            if (knots[span + i - j + 1] != knots[span + i - k + 1]) {
                alpha = (t - knots[span + i - k + 1]) / (knots[span + i - j + 1] - knots[span + i - k + 1]);
            }
            N[i] = alpha * N[i] + (1.0 - alpha) * N[i - 1];
        }
    }
    
    // Compute weighted sum
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    for (int i = 0; i < k; ++i) {
        if (span - k + 1 + i >= 0 && span - k + 1 + i < control_points.size()) {
            result += N[i] * control_points[span - k + 1 + i];
        }
    }
    
    return result;
}

int main() {
    std::cout << "Testing B-spline rotation limits" << std::endl;
    
    // Test different target rotations
    std::vector<double> target_rotations = {M_PI/2, M_PI, 3*M_PI/2, 2*M_PI};
    
    for (double target : target_rotations) {
        std::cout << "\n=== Target rotation: " << target << " rad (" << target * 180 / M_PI << " deg) ===" << std::endl;
        
        // Create control points for rotation
        std::vector<Eigen::Vector3d> control_points;
        
        // Method 1: Simple linear interpolation (what current code does)
        Eigen::Vector3d p0(0, 0, 0);
        Eigen::Vector3d p1(0, 0, target);
        
        control_points.push_back(p0);
        control_points.push_back(p0 + 0.25 * (p1 - p0));
        control_points.push_back(p0 + 0.5 * (p1 - p0));
        control_points.push_back(p0 + 0.75 * (p1 - p0));
        control_points.push_back(p1);
        
        // Evaluate at t=1.0 to see final rotation
        Eigen::Vector3d final_pose = EvaluateBSpline(control_points, 1.0);
        std::cout << "Method 1 (linear interp): Final rotation = " << final_pose[2] 
                  << " rad (" << final_pose[2] * 180 / M_PI << " deg)" << std::endl;
        std::cout << "Achieved: " << (final_pose[2] / target * 100) << "%" << std::endl;
        
        // Method 2: All control points at target (clamped)
        control_points.clear();
        control_points.push_back(p0);
        control_points.push_back(p0);
        control_points.push_back(p1);
        control_points.push_back(p1);
        control_points.push_back(p1);
        
        final_pose = EvaluateBSpline(control_points, 1.0);
        std::cout << "Method 2 (clamped): Final rotation = " << final_pose[2] 
                  << " rad (" << final_pose[2] * 180 / M_PI << " deg)" << std::endl;
        std::cout << "Achieved: " << (final_pose[2] / target * 100) << "%" << std::endl;
    }
    
    return 0;
}