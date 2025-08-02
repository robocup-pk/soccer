#include "BSplineTrajectory.h"
#include "RobotManager.h"
#include "Utils.h"
#include <iostream>
#include <cassert>
#include <algorithm>

namespace ctrl {

// Improved B-spline control point generation for better corner handling
void GenerateImprovedBSplineControlPoints(const std::vector<Eigen::Vector3d>& waypoints,
                                         std::vector<Eigen::Vector3d>& control_points,
                                         int spline_degree) {
    control_points.clear();
    
    if (waypoints.size() < 2) return;
    
    // For paths with corners, we need to handle them specially
    for (size_t i = 0; i < waypoints.size(); ++i) {
        if (i == 0) {
            // First waypoint - add it multiple times for clamped spline
            for (int j = 0; j <= spline_degree; ++j) {
                control_points.push_back(waypoints[i]);
            }
        } else if (i == waypoints.size() - 1) {
            // Last waypoint - add it multiple times
            for (int j = 0; j <= spline_degree; ++j) {
                control_points.push_back(waypoints[i]);
            }
        } else {
            // Interior waypoints - check for corners
            Eigen::Vector3d prev = waypoints[i-1];
            Eigen::Vector3d curr = waypoints[i];
            Eigen::Vector3d next = waypoints[i+1];
            
            // Calculate direction vectors
            Eigen::Vector3d v1 = curr - prev;
            Eigen::Vector3d v2 = next - curr;
            
            // Normalize to get directions
            double len1 = v1.head<2>().norm();
            double len2 = v2.head<2>().norm();
            
            if (len1 > 1e-6 && len2 > 1e-6) {
                Eigen::Vector2d dir1 = v1.head<2>() / len1;
                Eigen::Vector2d dir2 = v2.head<2>() / len2;
                
                // Calculate angle between segments
                double dot = dir1.dot(dir2);
                double angle = std::acos(std::clamp(dot, -1.0, 1.0));
                
                // If it's a sharp corner (> 30 degrees), add helper control points
                if (angle > M_PI/6) {
                    // Add control point before corner
                    double offset = std::min(len1, len2) * 0.2; // 20% of shorter segment
                    Eigen::Vector3d before_corner = curr;
                    before_corner.head<2>() -= dir1 * offset;
                    control_points.push_back(before_corner);
                    
                    // Add the corner point
                    control_points.push_back(curr);
                    
                    // Add control point after corner
                    Eigen::Vector3d after_corner = curr;
                    after_corner.head<2>() += dir2 * offset;
                    control_points.push_back(after_corner);
                } else {
                    // Small angle change - just add the waypoint
                    control_points.push_back(curr);
                }
            } else {
                // One of the segments has zero length
                control_points.push_back(curr);
            }
        }
    }
}

// Alternative: Use subdivision to create smoother corners
void GenerateSubdividedPath(const std::vector<Eigen::Vector3d>& waypoints,
                           std::vector<Eigen::Vector3d>& smooth_waypoints,
                           double corner_radius = 0.1) {
    smooth_waypoints.clear();
    
    if (waypoints.size() < 2) {
        smooth_waypoints = waypoints;
        return;
    }
    
    for (size_t i = 0; i < waypoints.size(); ++i) {
        if (i == 0 || i == waypoints.size() - 1) {
            // First and last points - keep as is
            smooth_waypoints.push_back(waypoints[i]);
        } else {
            // Interior points - check for corners
            Eigen::Vector3d prev = waypoints[i-1];
            Eigen::Vector3d curr = waypoints[i];
            Eigen::Vector3d next = waypoints[i+1];
            
            // Calculate segments
            Eigen::Vector3d seg1 = curr - prev;
            Eigen::Vector3d seg2 = next - curr;
            
            double len1 = seg1.head<2>().norm();
            double len2 = seg2.head<2>().norm();
            
            if (len1 > 2*corner_radius && len2 > 2*corner_radius) {
                // We can add a rounded corner
                
                // Point before corner
                Eigen::Vector3d p1 = curr - (seg1 / len1) * corner_radius;
                p1[2] = prev[2]; // Keep orientation from previous segment
                smooth_waypoints.push_back(p1);
                
                // Add intermediate points for smooth corner
                int num_corner_points = 3;
                for (int j = 1; j <= num_corner_points; ++j) {
                    double t = static_cast<double>(j) / (num_corner_points + 1);
                    
                    // Use circular interpolation for position
                    Eigen::Vector2d dir1 = -seg1.head<2>().normalized();
                    Eigen::Vector2d dir2 = seg2.head<2>().normalized();
                    
                    // Slerp between directions
                    double angle = std::atan2(dir2[1], dir2[0]) - std::atan2(dir1[1], dir1[0]);
                    while (angle > M_PI) angle -= 2*M_PI;
                    while (angle < -M_PI) angle += 2*M_PI;
                    
                    double interp_angle = std::atan2(dir1[1], dir1[0]) + t * angle;
                    Eigen::Vector2d interp_dir(std::cos(interp_angle), std::sin(interp_angle));
                    
                    Eigen::Vector3d corner_point;
                    corner_point.head<2>() = curr.head<2>() + interp_dir * corner_radius * (1.0 - 2*std::abs(t - 0.5));
                    
                    // Interpolate orientation
                    double theta_diff = curr[2] - prev[2];
                    while (theta_diff > M_PI) theta_diff -= 2*M_PI;
                    while (theta_diff < -M_PI) theta_diff += 2*M_PI;
                    corner_point[2] = prev[2] + t * theta_diff;
                    
                    smooth_waypoints.push_back(corner_point);
                }
                
                // Point after corner
                Eigen::Vector3d p2 = curr + (seg2 / len2) * corner_radius;
                p2[2] = curr[2]; // Keep orientation from corner
                smooth_waypoints.push_back(p2);
            } else {
                // Segments too short for corner rounding
                smooth_waypoints.push_back(curr);
            }
        }
    }
}

} // namespace ctrl