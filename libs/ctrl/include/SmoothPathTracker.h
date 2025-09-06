#pragma once

#include "PlanarCurve.h"
#include <vector>
#include <memory>

namespace ctrl {

/**
 * @brief A complete path smoothing and tracking controller.
 *
 * This class takes a series of waypoints, generates a smooth PlanarCurve through them,
 * and then uses a Pure Pursuit algorithm to follow that curve. It dynamically adjusts
 * the robot's speed based on the curve's curvature, ensuring smooth and efficient motion.
 * This class replaces the BangBangMotionPlanner and TrajectoryTracker.
 */
class SmoothPathTracker {
public:
    SmoothPathTracker();

    /**
     * @brief Plans and prepares the smooth path from a series of waypoints.
     * @param waypoints The sequence of waypoints (x, y, theta) from your RRTX planner.
     * @param maxVel The robot's maximum linear velocity (m/s).
     * @param maxAcc The robot's maximum linear acceleration (m/s^2).
     * @param maxOmega The robot's maximum angular velocity (rad/s).
     */
    void planPath(const std::vector<Eigen::Vector3d>& waypoints,
                  double maxVel, double maxAcc, double maxOmega);

    /**
     * @brief The main update loop for the tracker. Calculates the required robot velocity.
     * @param current_pose The robot's current estimated pose (x, y, theta) in the world frame.
     * @return The calculated body-frame velocity command (vx, vy, omega).
     */
    Eigen::Vector3d update(const Eigen::Vector3d& current_pose);

    /**
     * @brief Checks if the tracker has completed the path.
     */
    bool isFinished() const { return is_finished_; }

private:
    // --- Helper Functions ---
    double findClosestPointOnPath(const Eigen::Vector2d& robot_pos) const;
    double getTargetOrientation(double distance_along_path) const;

    // --- Path and Planner State ---
    std::unique_ptr<PlanarCurve> path_;
    std::vector<Eigen::Vector3d> original_waypoints_;
    double current_path_progress_; // Estimated distance traveled along the curve
    bool is_finished_;

    // --- Robot and Controller Parameters ---
    double max_velocity_;
    double max_acceleration_;
    double max_angular_velocity_;
    
    // Pure Pursuit lookahead distance. A larger value gives smoother control.
    static constexpr double LOOKAHEAD_DISTANCE = 0.15;
    // Gain for the orientation controller.
    static constexpr double HEADING_KP = 7.0;
    // Radius around the final waypoint to consider the path finished.
    static constexpr double GOAL_RADIUS = 0.03;
};

} // namespace ctrl
