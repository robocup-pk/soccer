#include "SmoothPathTracker.h"
#include "Utils.h"
#include <iostream>

namespace ctrl {

SmoothPathTracker::SmoothPathTracker()
    : path_(nullptr),
      current_path_progress_(0.0),
      is_finished_(true),
      max_velocity_(1.0),
      max_acceleration_(1.5),
      max_angular_velocity_(4.0) {}


void SmoothPathTracker::planPath(const std::vector<Eigen::Vector3d>& waypoints,
                                 double maxVel, double maxAcc, double maxOmega) {
    if (waypoints.size() < 2) {
        std::cerr << "[SmoothPathTracker] Path requires at least 2 waypoints." << std::endl;
        is_finished_ = true;
        return;
    }
    
    max_velocity_ = maxVel;
    max_acceleration_ = maxAcc;
    max_angular_velocity_ = maxOmega;
    original_waypoints_ = waypoints;

    // Convert 3D waypoints to 2D for the curve smoother
    std::vector<Eigen::Vector2d> waypoints_2d;
    waypoints_2d.reserve(waypoints.size());
    for (const auto& wp : waypoints) {
        waypoints_2d.push_back(wp.head<2>());
    }

    // Create the smooth spline curve
    path_ = std::make_unique<PlanarCurve>(waypoints_2d);

    // Reset state for the new path
    current_path_progress_ = 0.0;
    is_finished_ = !path_->isValid();
}

Eigen::Vector3d SmoothPathTracker::update(const Eigen::Vector3d& current_pose) {
    if (is_finished_ || !path_) {
        return Eigen::Vector3d::Zero();
    }

    // --- 1. Find our current position on the smooth path ---
    double s_current = findClosestPointOnPath(current_pose.head<2>());

    // --- 2. Check for goal completion ---
    double dist_to_goal = (original_waypoints_.back().head<2>() - current_pose.head<2>()).norm();
    if (dist_to_goal < GOAL_RADIUS) {
        is_finished_ = true;
        return Eigen::Vector3d::Zero();
    }

    // --- 3. Determine the lookahead point (Pure Pursuit) ---
    double s_lookahead = s_current + LOOKAHEAD_DISTANCE;
    Eigen::Vector2d lookahead_point = path_->getPositionAt(s_lookahead);

    // --- 4. Calculate curvature at the lookahead point to limit speed ---
    double curvature = std::abs(path_->getCurvatureAt(s_lookahead));
    double max_speed_for_curve = max_velocity_;
    if (curvature > 1e-3) {
        // v = sqrt(a_lateral / curvature)
        max_speed_for_curve = std::sqrt(max_acceleration_ * 0.8 / curvature);
    }

    // --- 5. Determine the final target speed ---
    double target_speed = std::min(max_velocity_, max_speed_for_curve);
    
    // --- 6. Calculate world-frame velocity command to drive towards the lookahead point ---
    Eigen::Vector2d world_vel_dir = (lookahead_point - current_pose.head<2>()).normalized();
    Eigen::Vector2d world_vel = world_vel_dir * target_speed;

    // --- 7. Calculate angular velocity command ---
    double desired_heading = getTargetOrientation(s_current);
    double heading_error = util::WrapAngle(desired_heading - current_pose.z());
    double omega = HEADING_KP * heading_error;
    omega = std::clamp(omega, -max_angular_velocity_, max_angular_velocity_);

    // --- 8. Combine and convert to body frame ---
    Eigen::Vector3d world_command(world_vel.x(), world_vel.y(), omega);
    return util::RotateAboutZ(world_command, -current_pose.z());
}

double SmoothPathTracker::findClosestPointOnPath(const Eigen::Vector2d& robot_pos) const {
    // Note: A more optimized implementation would start the search from the last known progress
    // to avoid re-scanning the whole path every frame.
    double best_s = current_path_progress_;
    double min_dist_sq = (path_->getPositionAt(best_s) - robot_pos).squaredNorm();

    // Coarse search forward from last known point
    int num_samples = 50;
    for (int i = 1; i <= num_samples; ++i) {
        double s = current_path_progress_ + (static_cast<double>(i) / num_samples) * (LOOKAHEAD_DISTANCE * 2);
        if (s > path_->getTotalLength()) break;

        double dist_sq = (path_->getPositionAt(s) - robot_pos).squaredNorm();
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            best_s = s;
        }
    }
    return best_s;
}

double SmoothPathTracker::getTargetOrientation(double distance_along_path) const {
    // Find which original waypoint segment we are on
    for (size_t i = 0; i < original_waypoints_.size() - 1; ++i) {
        double segment_start_dist = (original_waypoints_[i].head<2>() - original_waypoints_[0].head<2>()).norm();
        double segment_end_dist = (original_waypoints_[i+1].head<2>() - original_waypoints_[0].head<2>()).norm();
        if (distance_along_path >= segment_start_dist && distance_along_path <= segment_end_dist) {
            // Interpolate between the start and end orientation of this waypoint segment
            double segment_len = segment_end_dist - segment_start_dist;
            double progress = (segment_len > 1e-6) ? (distance_along_path - segment_start_dist) / segment_len : 0;

            double start_angle = original_waypoints_[i].z();
            double end_angle = original_waypoints_[i+1].z();
            end_angle = start_angle + util::WrapAngle(end_angle - start_angle); // Ensure shortest path
            
            return start_angle + progress * (end_angle - start_angle);
        }
    }
    return original_waypoints_.back().z(); // If past the end, use final orientation
}

} // namespace ctrl
