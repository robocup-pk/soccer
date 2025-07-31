#include "PurePursuitController.h"
#include <iostream>
#include <algorithm>
#include <limits>

namespace ctrl {

PurePursuitController::PurePursuitController()
    : config_(Config{}), current_target_index_(0), path_finished_(true) {
}

PurePursuitController::PurePursuitController(const Config& config)
    : config_(config), current_target_index_(0), path_finished_(true) {
}

void PurePursuitController::setPath(const std::vector<Eigen::Vector3d>& path) {
    if (path.empty()) {
        std::cerr << "[PurePursuitController] Warning: Empty path provided" << std::endl;
        path_.clear();
        path_finished_ = true;
        return;
    }
    
    path_ = path;
    current_target_index_ = 0;
    path_finished_ = false;
    
    std::cout << "[PurePursuitController] Path set with " << path.size() << " waypoints" << std::endl;
    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << "  Waypoint " << i << ": (" << path[i].x() << ", " << path[i].y() 
                  << ", " << path[i].z() << " rad)" << std::endl;
    }
}

PurePursuitController::ControlOutput PurePursuitController::update(const RobotState& robot_state) {
    ControlOutput output;
    
    if (path_.empty() || path_finished_) {
        output.path_finished = true;
        return output;
    }
    
    // Check if we've reached the final waypoint
    if (current_target_index_ >= static_cast<int>(path_.size()) - 1) {
        const auto& final_waypoint = path_.back();
        if (isWaypointReached(robot_state, final_waypoint)) {
            path_finished_ = true;
            output.path_finished = true;
            std::cout << "[PurePursuitController] Final waypoint reached. Path completed!" << std::endl;
            return output;
        }
    }
    
    // Find the target point to pursue
    int target_index = searchTargetIndex(robot_state);
    current_target_index_ = target_index;
    
    if (target_index < 0 || target_index >= static_cast<int>(path_.size())) {
        std::cerr << "[PurePursuitController] Invalid target index: " << target_index << std::endl;
        output.path_finished = true;
        return output;
    }
    
    // Calculate control output
    output = calculateControlOutput(robot_state, target_index);
    output.current_target_index = target_index;
    
    return output;
}

int PurePursuitController::searchTargetIndex(const RobotState& robot_state) const {
    if (path_.empty()) return -1;
    
    // Calculate look-ahead distance
    double look_ahead_distance = calculateLookAheadDistance(robot_state);
    
    // Start from current target index to avoid going backwards
    int best_index = current_target_index_;
    double robot_x = robot_state.x;
    double robot_y = robot_state.y;
    
    // Search forward from current target index
    for (int i = current_target_index_; i < static_cast<int>(path_.size()); ++i) {
        double dx = path_[i].x() - robot_x;
        double dy = path_[i].y() - robot_y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        // If we found a point that's approximately at the look-ahead distance, use it
        if (distance >= look_ahead_distance) {
            best_index = i;
            break;
        }
        
        // If this is the last point and we haven't found one at look-ahead distance,
        // target the last waypoint
        if (i == static_cast<int>(path_.size()) - 1) {
            best_index = i;
        }
    }
    
    return best_index;
}

double PurePursuitController::calculateLookAheadDistance(const RobotState& robot_state) const {
    // Calculate current speed
    double current_speed = std::sqrt(robot_state.vx * robot_state.vx + robot_state.vy * robot_state.vy);
    
    // Adaptive look-ahead distance: Lf = k * v + Lfc
    double look_ahead_distance = config_.look_ahead_gain * current_speed + config_.base_look_ahead_distance;
    
    // Ensure minimum and maximum look-ahead distances for SSL robots
    look_ahead_distance = std::max(look_ahead_distance, 0.05);  // Minimum 5cm
    look_ahead_distance = std::min(look_ahead_distance, 0.5);   // Maximum 50cm
    
    return look_ahead_distance;
}

PurePursuitController::ControlOutput PurePursuitController::calculateControlOutput(
    const RobotState& robot_state, int target_index) const {
    
    ControlOutput output;
    
    if (target_index < 0 || target_index >= static_cast<int>(path_.size())) {
        return output;
    }
    
    const auto& target_point = path_[target_index];
    
    // Calculate position error
    double dx = target_point.x() - robot_state.x;
    double dy = target_point.y() - robot_state.y;
    double distance_error = std::sqrt(dx * dx + dy * dy);
    
    // Pure Pursuit for omnidirectional robots: 
    // Direct velocity commands towards target point
    if (distance_error > config_.position_tolerance) {
        // Normalize direction vector
        double direction_x = dx / distance_error;
        double direction_y = dy / distance_error;
        
        // Calculate desired speed based on distance (proportional control)
        double desired_speed = config_.speed_proportional_gain * distance_error;
        desired_speed = std::min(desired_speed, config_.max_linear_velocity);
        
        // Command velocities in world frame
        output.vx_cmd = desired_speed * direction_x;
        output.vy_cmd = desired_speed * direction_y;
        
        std::cout << "[PurePursuitController] Target " << target_index 
                  << ": error=" << distance_error << "m, cmd_vel=(" 
                  << output.vx_cmd << ", " << output.vy_cmd << ")" << std::endl;
    }
    
    // Angular control: face the target orientation
    double angle_error = normalizeAngle(target_point.z() - robot_state.yaw);
    
    if (std::abs(angle_error) > config_.angular_tolerance) {
        // Proportional angular control
        output.vyaw_cmd = config_.speed_proportional_gain * angle_error;
        output.vyaw_cmd = std::max(-config_.max_angular_velocity, 
                                  std::min(config_.max_angular_velocity, output.vyaw_cmd));
    }
    
    return output;
}

double PurePursuitController::normalizeAngle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double PurePursuitController::calculateDistance(double x1, double y1, double x2, double y2) const {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

bool PurePursuitController::isWaypointReached(const RobotState& robot_state, 
                                             const Eigen::Vector3d& waypoint) const {
    double distance = calculateDistance(robot_state.x, robot_state.y, waypoint.x(), waypoint.y());
    double angle_error = std::abs(normalizeAngle(waypoint.z() - robot_state.yaw));
    
    return (distance < config_.position_tolerance) && (angle_error < config_.angular_tolerance);
}

void PurePursuitController::reset() {
    current_target_index_ = 0;
    path_finished_ = path_.empty();
    
    std::cout << "[PurePursuitController] Controller reset" << std::endl;
}

} // namespace ctrl