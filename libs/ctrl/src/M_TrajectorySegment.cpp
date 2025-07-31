#include "M_TrajectorySegment.h"
#include <iostream>
#include <iomanip>
#include <limits>

namespace ctrl {

//=============================================================================
// M_TrajectorySegment Implementation
//=============================================================================

M_TrajectorySegment::M_TrajectorySegment(
    std::shared_ptr<M_Trajectory> trajectory,
    double segment_end_time,
    std::shared_ptr<M_TrajectorySegment> child)
    : trajectory_(trajectory), segment_end_time_(segment_end_time), child_(child) {
    
    if (segment_end_time < -0.2) {
        throw std::invalid_argument("Invalid segment_end_time: " + std::to_string(segment_end_time));
    }
}

std::shared_ptr<M_TrajectorySegment> M_TrajectorySegment::create(
    const M_MoveConstraints& constraints,
    const Eigen::Vector3d& current_pose,
    const Eigen::Vector3d& current_velocity,
    const Eigen::Vector3d& destination) {
    
    auto traj = M_TrajectoryFactory::create3D(current_pose, destination, current_velocity, constraints);
    return std::make_shared<M_TrajectorySegment>(traj, traj->getTotalTime(), nullptr);
}

std::shared_ptr<M_TrajectorySegment> M_TrajectorySegment::append(
    const M_MoveConstraints& constraints,
    double connection_time,
    const Eigen::Vector3d& destination) {
    
    Eigen::Vector3d cur_pos = getPosition(connection_time);
    Eigen::Vector3d cur_vel = getVelocity(connection_time);
    
    auto child_segment = create(constraints, cur_pos, cur_vel, destination);
    return connect(child_segment, connection_time);
}

std::shared_ptr<M_TrajectorySegment> M_TrajectorySegment::relocate(
    const M_MoveConstraints& constraints,
    const Eigen::Vector3d& new_start_pose,
    const Eigen::Vector3d& new_start_velocity) {
    
    double time_offset = findOffsetForPosition(new_start_pose);
    
    if (time_offset >= segment_end_time_) {
        if (child_ == nullptr) {
            // If we've overshot the final destination, create a trajectory to return
            return create(constraints, new_start_pose, new_start_velocity, getFinalDestination());
        }
        // Relocate in the child segment
        return child_->relocate(constraints, new_start_pose, new_start_velocity);
    }
    
    // Create new parent trajectory from current position to original destination
    auto new_parent = create(constraints, new_start_pose, new_start_velocity, 
                           trajectory_->getFinalDestination());
    
    if (child_ != nullptr) {
        // Append the remaining waypoints
        return new_parent->append(constraints, segment_end_time_ - time_offset, 
                                child_->getFinalDestination());
    }
    
    return new_parent;
}

Eigen::Vector3d M_TrajectorySegment::getNextDestination(double t) const {
    if (child_ == nullptr || t < segment_end_time_) {
        return trajectory_->getFinalDestination();
    }
    return child_->getNextDestination(t - segment_end_time_);
}

bool M_TrajectorySegment::passesThrough(const Eigen::Vector3d& waypoint, double tolerance) const {
    // Check if this segment's destination is close to the waypoint
    double dist = (trajectory_->getFinalDestination() - waypoint).head<2>().norm();
    if (dist < tolerance) {
        return true;
    }
    
    // Check child segments
    if (child_ != nullptr) {
        return child_->passesThrough(waypoint, tolerance);
    }
    
    return false;
}

Eigen::Vector3d M_TrajectorySegment::getPosition(double t) const {
    if (t <= segment_end_time_) {
        return trajectory_->getPosition(t);
    }
    if (child_ != nullptr) {
        return child_->getPosition(t - segment_end_time_);
    }
    return trajectory_->getPosition(segment_end_time_);
}

Eigen::Vector3d M_TrajectorySegment::getVelocity(double t) const {
    if (t <= segment_end_time_) {
        return trajectory_->getVelocity(t);
    }
    if (child_ != nullptr) {
        return child_->getVelocity(t - segment_end_time_);
    }
    return trajectory_->getVelocity(segment_end_time_);
}

Eigen::Vector3d M_TrajectorySegment::getAcceleration(double t) const {
    if (t <= segment_end_time_) {
        return trajectory_->getAcceleration(t);
    }
    if (child_ != nullptr) {
        return child_->getAcceleration(t - segment_end_time_);
    }
    return trajectory_->getAcceleration(segment_end_time_);
}

double M_TrajectorySegment::getTotalTime() const {
    if (child_ != nullptr) {
        return segment_end_time_ + child_->getTotalTime();
    }
    return segment_end_time_;
}

Eigen::Vector3d M_TrajectorySegment::getFinalDestination() const {
    if (child_ != nullptr) {
        return child_->getFinalDestination();
    }
    return trajectory_->getPosition(segment_end_time_);
}

double M_TrajectorySegment::getMaxSpeed() const {
    double max_speed = trajectory_->getMaxSpeed();
    if (child_ != nullptr) {
        max_speed = std::max(max_speed, child_->getMaxSpeed());
    }
    return max_speed;
}

void M_TrajectorySegment::print() const {
    Eigen::Vector3d p0 = trajectory_->getPosition(0);
    Eigen::Vector3d p1 = trajectory_->getPosition(segment_end_time_);
    Eigen::Vector3d p2 = trajectory_->getFinalDestination();
    
    std::cout << "[Segment] " << std::fixed << std::setprecision(3)
              << "(" << p0.x() << "," << p0.y() << "," << p0.z() << ") -> "
              << "(" << p1.x() << "," << p1.y() << "," << p1.z() << ") => "
              << "(" << p2.x() << "," << p2.y() << "," << p2.z() << "): "
              << segment_end_time_ << "s/" << trajectory_->getTotalTime() << "s/"
              << getTotalTime() << "s";
    
    if (child_ != nullptr) {
        std::cout << "\n  => ";
        child_->print();
    } else {
        std::cout << std::endl;
    }
}

double M_TrajectorySegment::findOffsetForPosition(const Eigen::Vector3d& pos) const {
    double min_dist = std::numeric_limits<double>::max();
    double best_time = 0.0;
    
    // Sample the trajectory to find closest point
    for (double t = 0; t < segment_end_time_; t += 0.01) {
        double dist = (trajectory_->getPosition(t) - pos).head<2>().squaredNorm();
        if (dist < min_dist) {
            min_dist = dist;
            best_time = t;
        } else if (dist > min_dist * 1.1) {
            // Distance is increasing, we've passed the closest point
            break;
        }
    }
    
    return best_time;
}

std::shared_ptr<M_TrajectorySegment> M_TrajectorySegment::connect(
    std::shared_ptr<M_TrajectorySegment> new_segment,
    double connection_time) {
    
    if (child_ != nullptr && connection_time > segment_end_time_) {
        // Connection point is in child segment
        auto new_child = child_->connect(new_segment, connection_time - segment_end_time_);
        return std::make_shared<M_TrajectorySegment>(trajectory_, segment_end_time_, new_child);
    }
    
    // Connection point is in this segment
    return std::make_shared<M_TrajectorySegment>(trajectory_, connection_time, new_segment);
}

//=============================================================================
// M_MultiWaypointPlanner Implementation
//=============================================================================

std::shared_ptr<M_TrajectorySegment> M_MultiWaypointPlanner::generateMultiWaypointTrajectory(
    const M_RobotState& robot_state,
    const std::vector<Eigen::Vector3d>& waypoints,
    const M_MoveConstraints& constraints) {
    
    if (waypoints.empty()) {
        throw std::invalid_argument("No waypoints provided");
    }
    
    // Check if we're already at the first waypoint
    double dist_to_first = (robot_state.position - waypoints[0]).head<2>().norm();
    size_t start_idx = 0;
    
    if (dist_to_first < 0.01) {  // Already at first waypoint
        start_idx = 1;
        if (waypoints.size() == 1) {
            // Already at the only waypoint
            return M_TrajectorySegment::create(
                constraints, robot_state.position, robot_state.velocity, waypoints[0]);
        }
    }
    
    // Create initial trajectory to first relevant waypoint
    auto trajectory = M_TrajectorySegment::create(
        constraints, robot_state.position, robot_state.velocity, waypoints[start_idx]);
    
    // Append remaining waypoints
    for (size_t i = start_idx + 1; i < waypoints.size(); ++i) {
        // Connect at the end of the current trajectory
        double connection_time = trajectory->getTotalTime();
        trajectory = trajectory->append(constraints, connection_time, waypoints[i]);
    }
    
    return trajectory;
}

std::shared_ptr<M_TrajectorySegment> M_MultiWaypointPlanner::updateTrajectory(
    std::shared_ptr<M_TrajectorySegment> current_trajectory,
    const M_RobotState& robot_state,
    const std::vector<Eigen::Vector3d>& remaining_waypoints,
    const M_MoveConstraints& constraints) {
    
    if (remaining_waypoints.empty()) {
        return current_trajectory;
    }
    
    // Calculate current time on trajectory (simplified - in practice would track actual time)
    double current_time = 0.0;  // This should be tracked externally
    
    // Check if replanning is needed
    if (needsReplanning(robot_state, current_trajectory, current_time)) {
        // Replan from current position
        return generateMultiWaypointTrajectory(robot_state, remaining_waypoints, constraints);
    }
    
    // Check if we need to relocate due to position drift
    double deviation = calculatePositionDeviation(robot_state.position, current_trajectory, current_time);
    if (deviation > 0.05) {  // 5cm threshold
        // Relocate the trajectory
        return current_trajectory->relocate(constraints, robot_state.position, robot_state.velocity);
    }
    
    return current_trajectory;
}

bool M_MultiWaypointPlanner::isWaypointReached(
    const Eigen::Vector3d& robot_position,
    const Eigen::Vector3d& waypoint,
    double position_tolerance,
    double angle_tolerance) {
    
    double pos_error = (robot_position.head<2>() - waypoint.head<2>()).norm();
    double angle_error = std::abs(M_TrajectoryPlanner::normalizeAngle(robot_position.z() - waypoint.z()));
    
    return pos_error < position_tolerance && angle_error < angle_tolerance;
}

std::vector<Eigen::Vector3d> M_MultiWaypointPlanner::generateIntermediateWaypoints(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const std::vector<Eigen::Vector2d>& obstacles) {
    
    std::vector<Eigen::Vector3d> waypoints;
    
    // Simple implementation: if direct path is blocked, go around
    // In practice, this would use a more sophisticated algorithm like RRT*
    
    Eigen::Vector2d start_2d = start.head<2>();
    Eigen::Vector2d end_2d = end.head<2>();
    Eigen::Vector2d direction = (end_2d - start_2d).normalized();
    Eigen::Vector2d perpendicular(-direction.y(), direction.x());
    
    // Check if any obstacle is in the direct path
    bool path_blocked = false;
    for (const auto& obstacle : obstacles) {
        // Simple check - in practice would be more sophisticated
        double dist_to_line = std::abs((obstacle - start_2d).dot(perpendicular));
        double dist_along_line = (obstacle - start_2d).dot(direction);
        
        if (dist_to_line < 0.5 && dist_along_line > 0 && 
            dist_along_line < (end_2d - start_2d).norm()) {
            path_blocked = true;
            break;
        }
    }
    
    if (path_blocked) {
        // Add intermediate waypoint to go around
        Eigen::Vector3d intermediate = start;
        intermediate.head<2>() += perpendicular * 1.0;  // Go 1m to the side
        waypoints.push_back(intermediate);
    }
    
    waypoints.push_back(end);
    return waypoints;
}

double M_MultiWaypointPlanner::calculatePositionDeviation(
    const Eigen::Vector3d& actual_position,
    std::shared_ptr<M_TrajectorySegment> trajectory,
    double current_time) {
    
    Eigen::Vector3d planned_position = trajectory->getPosition(current_time);
    return (actual_position.head<2>() - planned_position.head<2>()).norm();
}

bool M_MultiWaypointPlanner::needsReplanning(
    const M_RobotState& robot_state,
    std::shared_ptr<M_TrajectorySegment> trajectory,
    double current_time,
    double position_threshold,
    double velocity_threshold) {
    
    // Check position deviation
    double pos_deviation = calculatePositionDeviation(robot_state.position, trajectory, current_time);
    if (pos_deviation > position_threshold) {
        return true;
    }
    
    // Check velocity deviation
    Eigen::Vector3d planned_velocity = trajectory->getVelocity(current_time);
    double vel_deviation = (robot_state.velocity - planned_velocity).norm();
    if (vel_deviation > velocity_threshold) {
        return true;
    }
    
    return false;
}

} // namespace ctrl