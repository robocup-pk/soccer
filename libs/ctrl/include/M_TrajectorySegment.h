#ifndef M_TRAJECTORY_SEGMENT_H
#define M_TRAJECTORY_SEGMENT_H

#include "M_TrajectoryPlanner.h"
#include <memory>
#include <vector>

namespace ctrl {

/**
 * M_TrajectorySegment - A linked-list style trajectory implementation
 * Inspired by Sumatra's TrajPath approach for handling multiple waypoints
 * and runtime trajectory changes
 */
class M_TrajectorySegment : public M_Trajectory {
public:
    M_TrajectorySegment() = default;
    
    /**
     * Create a new trajectory segment
     * @param trajectory The base trajectory for this segment
     * @param segment_end_time The end time for this segment (may be less than trajectory total time)
     * @param child The next segment in the chain (optional)
     */
    M_TrajectorySegment(
        std::shared_ptr<M_Trajectory> trajectory,
        double segment_end_time,
        std::shared_ptr<M_TrajectorySegment> child = nullptr
    );
    
    /**
     * Create a trajectory segment from current state to destination
     */
    static std::shared_ptr<M_TrajectorySegment> create(
        const M_MoveConstraints& constraints,
        const Eigen::Vector3d& current_pose,
        const Eigen::Vector3d& current_velocity,
        const Eigen::Vector3d& destination
    );
    
    /**
     * Append a new segment at the specified connection time
     * @param constraints Movement constraints for the new segment
     * @param connection_time Time on current trajectory where new segment connects
     * @param destination New destination for the appended segment
     * @return A new trajectory with the appended segment
     */
    std::shared_ptr<M_TrajectorySegment> append(
        const M_MoveConstraints& constraints,
        double connection_time,
        const Eigen::Vector3d& destination
    );
    
    /**
     * Relocate the trajectory to a new initial state
     * Used when robot's actual position differs from planned trajectory
     * @param constraints Movement constraints
     * @param new_start_pose New starting position
     * @param new_start_velocity New starting velocity
     * @return A new relocated trajectory
     */
    std::shared_ptr<M_TrajectorySegment> relocate(
        const M_MoveConstraints& constraints,
        const Eigen::Vector3d& new_start_pose,
        const Eigen::Vector3d& new_start_velocity
    );
    
    /**
     * Get the next waypoint destination in the trajectory chain
     * @param t Current time
     * @return The destination of the current or next segment
     */
    Eigen::Vector3d getNextDestination(double t) const;
    
    /**
     * Check if trajectory passes through a specific waypoint
     * @param waypoint The waypoint to check
     * @param tolerance Distance tolerance for waypoint matching
     * @return True if trajectory passes through the waypoint
     */
    bool passesThrough(const Eigen::Vector3d& waypoint, double tolerance = 0.1) const;
    
    // Trajectory interface implementation
    Eigen::Vector3d getPosition(double t) const override;
    Eigen::Vector3d getVelocity(double t) const override;
    Eigen::Vector3d getAcceleration(double t) const override;
    double getTotalTime() const override;
    Eigen::Vector3d getFinalDestination() const override;
    double getMaxSpeed() const override;
    void print() const override;
    
    // Getters
    std::shared_ptr<M_Trajectory> getTrajectory() const { return trajectory_; }
    std::shared_ptr<M_TrajectorySegment> getChild() const { return child_; }
    double getSegmentEndTime() const { return segment_end_time_; }
    
private:
    std::shared_ptr<M_Trajectory> trajectory_;
    double segment_end_time_;
    std::shared_ptr<M_TrajectorySegment> child_;
    
    /**
     * Find the time offset where the trajectory is closest to a given position
     * Used for relocation
     */
    double findOffsetForPosition(const Eigen::Vector3d& pos) const;
    
    /**
     * Connect a new trajectory segment at the specified time
     */
    std::shared_ptr<M_TrajectorySegment> connect(
        std::shared_ptr<M_TrajectorySegment> new_segment,
        double connection_time
    );
};

/**
 * M_MultiWaypointPlanner - Enhanced trajectory planner for multiple waypoints
 * Handles dynamic replanning and waypoint sequences
 */
class M_MultiWaypointPlanner {
public:
    M_MultiWaypointPlanner() = default;
    
    /**
     * Generate a trajectory through multiple waypoints
     * @param robot_state Current robot state
     * @param waypoints Sequence of waypoints to visit
     * @param constraints Movement constraints
     * @return A segmented trajectory through all waypoints
     */
    static std::shared_ptr<M_TrajectorySegment> generateMultiWaypointTrajectory(
        const M_RobotState& robot_state,
        const std::vector<Eigen::Vector3d>& waypoints,
        const M_MoveConstraints& constraints
    );
    
    /**
     * Update trajectory during runtime
     * Handles replanning when robot deviates or obstacles appear
     * @param current_trajectory Current trajectory being executed
     * @param robot_state Current robot state
     * @param remaining_waypoints Waypoints not yet reached
     * @param constraints Movement constraints
     * @return Updated trajectory (may be same if no replanning needed)
     */
    static std::shared_ptr<M_TrajectorySegment> updateTrajectory(
        std::shared_ptr<M_TrajectorySegment> current_trajectory,
        const M_RobotState& robot_state,
        const std::vector<Eigen::Vector3d>& remaining_waypoints,
        const M_MoveConstraints& constraints
    );
    
    /**
     * Check if robot has reached a waypoint
     * @param robot_position Current robot position
     * @param waypoint Target waypoint
     * @param position_tolerance Position tolerance in meters
     * @param angle_tolerance Angle tolerance in radians
     * @return True if waypoint is reached within tolerances
     */
    static bool isWaypointReached(
        const Eigen::Vector3d& robot_position,
        const Eigen::Vector3d& waypoint,
        double position_tolerance = 0.05,
        double angle_tolerance = 0.1
    );
    
    /**
     * Generate intermediate waypoints for obstacle avoidance
     * @param start Starting position
     * @param end Target position
     * @param obstacles List of obstacles to avoid
     * @return List of intermediate waypoints
     */
    static std::vector<Eigen::Vector3d> generateIntermediateWaypoints(
        const Eigen::Vector3d& start,
        const Eigen::Vector3d& end,
        const std::vector<Eigen::Vector2d>& obstacles
    );
    
private:
    /**
     * Calculate position deviation from planned trajectory
     */
    static double calculatePositionDeviation(
        const Eigen::Vector3d& actual_position,
        std::shared_ptr<M_TrajectorySegment> trajectory,
        double current_time
    );
    
    /**
     * Determine if replanning is necessary
     */
    static bool needsReplanning(
        const M_RobotState& robot_state,
        std::shared_ptr<M_TrajectorySegment> trajectory,
        double current_time,
        double position_threshold = 0.1,
        double velocity_threshold = 0.2
    );
};

} // namespace ctrl

#endif // M_TRAJECTORY_SEGMENT_H