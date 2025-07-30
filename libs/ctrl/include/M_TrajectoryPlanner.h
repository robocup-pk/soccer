#ifndef M_TRAJECTORY_PLANNER_H
#define M_TRAJECTORY_PLANNER_H

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <functional>

namespace ctrl {

/**
 * M_TrajectoryPlanner - Advanced trajectory planner with Bang-Bang implementation
 * Implements 2D Bang-Bang trajectories with synchronization and path planning
 */

// Forward declarations
class M_Trajectory;
class M_BangBangTrajectory1D;
class M_BangBangTrajectory2D;

// Movement constraints structure
struct M_MoveConstraints {
    double vel_max = 2.0;        // Maximum velocity [m/s]
    double acc_max = 3.0;        // Maximum acceleration [m/s²]
    double brk_max = 3.0;        // Maximum deceleration [m/s²]
    double vel_max_w = 10.0;     // Maximum angular velocity [rad/s]
    double acc_max_w = 20.0;     // Maximum angular acceleration [rad/s²]
    Eigen::Vector2d primary_direction = Eigen::Vector2d::Zero(); // Primary direction for async trajectories
};

// Robot state information
struct M_RobotState {
    Eigen::Vector3d position;    // Position [x, y, theta] in meters
    Eigen::Vector3d velocity;    // Velocity [vx, vy, omega] in m/s and rad/s
    M_MoveConstraints constraints;
};

// Path finding input
struct M_PathFinderInput {
    M_RobotState robot_state;
    Eigen::Vector3d destination;  // Target position [x, y, theta]
    double target_time = -1.0;    // Optional target time (-1 = optimal time)
    bool use_primary_direction = false;
    Eigen::Vector2d primary_direction = Eigen::Vector2d::Zero();
};

// Path finding result
struct M_PathFinderResult {
    std::shared_ptr<M_Trajectory> trajectory;
    bool path_found = false;
    double total_time = 0.0;
    std::string info;
};

/**
 * Base trajectory interface for Bang-Bang trajectories
 */
class M_Trajectory {
public:
    virtual ~M_Trajectory() = default;
    
    // Core trajectory interface
    virtual Eigen::Vector3d getPosition(double t) const = 0;
    virtual Eigen::Vector3d getVelocity(double t) const = 0;
    virtual Eigen::Vector3d getAcceleration(double t) const = 0;
    virtual double getTotalTime() const = 0;
    
    // Utility methods
    virtual Eigen::Vector3d getFinalDestination() const { return getPosition(getTotalTime()); }
    virtual double getMaxSpeed() const = 0;
    virtual void print() const = 0;
};

/**
 * 1D Bang-Bang trajectory implementation with optimal time planning
 */
class M_BangBangTrajectory1D : public M_Trajectory {
public:
    M_BangBangTrajectory1D() = default;
    
    // Generate 1D Bang-Bang trajectory
    M_BangBangTrajectory1D& generate(
        double s0,          // Initial position
        double s1,          // Final position  
        double v0,          // Initial velocity
        double v_max,       // Maximum velocity
        double a_max        // Maximum acceleration
    );
    
    // Trajectory interface implementation
    Eigen::Vector3d getPosition(double t) const override;
    Eigen::Vector3d getVelocity(double t) const override;
    Eigen::Vector3d getAcceleration(double t) const override;
    double getTotalTime() const override { return total_time_; }
    double getMaxSpeed() const override { return v_max_; }
    void print() const override;
    
    // 1D specific methods
    double getPosition1D(double t) const;
    double getVelocity1D(double t) const;
    double getAcceleration1D(double t) const;
    
    // Getter methods for trajectory parameters
    double getS0() const { return s0_; }
    double getS1() const { return s1_; }
    double getV0() const { return v0_; }
    double getVMax() const { return v_max_; }
    double getAMax() const { return a_max_; }

private:
    // Trajectory parameters
    double s0_ = 0.0, s1_ = 0.0, v0_ = 0.0;
    double v_max_ = 0.0, a_max_ = 0.0;
    double total_time_ = 0.0;
    
    // Trajectory phases
    double t1_ = 0.0;  // End of acceleration phase
    double t2_ = 0.0;  // End of constant velocity phase
    double t3_ = 0.0;  // End of deceleration phase
    
    double v_peak_ = 0.0;  // Peak velocity reached
    bool has_cruise_phase_ = false;
    
    void calculateTrajectory();
};

/**
 * 2D Bang-Bang trajectory implementation with axis synchronization
 */
class M_BangBangTrajectory2D : public M_Trajectory {
public:
    M_BangBangTrajectory2D() = default;
    
    // Generate synchronized 2D Bang-Bang trajectory
    M_BangBangTrajectory2D& generate(
        const Eigen::Vector2d& s0,  // Initial position
        const Eigen::Vector2d& s1,  // Final position
        const Eigen::Vector2d& v0,  // Initial velocity
        double v_max,               // Maximum velocity
        double a_max,               // Maximum acceleration
        double sync_accuracy = 1e-3 // Synchronization accuracy
    );
    
    // Generate asynchronized 2D Bang-Bang trajectory with primary direction
    M_BangBangTrajectory2D& generateAsync(
        const Eigen::Vector2d& s0,     // Initial position
        const Eigen::Vector2d& s1,     // Final position
        const Eigen::Vector2d& v0,     // Initial velocity
        double v_max,                  // Maximum velocity
        double a_max,                  // Maximum acceleration
        const Eigen::Vector2d& primary_dir, // Primary direction
        double sync_accuracy = 1e-3    // Synchronization accuracy
    );
    
    // Trajectory interface implementation
    Eigen::Vector3d getPosition(double t) const override;
    Eigen::Vector3d getVelocity(double t) const override;
    Eigen::Vector3d getAcceleration(double t) const override;
    double getTotalTime() const override;
    double getMaxSpeed() const override;
    void print() const override;
    
    // 2D specific methods
    Eigen::Vector2d getPosition2D(double t) const;
    Eigen::Vector2d getVelocity2D(double t) const;
    Eigen::Vector2d getAcceleration2D(double t) const;

private:
    M_BangBangTrajectory1D traj_x_, traj_y_;
    bool is_async_ = false;
    double rotation_ = 0.0;
    Eigen::Vector2d offset_;
    
    // Synchronization for 2D trajectory
    void synchronizeTrajectories(double sync_accuracy);
    double findSyncFactor(double target_time, double sync_accuracy);
};

/**
 * 3D trajectory combining 2D position and 1D rotation
 */
class M_BangBangTrajectory3D : public M_Trajectory {
public:
    M_BangBangTrajectory3D() = default;
    
    // Generate 3D trajectory (2D position + rotation)
    M_BangBangTrajectory3D& generate(
        const Eigen::Vector3d& s0,  // Initial pose [x, y, theta]
        const Eigen::Vector3d& s1,  // Final pose [x, y, theta]
        const Eigen::Vector3d& v0,  // Initial velocity [vx, vy, omega]
        const M_MoveConstraints& constraints
    );
    
    // Trajectory interface implementation
    Eigen::Vector3d getPosition(double t) const override;
    Eigen::Vector3d getVelocity(double t) const override;
    Eigen::Vector3d getAcceleration(double t) const override;
    double getTotalTime() const override;
    double getMaxSpeed() const override;
    void print() const override;

private:
    M_BangBangTrajectory2D position_traj_;
    M_BangBangTrajectory1D rotation_traj_;
};

/**
 * Trajectory factory for creating optimized Bang-Bang trajectories
 */
class M_TrajectoryFactory {
public:
    // Create synchronized 2D trajectory
    static std::shared_ptr<M_BangBangTrajectory2D> createSync(
        const Eigen::Vector2d& s0,
        const Eigen::Vector2d& s1,
        const Eigen::Vector2d& v0,
        double v_max,
        double a_max
    );
    
    // Create asynchronized 2D trajectory with primary direction
    static std::shared_ptr<M_BangBangTrajectory2D> createAsync(
        const Eigen::Vector2d& s0,
        const Eigen::Vector2d& s1,
        const Eigen::Vector2d& v0,
        double v_max,
        double a_max,
        const Eigen::Vector2d& primary_dir
    );
    
    // Create 1D trajectory
    static std::shared_ptr<M_BangBangTrajectory1D> createSingle(
        double s0,
        double s1,
        double v0,
        double v_max,
        double a_max
    );
    
    // Create rotation trajectory
    static std::shared_ptr<M_BangBangTrajectory1D> createOrientation(
        double s0,
        double s1,
        double v0,
        double v_max,
        double a_max
    );
    
    // Create 3D trajectory
    static std::shared_ptr<M_BangBangTrajectory3D> create3D(
        const Eigen::Vector3d& s0,
        const Eigen::Vector3d& s1,
        const Eigen::Vector3d& v0,
        const M_MoveConstraints& constraints
    );
};

/**
 * Main trajectory planner class with advanced path optimization
 */
class M_TrajectoryPlanner {
public:
    M_TrajectoryPlanner() = default;
    
    // Generate position trajectory for robot
    static std::shared_ptr<M_Trajectory> generatePositionTrajectory(
        const M_RobotState& robot_state,
        const Eigen::Vector3d& destination
    );
    
    // Generate position trajectory with custom constraints
    static std::shared_ptr<M_Trajectory> generatePositionTrajectory(
        const M_MoveConstraints& constraints,
        const Eigen::Vector3d& current_pose,
        const Eigen::Vector3d& current_velocity,
        const Eigen::Vector3d& destination
    );
    
    // Generate rotation trajectory
    static std::shared_ptr<M_Trajectory> generateRotationTrajectory(
        const M_RobotState& robot_state,
        double target_angle
    );
    
    // Generate trajectory to reach point in specific time
    static std::shared_ptr<M_Trajectory> generateTrajectoryToReachPointInTime(
        const M_RobotState& robot_state,
        const Eigen::Vector3d& destination,
        double target_time
    );
    
    // Path finding with obstacle avoidance
    static M_PathFinderResult findPath(const M_PathFinderInput& input);
    
    // Utility methods
    static double calculateOptimalTime(
        const Eigen::Vector3d& displacement,
        const Eigen::Vector3d& initial_velocity,
        const M_MoveConstraints& constraints
    );
    
    static bool isComeToStopFaster(
        const M_RobotState& robot_state,
        const Eigen::Vector3d& destination
    );
    
    // Utility methods (made public for factory access)
    static double normalizeAngle(double angle);
    static Eigen::Vector2d adaptVelocity(const Eigen::Vector2d& v0, double v_max);
    static double adaptVelocity(double v0, double v_max);

private:
    // Internal helper methods
};

} // namespace ctrl

#endif // M_TRAJECTORY_PLANNER_H