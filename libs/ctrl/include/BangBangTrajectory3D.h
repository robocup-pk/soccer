#ifndef BANGBANG_TRAJECTORY_3D_H
#define BANGBANG_TRAJECTORY_3D_H

#include <optional>
#include <memory>
#include <vector>
#include <mutex>
#include <Eigen/Dense>

#include "Trajectory3D.h"
#include "AccelerationEnvelope.h"

namespace ctrl {

// Enums and structures for BangBangTrajectory3D (to avoid conflicts with BangBangTrajectory.h)
enum class BangBangCase3D {
    CASE_1,      // w_dot_0 < 0 (moving away from destination)
    CASE_2_1,    // Accelerate to v_max
    CASE_2_2,    // Cruise at v_max  
    CASE_2_3,    // Decelerate to target
    CASE_3       // w_dot_0 > v_max (too fast)
};

// Trajectory structures matching the exact paper implementation
struct TrajectorySegment3D {
    BangBangCase3D case_type;
    double control_effort;    // u (acceleration)
    double duration;         // Δt
    double distance_traveled; // Δw
    double final_velocity;   // w_dot at end
};

struct DOFTrajectory3D {
    std::vector<TrajectorySegment3D> segments;
    double total_time;
    double total_distance;
};

// Complete Translation and Rotation trajectories (Section 5-7 from paper)
struct TranslationTrajectory3D {
    DOFTrajectory3D x_trajectory;
    DOFTrajectory3D y_trajectory;
    double alpha;               // Synchronization parameter (Section 5)
    double total_execution_time;
};

struct TrajectoryComplete3D {
    TranslationTrajectory3D translation;
    DOFTrajectory3D rotation;
    double total_execution_time;
};

/**
 * @brief BangBang trajectory implementation that exactly matches TrapezoidalTrajectoryVi3D interface
 * 
 * This class provides a drop-in replacement for TrapezoidalTrajectoryVi3D with:
 * - Exact same constructor signature
 * - Same virtual function implementations (VelocityAtT, PositionAtT, etc.)
 * - Same variable usage and normalization as existing system
 * - Optimized lookup table that runs once with cached interpolation
 * - Velocity output (not acceleration) for motor control (max RPM 280)
 * - Jerk handling similar to TrapezoidalTrajectory3D
 */
class BangBangTrajectory3D : public Trajectory3D {
public:
    /**
     * @brief Constructor matching TrapezoidalTrajectoryVi3D exactly
     * @param pose_start Starting pose [x, y, θ]
     * @param pose_end Target pose [x, y, θ]
     * @param t_start_s Start time (seconds)
     * @param t_end_s End time (seconds)
     * @param v0 Initial velocity [vx, vy, ω] (default: zero)
     */
    BangBangTrajectory3D(Eigen::Vector3d pose_start, Eigen::Vector3d pose_end, 
                         double t_start_s, double t_end_s, 
                         Eigen::Vector3d v0 = Eigen::Vector3d(0, 0, 0));

    // Virtual function implementations matching Trajectory3D interface
    Eigen::Vector3d VelocityAtT(double t) override;
    Eigen::Vector3d PositionAtT(double t) override;
    
    // Helper functions matching TrapezoidalTrajectoryVi3D
    void Print() override;
    Eigen::Vector3d TotalDistance() override;

    void SetTFinish(double t_finish_s) override { this->t_finish_s = t_finish_s; }
    double GetTStart() override { return t_start_s; }
    double GetTFinish() override { return t_finish_s; }
    Eigen::Vector3d GetPoseStart() override { return pose_start; }
    Eigen::Vector3d GetPoseEnd() override { return pose_end; }

private:
    // Same member variables as TrapezoidalTrajectoryVi3D for compatibility
    Eigen::Vector3d pose_start;
    Eigen::Vector3d pose_end;
    double t_start_s;
    double t_finish_s;
    double T;           // Total trajectory time
    Eigen::Vector3d h;  // Total displacement
    Eigen::Vector3d v0; // Initial velocity

    // Complete paper implementation structures (now defined above)
    
    // BangBang trajectory data (full paper implementation)
    DOFTrajectory3D x_trajectory_dof;
    DOFTrajectory3D y_trajectory_dof;
    DOFTrajectory3D theta_trajectory_dof;

    // Static acceleration envelope cache (optimized to run only once)
    static std::unique_ptr<AccelerationEnvelope> acceleration_envelope_;
    static bool envelope_initialized_;
    static std::mutex envelope_mutex_;
    
    // Cached acceleration data for interpolation
    struct AccelCache {
        std::vector<double> angles;
        std::vector<double> a_max_values;
        std::vector<double> alpha_max_values;
    };
    static AccelCache accel_cache_;
    
    // Core trajectory generation functions (full paper implementation)
    void InitializeAccelerationEnvelope();
    std::pair<double, double> GetMaxAcceleration(double direction_angle);
    TrajectoryComplete3D GenerateCompleteTrajectory(const Eigen::Vector3d& displacement, const Eigen::Vector3d& initial_velocity);
    DOFTrajectory3D GenerateSingleDOF(double w0, double wf, double w_dot_0, double w_dot_f, double v_max, double a_max);
    
    // Section 4: All 5 cases from Purwin & D'Andrea paper
    TrajectorySegment3D HandleCase1(double w_dot_0, double a_max);
    TrajectorySegment3D HandleCase2_1(double w_dot_0, double remaining_distance, double v_max, double a_max);
    TrajectorySegment3D HandleCase2_2(double w_dot_0, double remaining_distance, double v_max, double a_max);
    TrajectorySegment3D HandleCase2_3(double w_dot_0, double remaining_distance, double w_dot_f, double a_max);
    TrajectorySegment3D HandleCase3(double w_dot_0, double v_max, double a_max);
    
    // Section 5: X-Y synchronization via α-parameterization and bisection algorithm
    void SynchronizeTranslationTrajectories(TranslationTrajectory3D& translation);
    DOFTrajectory3D ScaleTrajectoryByAlpha(const DOFTrajectory3D& original, double alpha);
    
    // Trajectory evaluation functions
    double EvaluateTrajectoryAtTime(const DOFTrajectory3D& trajectory, double t, bool get_velocity);
    
    // Jerk handling for smooth motion profiles (similar to TrapezoidalTrajectory3D)
    void ApplyJerkLimiting();
    void SmoothTrajectoryTransitions(DOFTrajectory3D& trajectory, double min_time);
    double jerk_limit_ = 50.0;  // Similar to TrapezoidalTrajectory3D
    
    // Motor velocity conversion (max RPM 280)
    Eigen::Vector3d ConvertToMotorVelocities(const Eigen::Vector3d& body_velocity);
    double max_rpm_ = 280.0;
    double wheel_radius_ = 0.025;  // 25mm wheels
};

} // namespace ctrl

#endif // BANGBANG_TRAJECTORY_3D_H