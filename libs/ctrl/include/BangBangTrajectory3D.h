#ifndef BANGBANG_TRAJECTORY_3D_H
#define BANGBANG_TRAJECTORY_3D_H

#include <optional>
#include <memory>
#include <Eigen/Dense>

#include "Trajectory3D.h"
#include "AccelerationEnvelope.h"
#include <mutex>

namespace ctrl {

// Enum for 5 cases from Section 4 of Purwin & D'Andrea paper
enum class BangBangCase3D {
    CASE_1,     // w_dot_0 < 0: reverse direction first
    CASE_2_1,   // accelerate to max velocity
    CASE_2_2,   // cruise at max velocity  
    CASE_2_3,   // decelerate to destination
    CASE_3      // w_dot_0 > v_max: decelerate first
};

// Single DOF bang-bang trajectory segment
struct TrajectorySegment3D {
    BangBangCase3D case_type;
    double control_effort;      // qw(t): acceleration command
    double duration;            // t': segment duration
    double distance_traveled;   // w': distance covered in segment
    double final_velocity;      // w_dot': velocity at end of segment
};

// Complete trajectory for one degree of freedom
struct DOFTrajectory3D {
    std::vector<TrajectorySegment3D> segments;
    double total_time;
    double total_distance;
};

// 2D trajectory combining x and y DOFs with synchronization
struct TranslationTrajectory3D {
    DOFTrajectory3D x_trajectory;
    DOFTrajectory3D y_trajectory;
    double total_execution_time;
    double alpha;               // synchronization parameter α
};

// Complete 3DOF trajectory including rotation
struct TrajectoryComplete3D {
    TranslationTrajectory3D translation;
    DOFTrajectory3D rotation;
    double total_execution_time;
};

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

    // Store trajectories for evaluation
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
    
    // Helper functions for complete paper implementation
    void InitializeAccelerationEnvelope();
    std::pair<double, double> GetMaxAcceleration(double direction_angle);
    
    // Core trajectory generation from paper
    TrajectoryComplete3D GenerateCompleteTrajectory(const Eigen::Vector3d& displacement, const Eigen::Vector3d& initial_velocity);
    DOFTrajectory3D GenerateSingleDOF(double w0, double wf, double w_dot_0, double w_dot_f, double v_max, double a_max);
    
    // Section 4: All 5 cases from Purwin & D'Andrea paper
    TrajectorySegment3D HandleCase1(double w_dot_0, double a_max);
    TrajectorySegment3D HandleCase2_1(double w_dot_0, double remaining_distance, double v_max, double a_max);
    TrajectorySegment3D HandleCase2_2(double w_dot_0, double remaining_distance, double v_max, double a_max);
    TrajectorySegment3D HandleCase2_3(double w_dot_0, double remaining_distance, double w_dot_f, double a_max);
    TrajectorySegment3D HandleCase3(double w_dot_0, double v_max, double a_max);
    
    // Section 5: X-Y synchronization
    double FindOptimalAlpha(double xf, double yf, double x_dot_0, double y_dot_0, double v_max, double a_max);
    void SynchronizeTranslationTrajectories(TranslationTrajectory3D& translation);
    DOFTrajectory3D ScaleTrajectoryByAlpha(const DOFTrajectory3D& original, double alpha);
    
    // Trajectory evaluation
    double EvaluateTrajectoryAtTime(const DOFTrajectory3D& trajectory, double t, bool get_velocity);
    void SmoothTrajectoryTransitions(DOFTrajectory3D& trajectory, double min_time);
    
    // Jerk handling for smooth motion profiles
    void ApplyJerkLimiting();
    double jerk_limit_ = 50.0;  // Similar to
    
    // Motor velocity conversion (max RPM 280)
    Eigen::Vector3d ConvertToMotorVelocities(const Eigen::Vector3d& body_velocity);
    double max_rpm_ = 300.0;
    double wheel_radius_ = 0.040;  // 40mm wheels converted to meters
};

} // namespace ctrl

#endif // BANGBANG_TRAJECTORY_3D_H