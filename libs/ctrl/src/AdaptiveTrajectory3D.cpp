#include "AdaptiveTrajectory3D.h"
#include "SystemConfig.h"

// Include the tinyspline library
#include <tinysplinecxx.h>
#include <vector>
#include <deque>
#include <cmath>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <stdexcept>

// Basic geometric types for the follower
struct Point2D {
  double x, y;
  Point2D(double x = 0, double y = 0) : x(x), y(y) {}
  Point2D operator+(const Point2D& other) const { return Point2D(x + other.x, y + other.y); }
  Point2D operator-(const Point2D& other) const { return Point2D(x - other.x, y - other.y); }
  Point2D operator*(double scalar) const { return Point2D(x * scalar, y * scalar); }
  Point2D operator+(const Vector2D& v) const;
  Point2D operator-(const Vector2D& v) const;
};

struct Vector2D {
  double x, y;
  Vector2D(double x = 0, double y = 0) : x(x), y(y) {}
  Vector2D(const Point2D& p) : x(p.x), y(p.y) {}
  Vector2D operator+(const Vector2D& other) const { return Vector2D(x + other.x, y + other.y); }
  Vector2D operator-(const Vector2D& other) const { return Vector2D(x - other.x, y - other.y); }
  Vector2D operator*(double scalar) const { return Vector2D(x * scalar, y * scalar); }
};

// Implement Point2D + Vector2D operations after Vector2D is defined
inline Point2D Point2D::operator+(const Vector2D& v) const { return Point2D(x + v.x, y + v.y); }
inline Point2D Point2D::operator-(const Vector2D& v) const { return Point2D(x - v.x, y - v.y); }

// Utility functions
inline double distance(const Point2D& a, const Point2D& b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

inline double magnitude(const Vector2D& v) { return std::sqrt(v.x * v.x + v.y * v.y); }

inline Vector2D normalize(const Vector2D& v) {
  double mag = magnitude(v);
  return (mag > 1e-9) ? Vector2D(v.x / mag, v.y / mag) : Vector2D(0, 0);
}

inline double dot(const Vector2D& a, const Vector2D& b) { return a.x * b.x + a.y * b.y; }

inline Point2D lerp(const Point2D& a, const Point2D& b, double t) {
  return Point2D(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y));
}

inline Vector2D lerp(const Vector2D& a, const Vector2D& b, double t) {
  return Vector2D(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y));
}

inline double lerp(double a, double b, double t) { return a + t * (b - a); }

// Forward declaration for VelocityPoint
struct VelocityPoint {
  double time;
  double arc_length;
  double speed;
  Point2D position;
  Vector2D direction;

  VelocityPoint() : time(0), arc_length(0), speed(0) {}
};

// Full AdaptiveTrajectoryFollower class implementation
class AdaptiveTrajectoryFollower {
 private:
  // Waypoint management
  std::deque<Point2D> remaining_waypoints;
  Point2D actual_robot_position;

  // Trajectory state with segmentation
  struct TrajectorySegment {
    tinyspline::BSpline spline;
    std::vector<VelocityPoint> velocity_profile;
    bool ends_at_stop;
    double total_time;

    TrajectorySegment() : ends_at_stop(true), total_time(0.0) {}
  };

  std::vector<TrajectorySegment> trajectory_segments;
  size_t current_segment_index;
  double trajectory_start_time;
  double current_time;
  bool trajectory_active;
  bool between_segments;
  double segment_pause_duration;

  // Control parameters
  double max_velocity;
  double max_acceleration;
  double max_lateral_acceleration;
  double position_tolerance;
  double replan_threshold;
  int min_replan_interval_ms;

  // Performance optimization
  bool needs_replanning;
  std::chrono::steady_clock::time_point last_replan_time;

  struct ArcPoint {
    double arc_length;
    Point2D position;
    Vector2D tangent;
    double spline_param;

    ArcPoint(double s, const Point2D& pos, const Vector2D& tan, double u)
        : arc_length(s), position(pos), tangent(tan), spline_param(u) {}
  };

 public:
  AdaptiveTrajectoryFollower(double max_vel, double max_acc, double pos_tol,
                             double replan_thresh = 0.2, int min_replan_interval_ms = 100)
      : max_velocity(max_vel),
        max_acceleration(max_acc),
        max_lateral_acceleration(max_acc * 0.8),
        position_tolerance(pos_tol),
        replan_threshold(replan_thresh),
        min_replan_interval_ms(min_replan_interval_ms),
        current_segment_index(0),
        trajectory_active(false),
        between_segments(false),
        needs_replanning(false),
        current_time(0.0),
        segment_pause_duration(0.1) {}

  // Main update function
  Vector2D update(const Point2D& robot_position, double dt = 0.05) {
    actual_robot_position = robot_position;

    if (shouldReplan()) {
      replanTrajectory();
    }

    updateWaypointProgress();
    return getVelocityCommand(dt);
  }

  void addWaypoints(const std::vector<Point2D>& new_waypoints) {
    for (const auto& wp : new_waypoints) {
      remaining_waypoints.push_back(wp);
    }

    while (remaining_waypoints.size() > 30) {
      remaining_waypoints.pop_back();
    }

    needs_replanning = true;
  }

  // Status queries
  bool isTrajectoryActive() const { return trajectory_active; }
  bool hasRemainingWaypoints() const { return !remaining_waypoints.empty(); }
  size_t getRemainingWaypointCount() const { return remaining_waypoints.size(); }

  // Parameter setters
  void setMaxVelocity(double vel) { max_velocity = vel; }
  void setMaxAcceleration(double acc) {
    max_acceleration = acc;
    max_lateral_acceleration = acc * 0.8;
  }
  void setPositionTolerance(double tol) { position_tolerance = tol; }
  void setReplanThreshold(double thresh) { replan_threshold = thresh; }

 private:
  bool shouldReplan() {
    if (needs_replanning) return true;
    if (!trajectory_active) return !remaining_waypoints.empty();

    Point2D ideal_position = getCurrentIdealPosition();
    double position_error = distance(actual_robot_position, ideal_position);

    if (position_error > replan_threshold) {
      auto now = std::chrono::steady_clock::now();
      auto time_since_last_replan =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - last_replan_time).count();

      return time_since_last_replan > min_replan_interval_ms;
    }

    return false;
  }

  void replanTrajectory() {
    if (remaining_waypoints.empty()) {
      trajectory_active = false;
      return;
    }

    std::vector<Point2D> planning_waypoints;
    planning_waypoints.push_back(actual_robot_position);

    for (const auto& wp : remaining_waypoints) {
      planning_waypoints.push_back(wp);
    }

    try {
      auto segment_waypoints = splitIntoSegments(planning_waypoints);

      trajectory_segments.clear();
      trajectory_segments.reserve(segment_waypoints.size());

      for (const auto& segment : segment_waypoints) {
        if (segment.size() >= 2) {
          TrajectorySegment traj_segment;
          std::vector<Point2D> processed = preprocessWaypoints(segment);

          createBSpline(processed, traj_segment.spline);
          generateVelocityProfile(traj_segment.spline, traj_segment.velocity_profile);

          traj_segment.ends_at_stop = true;
          traj_segment.total_time = traj_segment.velocity_profile.empty()
                                        ? 0.0
                                        : traj_segment.velocity_profile.back().time;

          trajectory_segments.push_back(traj_segment);
        }
      }

      current_segment_index = 0;
      between_segments = false;
      trajectory_start_time = current_time;
      trajectory_active = !trajectory_segments.empty();
      needs_replanning = false;
      last_replan_time = std::chrono::steady_clock::now();

    } catch (const std::exception& e) {
      std::cerr << "Replanning failed: " << e.what() << std::endl;
      trajectory_active = false;
    }
  }

  std::vector<std::vector<Point2D>> splitIntoSegments(const std::vector<Point2D>& waypoints) {
    std::vector<std::vector<Point2D>> segments;
    if (waypoints.size() < 2) return segments;

    std::vector<Point2D> current_segment;
    current_segment.push_back(waypoints[0]);

    for (size_t i = 1; i < waypoints.size(); i++) {
      current_segment.push_back(waypoints[i]);

      if (i < waypoints.size() - 1) {
        Vector2D v1 = normalize(waypoints[i] - waypoints[i - 1]);
        Vector2D v2 = normalize(waypoints[i + 1] - waypoints[i]);
        double dot_product = dot(v1, v2);

        if (dot_product < -0.5) {
          segments.push_back(current_segment);
          current_segment.clear();
          current_segment.push_back(waypoints[i]);
        }
      }
    }

    if (current_segment.size() >= 2) {
      segments.push_back(current_segment);
    }

    return segments;
  }

  void updateWaypointProgress() {
    while (!remaining_waypoints.empty()) {
      double dist_to_next = distance(actual_robot_position, remaining_waypoints.front());

      if (dist_to_next < position_tolerance) {
        remaining_waypoints.pop_front();
        needs_replanning = true;
      } else {
        break;
      }
    }

    if (remaining_waypoints.empty()) {
      trajectory_active = false;
    }
  }

  Point2D getCurrentIdealPosition() {
    if (!trajectory_active || trajectory_segments.empty() ||
        current_segment_index >= trajectory_segments.size()) {
      return actual_robot_position;
    }

    const auto& current_segment = trajectory_segments[current_segment_index];
    if (current_segment.velocity_profile.empty()) {
      return actual_robot_position;
    }

    double elapsed_time = current_time - trajectory_start_time;
    VelocityPoint current_state =
        interpolateProfile(current_segment.velocity_profile, elapsed_time);
    return current_state.position;
  }

  Vector2D getVelocityCommand(double dt) {
    current_time += dt;

    if (!trajectory_active || trajectory_segments.empty()) {
      return Vector2D(0, 0);
    }

    if (current_segment_index >= trajectory_segments.size()) {
      trajectory_active = false;
      needs_replanning = !remaining_waypoints.empty();
      return Vector2D(0, 0);
    }

    const auto& current_segment = trajectory_segments[current_segment_index];
    double elapsed_time = current_time - trajectory_start_time;

    if (elapsed_time >= current_segment.total_time) {
      if (current_segment.ends_at_stop) {
        if (!between_segments) {
          between_segments = true;
          trajectory_start_time = current_time;
          return Vector2D(0, 0);
        } else {
          if (current_time - trajectory_start_time >= segment_pause_duration) {
            current_segment_index++;
            between_segments = false;
            trajectory_start_time = current_time;

            if (current_segment_index >= trajectory_segments.size()) {
              trajectory_active = false;
              needs_replanning = !remaining_waypoints.empty();
              return Vector2D(0, 0);
            }
          } else {
            return Vector2D(0, 0);
          }
        }
      }
    }

    if (current_segment_index < trajectory_segments.size() && !between_segments) {
      const auto& segment = trajectory_segments[current_segment_index];
      if (!segment.velocity_profile.empty()) {
        VelocityPoint current_state = interpolateProfile(segment.velocity_profile, elapsed_time);
        return current_state.direction * current_state.speed;
      }
    }

    return Vector2D(0, 0);
  }

  void createBSpline(const std::vector<Point2D>& waypoints, tinyspline::BSpline& spline) {
    if (waypoints.size() < 2) return;

    std::vector<tinyspline::real> control_points;
    control_points.reserve(waypoints.size() * 2);

    for (const auto& wp : waypoints) {
      control_points.push_back(wp.x);
      control_points.push_back(wp.y);
    }

    int degree = std::min(3, static_cast<int>(waypoints.size() - 1));
    spline = tinyspline::BSpline(waypoints.size(), 2, degree);
    spline.setControlPoints(control_points);
  }

  std::vector<Point2D> preprocessWaypoints(const std::vector<Point2D>& waypoints) {
    if (waypoints.size() < 3) return waypoints;

    std::vector<Point2D> processed;
    processed.reserve(waypoints.size() * 2);
    processed.push_back(waypoints[0]);

    const double sharp_angle_threshold = 0.5;
    const double turning_radius = 0.2;

    for (size_t i = 1; i < waypoints.size() - 1; i++) {
      Vector2D v1 = normalize(waypoints[i] - waypoints[i - 1]);
      Vector2D v2 = normalize(waypoints[i + 1] - waypoints[i]);
      double dot_product = dot(v1, v2);

      if (dot_product < sharp_angle_threshold && dot_product > -0.5) {
        Point2D approach = waypoints[i] - v1 * turning_radius;
        Point2D exit = waypoints[i] + v2 * turning_radius;

        processed.push_back(approach);
        processed.push_back(waypoints[i]);
        processed.push_back(exit);
      } else {
        processed.push_back(waypoints[i]);
      }
    }

    processed.push_back(waypoints.back());
    return processed;
  }

  void generateVelocityProfile(const tinyspline::BSpline& spline,
                               std::vector<VelocityPoint>& profile) {
    profile.clear();

    const int num_samples = 50;
    std::vector<ArcPoint> arc_samples;
    arc_samples.reserve(num_samples + 1);

    double total_length = 0;
    auto eval_result = spline.eval(0.0);
    Point2D prev_pos(eval_result.result()[0], eval_result.result()[1]);

    for (int i = 0; i <= num_samples; i++) {
      double u = static_cast<double>(i) / num_samples;

      auto pos_result = spline.eval(u);
      Point2D pos(pos_result.result()[0], pos_result.result()[1]);

      if (i > 0) {
        total_length += distance(pos, prev_pos);
      }

      auto deriv_result = spline.derive().eval(u);
      Vector2D tangent(deriv_result.result()[0], deriv_result.result()[1]);

      arc_samples.emplace_back(total_length, pos, normalize(tangent), u);
      prev_pos = pos;
    }

    if (arc_samples.empty()) return;
    createVelocityProfile(spline, arc_samples, profile);
  }

  void createVelocityProfile(const tinyspline::BSpline& spline,
                             const std::vector<ArcPoint>& arc_samples,
                             std::vector<VelocityPoint>& profile) {
    profile.clear();
    profile.reserve(arc_samples.size());

    for (const auto& sample : arc_samples) {
      VelocityPoint point;
      point.arc_length = sample.arc_length;
      point.position = sample.position;
      point.direction = sample.tangent;

      double curvature = calculateCurvature(spline, sample.spline_param);
      if (curvature > 1e-6) {
        point.speed = std::min(max_velocity, std::sqrt(max_lateral_acceleration / curvature));
      } else {
        point.speed = max_velocity;
      }

      profile.push_back(point);
    }

    if (profile.empty()) return;

    applyAccelerationLimits(profile);
    convertToTimeProfile(profile);
  }

  double calculateCurvature(const tinyspline::BSpline& spline, double u) {
    try {
      auto first_deriv = spline.derive().eval(u);
      auto second_deriv = spline.derive().derive().eval(u);

      Vector2D v1(first_deriv.result()[0], first_deriv.result()[1]);
      Vector2D v2(second_deriv.result()[0], second_deriv.result()[1]);

      double cross = v1.x * v2.y - v1.y * v2.x;
      double speed_cubed = std::pow(magnitude(v1), 3);

      return (speed_cubed > 1e-9) ? std::abs(cross) / speed_cubed : 0.0;
    } catch (...) {
      return 0.0;
    }
  }

  void applyAccelerationLimits(std::vector<VelocityPoint>& profile) {
    if (profile.empty()) return;

    profile[0].speed = 0;

    for (size_t i = 1; i < profile.size(); i++) {
      double ds = profile[i].arc_length - profile[i - 1].arc_length;
      double v_prev = profile[i - 1].speed;

      if (ds > 1e-9) {
        double v_max_acc = std::sqrt(v_prev * v_prev + 2 * max_acceleration * ds);
        profile[i].speed = std::min(profile[i].speed, v_max_acc);
      }
    }

    profile.back().speed = 0;

    for (int i = static_cast<int>(profile.size()) - 2; i >= 0; i--) {
      double ds = profile[i + 1].arc_length - profile[i].arc_length;
      double v_next = profile[i + 1].speed;

      if (ds > 1e-9) {
        double v_max_dec = std::sqrt(v_next * v_next + 2 * max_acceleration * ds);
        profile[i].speed = std::min(profile[i].speed, v_max_dec);
      }
    }
  }

  void convertToTimeProfile(std::vector<VelocityPoint>& profile) {
    if (profile.empty()) return;

    double current_time = 0.0;
    profile[0].time = 0.0;

    for (size_t i = 1; i < profile.size(); i++) {
      double ds = profile[i].arc_length - profile[i - 1].arc_length;
      double avg_speed = (profile[i].speed + profile[i - 1].speed) / 2.0;

      if (avg_speed > 1e-6) {
        current_time += ds / avg_speed;
      } else {
        current_time += 0.05;
      }

      profile[i].time = current_time;
    }
  }

  VelocityPoint interpolateProfile(const std::vector<VelocityPoint>& profile, double query_time) {
    if (profile.empty()) return VelocityPoint();
    if (profile.size() == 1) return profile[0];

    if (query_time <= profile[0].time) return profile[0];
    if (query_time >= profile.back().time) return profile.back();

    auto it = std::lower_bound(profile.begin(), profile.end(), query_time,
                               [](const VelocityPoint& p, double t) { return p.time < t; });

    if (it == profile.begin()) return profile[0];

    auto it_prev = it - 1;
    double t_ratio = (query_time - it_prev->time) / (it->time - it_prev->time);

    VelocityPoint result;
    result.time = query_time;
    result.arc_length = lerp(it_prev->arc_length, it->arc_length, t_ratio);
    result.position = lerp(it_prev->position, it->position, t_ratio);
    result.direction = normalize(lerp(it_prev->direction, it->direction, t_ratio));
    result.speed = lerp(it_prev->speed, it->speed, t_ratio);

    return result;
  }
};

// Now implement the AdaptiveTrajectory wrapper class
namespace ctrl {

AdaptiveTrajectory::AdaptiveTrajectory(double max_velocity, double max_acceleration,
                                       double position_tolerance, double replan_threshold,
                                       int min_replan_interval_ms)
    : initialized_(false),
      target_yaw_(0.0),
      current_yaw_(0.0),
      active_traj_t_finish_s(0.0),
      last_velocity_(0, 0, 0) {
  follower_ = std::make_unique<AdaptiveTrajectoryFollower>(max_velocity, max_acceleration,
                                                           position_tolerance, replan_threshold,
                                                           min_replan_interval_ms);

  last_update_time_ = std::chrono::steady_clock::now();
}

AdaptiveTrajectory::~AdaptiveTrajectory() = default;

bool AdaptiveTrajectory::CreateTrajectoriesFromPath(std::vector<Eigen::Vector3d> path_fWorld,
                                                    double t_start_s) {
  std::unique_lock<std::mutex> lock(follower_mutex_);

  if (path_fWorld.size() <= 1) return false;

  // Convert 3D path to 2D waypoints (ignoring Z for now)
  std::vector<Point2D> waypoints_2d = convertPath(path_fWorld);

  // Set target yaw from the last waypoint (Z component represents yaw)
  if (!path_fWorld.empty()) {
    target_yaw_ = path_fWorld.back()[2];
  }

  // Add waypoints to the follower
  follower_->addWaypoints(waypoints_2d);

  // Update finish time estimate (rough approximation)
  double total_distance = 0.0;
  for (size_t i = 1; i < path_fWorld.size(); ++i) {
    Eigen::Vector3d diff = path_fWorld[i] - path_fWorld[i - 1];
    total_distance += std::sqrt(diff[0] * diff[0] + diff[1] * diff[1]);  // Only X,Y distance
  }

  // Estimate time based on average velocity (conservative estimate)
  double avg_velocity = 0.6;  // Conservative factor times max velocity
  active_traj_t_finish_s = t_start_s + (total_distance / avg_velocity);

  initialized_ = true;

  std::cout << "[AdaptiveTrajectory::CreateTrajectoriesFromPath] Added " << waypoints_2d.size()
            << " waypoints" << std::endl;

  return true;
}

void AdaptiveTrajectory::MergeNewTrajectories(Trajectories&& new_trajectories) {
  // Convert traditional trajectories to waypoints
  std::vector<Eigen::Vector3d> waypoints;

  while (!new_trajectories.empty()) {
    auto& traj = new_trajectories.front();

    // Add start and end positions of each trajectory segment
    waypoints.push_back(traj->GetPoseStart());
    waypoints.push_back(traj->GetPoseEnd());

    new_trajectories.pop();
  }

  if (!waypoints.empty()) {
    addWaypoints(waypoints);
  }
}

Eigen::Vector3d AdaptiveTrajectory::GetVelocityAtT(double time_s) {
  std::unique_lock<std::mutex> lock(follower_mutex_);

  if (!initialized_ || !follower_->isTrajectoryActive()) {
    return Eigen::Vector3d(0, 0, 0);
  }

  // Return the last computed velocity since the adaptive system
  // doesn't directly support time-based queries
  return last_velocity_;
}

std::pair<bool, Eigen::Vector3d> AdaptiveTrajectory::Update(Eigen::Vector3d pose_est) {
  std::unique_lock<std::mutex> lock(follower_mutex_);

  // Calculate dt
  auto current_time = std::chrono::steady_clock::now();
  auto dt_us =
      std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_update_time_);
  double dt = dt_us.count() / 1000000.0;  // Convert to seconds

  last_update_time_ = current_time;
  dt = std::min(dt, 0.1);  // Cap dt to prevent huge jumps

  if (!initialized_) {
    return std::make_pair(false, Eigen::Vector3d(0, 0, 0));
  }

  // Convert pose to 2D
  Point2D robot_pos_2d(pose_est[0], pose_est[1]);
  current_yaw_ = pose_est[2];

  // Update the follower
  Vector2D velocity_2d = follower_->update(robot_pos_2d, dt);

  // Convert back to 3D with yaw control
  Eigen::Vector3d velocity_3d(velocity_2d.x, velocity_2d.y, 0.0);

  // Simple yaw control towards target
  double yaw_error = target_yaw_ - current_yaw_;
  // Normalize angle to [-pi, pi]
  while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
  while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

  // Simple proportional yaw control
  velocity_3d[2] = yaw_error * 2.0;  // Proportional gain

  last_velocity_ = velocity_3d;
  last_pose_ = pose_est;

  bool is_active = follower_->isTrajectoryActive();

  return std::make_pair(is_active, velocity_3d);
}

void AdaptiveTrajectory::addWaypoints(const std::vector<Eigen::Vector3d>& new_waypoints) {
  std::unique_lock<std::mutex> lock(follower_mutex_);

  std::vector<Point2D> waypoints_2d = convertPath(new_waypoints);
  follower_->addWaypoints(waypoints_2d);

  if (!new_waypoints.empty()) {
    target_yaw_ = new_waypoints.back()[2];
  }
}

bool AdaptiveTrajectory::isTrajectoryActive() const {
  std::unique_lock<std::mutex> lock(follower_mutex_);
  return follower_->isTrajectoryActive();
}

bool AdaptiveTrajectory::hasRemainingWaypoints() const {
  std::unique_lock<std::mutex> lock(follower_mutex_);
  return follower_->hasRemainingWaypoints();
}

size_t AdaptiveTrajectory::getRemainingWaypointCount() const {
  std::unique_lock<std::mutex> lock(follower_mutex_);
  return follower_->getRemainingWaypointCount();
}

void AdaptiveTrajectory::setMaxVelocity(double vel) {
  std::unique_lock<std::mutex> lock(follower_mutex_);
  follower_->setMaxVelocity(vel);
}

void AdaptiveTrajectory::setMaxAcceleration(double acc) {
  std::unique_lock<std::mutex> lock(follower_mutex_);
  follower_->setMaxAcceleration(acc);
}

void AdaptiveTrajectory::setPositionTolerance(double tol) {
  std::unique_lock<std::mutex> lock(follower_mutex_);
  follower_->setPositionTolerance(tol);
}

void AdaptiveTrajectory::setReplanThreshold(double thresh) {
  std::unique_lock<std::mutex> lock(follower_mutex_);
  follower_->setReplanThreshold(thresh);
}

// Legacy interface support methods
Eigen::Vector3d AdaptiveTrajectory::FindV0AtT(double t) { return GetVelocityAtT(t); }

void AdaptiveTrajectory::MergeNewTrajectoriesFirstCall(Trajectories&& new_trajectories) {
  MergeNewTrajectories(std::move(new_trajectories));
}

void AdaptiveTrajectory::MergeNewTrajectoriesInFuture(Trajectories&& new_trajectories) {
  MergeNewTrajectories(std::move(new_trajectories));
}

void AdaptiveTrajectory::MergeNewTrajectoriesAtT(Trajectories&& new_trajectories) {
  MergeNewTrajectories(std::move(new_trajectories));
}

void AdaptiveTrajectory::Print() {
  std::unique_lock<std::mutex> lock(follower_mutex_);
  std::cout << "[AdaptiveTrajectory] Status:" << std::endl;
  std::cout << "  Active: " << follower_->isTrajectoryActive() << std::endl;
  std::cout << "  Remaining waypoints: " << follower_->getRemainingWaypointCount() << std::endl;
  std::cout << "  Target yaw: " << target_yaw_ << std::endl;
  std::cout << "  Current yaw: " << current_yaw_ << std::endl;
}

// Helper method implementation
std::vector<Point2D> AdaptiveTrajectory::convertPath(const std::vector<Eigen::Vector3d>& path) {
  std::vector<Point2D> waypoints_2d;
  waypoints_2d.reserve(path.size());

  for (const auto& point : path) {
    waypoints_2d.emplace_back(point[0], point[1]);  // X, Y only
  }

  return waypoints_2d;
}

}  // namespace ctrl