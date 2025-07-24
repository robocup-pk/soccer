#include "SoccerField.h"
#include "RRT.h"
#include <Eigen/Dense>

namespace algos {
std::mt19937 rng(std::random_device{}());
std::uniform_real_distribution<double> x_distribution =
    std::uniform_real_distribution<double>(0, vis::SoccerField::GetInstance().width_mm);
std::uniform_real_distribution<double> y_distribution =
    std::uniform_real_distribution<double>(0, vis::SoccerField::GetInstance().height_mm);

RRTParams DefaultRRTParams() { return RRTParams(); }
}  // namespace algos

state::Path algos::FindSinglePath(const state::Waypoint& start, const state::Waypoint& goal,
                                  const RRTParams& params) {
  double goal_tolerance = params.goal_tolerance;
  std::vector<algos::NodeRRT> path;
  state::Waypoint start_cpy = start;
  NodeRRT start_NodeRRT(start_cpy, -1);
  path.push_back(start_NodeRRT);

  // Debug output
  std::cout << "[RRT] Starting path planning from (" << start.x << ", " << start.y 
            << ") to (" << goal.x << ", " << goal.y << ")" << std::endl;

  int max_iterations = params.max_iterations;  // Balanced iterations for reasonable performance
  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    state::Waypoint random_wp = FindRandomWaypoint(goal, params.goal_bias);
    int nearest_wp_idx = FindNearestWaypointIdx(random_wp, path);  // parent
    double dynamic_step = params.step_size;
    double dist_to_goal = (path[nearest_wp_idx].wp - goal).Norm();
    if (dist_to_goal < params.step_size) dynamic_step = dist_to_goal;
    state::Waypoint new_wp = Extend(path[nearest_wp_idx].wp, random_wp, dynamic_step);
    
    // Skip if new waypoint is same as parent (no progress)
    if ((new_wp - path[nearest_wp_idx].wp).Norm() < 1.0) {
      continue;
    }
    
    path.push_back(NodeRRT(new_wp, nearest_wp_idx));

    // Correct goal checking: check euclidean distance
    double distance_to_goal = (new_wp - goal).Norm();
    if (distance_to_goal <= goal_tolerance) {
      std::cout << "[RRT] Goal reached! Distance: " << distance_to_goal 
                << "mm, Iterations: " << iteration + 1 << std::endl;
      state::Path sol = ReconstructPath(path.size() - 1, goal, path);
      
      // Smooth path by removing unnecessary intermediate waypoints
      state::Path smoothed_path = SmoothPath(sol);
      std::cout << "[RRT] Path smoothed from " << sol.size() << " to " 
                << smoothed_path.size() << " waypoints" << std::endl;
      return smoothed_path;
    }
    
    // Progress reporting every 100 iterations
    if ((iteration + 1) % 100 == 0) {
      std::cout << "[RRT] Iteration " << iteration + 1 << ", closest distance: " 
                << distance_to_goal << "mm" << std::endl;
    }
  }

  std::cout << "[RRT] Failed to find path after " << max_iterations << " iterations" << std::endl;
  return state::Path();
}

state::Path algos::ReconstructPath(const int goal_idx, const state::Waypoint& goal,
                                   const std::vector<algos::NodeRRT>& path) {
  state::Path sol;
  sol.push_back(goal);

  int current_idx = goal_idx;
  while (current_idx != -1) {
    sol.push_back(path[current_idx].wp);
    current_idx = path[current_idx].parent_index;
  }

  std::reverse(sol.begin(), sol.end());
  return sol;
}

state::Waypoint algos::Extend(const state::Waypoint& from, const state::Waypoint& to, double step) {
  state::Waypoint direction_wp = to - from;
  double distance = direction_wp.Norm();

  if (distance <= step) {
    return to;
  }

  direction_wp.Normalize();
  state::Waypoint new_wp = from + direction_wp * step;
  new_wp.angle = from.angle;
  return new_wp;
}

int algos::FindNearestWaypointIdx(const state::Waypoint& random_wp,
                                  const std::vector<algos::NodeRRT>& path) {
  double min_distance = (random_wp - path[0].wp).Norm();
  int nearest_wp_idx = 0;

  for (int i = 0; i < path.size(); ++i) {
    double distance = (path[i].wp - random_wp).Norm();
    if (distance < min_distance) {
      min_distance = distance;
      nearest_wp_idx = i;
    }
  }

  return nearest_wp_idx;
}

state::Waypoint algos::FindRandomWaypoint(const state::Waypoint& goal, double bias) {
  std::uniform_real_distribution<double> bias_distribution(0, 1.0);

  if (bias_distribution(rng) < bias) {
    return goal;
  }

  return state::Waypoint(x_distribution(rng), y_distribution(rng));
}

state::Path algos::SmoothPath(const state::Path& original_path) {
  if (original_path.size() <= 2) {
    return original_path;
  }

  state::Path smoothed_path;
  smoothed_path.push_back(original_path.front());

  // Remove waypoints that are approximately on a straight line
  const double distance_threshold = 20.0;  // mm

  for (size_t i = 1; i < original_path.size() - 1; ++i) {
    state::Waypoint prev = smoothed_path.back();
    state::Waypoint next = original_path[i + 1];
    state::Waypoint curr = original_path[i];

    Eigen::Vector2d p(prev.x, prev.y);
    Eigen::Vector2d q(next.x, next.y);
    Eigen::Vector2d r(curr.x, curr.y);

    Eigen::Vector2d line = q - p;
    double line_mag = line.norm();
    if (line_mag > 1e-6) {
      double distance = std::abs((q.x() - p.x()) * (p.y() - r.y()) - (p.x() - r.x()) * (q.y() - p.y())) / line_mag;
      if (distance < distance_threshold) {
        continue;  // skip nearly collinear waypoint
      }
    }
    smoothed_path.push_back(curr);
  }

  smoothed_path.push_back(original_path.back());
  return smoothed_path;
}
