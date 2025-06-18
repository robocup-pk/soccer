#include "Coordinates.h"

#include "RRT.h"

namespace algos {
std::mt19937 rng(std::random_device{}());
auto x_distribution = std::uniform_real_distribution<double>(0, cfg::Coordinates::field_width_ft);
auto y_distribution = std::uniform_real_distribution<double>(0, cfg::Coordinates::field_height_ft);
double step_size = 1;
}  // namespace algos

state::Path algos::FindSinglePath(const state::Waypoint& start, const state::Waypoint& goal) {
  state::Waypoint goal_tolerance(0.2, 0.2);
  std::vector<algos::Node> path;
  state::Waypoint start_cpy = start;
  Node start_node(start_cpy, -1);
  path.push_back(start_node);

  int iterations = 100;
  for (int iteration = 0; iteration < iterations; ++iteration) {
    state::Waypoint random_wp = FindRandomWaypoint(goal);
    int nearest_wp_idx = FindNearestWaypointIdx(random_wp, path);  // parent
    state::Waypoint new_wp = Extend(path[nearest_wp_idx].wp, random_wp);
    path.push_back(Node(new_wp, nearest_wp_idx));

    if ((new_wp - goal) > state::Waypoint(-goal_tolerance.x, -goal_tolerance.y) && (new_wp - goal) < goal_tolerance) {
      std::cout << "Found goal. wp: " << new_wp << ". goal: " << goal << std::endl;
      state::Path sol = ReconstructPath(path.size() - 1, goal, path);
      return sol;
    }
  }

  return state::Path();
}

state::Path algos::ReconstructPath(const int goal_idx, const state::Waypoint& goal,
                                   const std::vector<algos::Node>& path) {
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

state::Waypoint algos::Extend(const state::Waypoint& from, const state::Waypoint& to) {
  state::Waypoint direction_wp = to - from;
  double distance = direction_wp.Norm();

  if (distance <= step_size) {
    return to;
  }

  direction_wp.Normalize();
  return from + direction_wp * step_size;
}

int algos::FindNearestWaypointIdx(const state::Waypoint& random_wp, const std::vector<algos::Node>& path) {
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

state::Waypoint algos::FindRandomWaypoint(const state::Waypoint& goal) {
  std::uniform_real_distribution<double> bias_distribution(0, 1.0);

  if (bias_distribution(rng) < 0.1) {
    return goal;
  }

  return state::Waypoint(x_distribution(rng), y_distribution(rng));
}
