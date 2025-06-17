#include <random>
#include <vector>
#include <cmath>
#include <algorithm>

#include "Coordinates.h"

#include "RRT.h"

std::mt19937 rng(std::random_device{}());
auto x_distribution = std::uniform_real_distribution<double>(0, cfg::Coordinates::field_width_ft);
auto y_distribution = std::uniform_real_distribution<double>(0, cfg::Coordinates::field_height_ft);

double step_size = 1;

std::vector<algos::Node> path;
state::Waypoint goal_tolerance(0.2, 0.2);

state::Path algos::FindSinglePath(const state::Waypoint& start, const state::Waypoint& goal) {
  state::Waypoint start_cpy = start;
  Node start_node(start_cpy, nullptr);
  path.push_back(start_node);

  int iterations = 5;
  for (int iteration = 0; iteration < iterations; ++iteration) {
    state::Waypoint random_wp = FindRandomWaypoint(goal);
    int nearest_wp_idx = FindNearestWaypointIdx(random_wp);  // parent
    state::Waypoint new_wp = Extend(path[nearest_wp_idx].wp, random_wp);
    path.push_back(Node(new_wp, &path[nearest_wp_idx].wp));

    if (new_wp - goal < goal_tolerance) {
      return ReconstructPath(path, goal);
    }
  }

  return state::Path();
}

state::Path ReconstructPath(std::vector<algos::Node>& path, const state::Waypoint& goal) {
  state::Path sol;
  sol.push_back(goal);

  Node wp(goal);

  int current_idx = goal_idx;
  while (current_idx != -1) {
    path.push_back(tree[current_idx].position);
    current_idx = tree[current_idx].parent_index;
  }

  std::reverse(sol.begin(), sol.end());
  return sol;
}

state::Waypoint Extend(const state::Waypoint& from, const state::Waypoint& to) {
  state::Waypoint direction_wp = to - from;
  double distance = direction_wp.Norm();

  if (distance <= step_size) {
    return to;
  }

  direction_wp.Normalize();
  return from + direction_wp * step_size;
}

int FindNearestWaypointIdx(const state::Waypoint& random_wp) {
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

state::Waypoint FindRandomWaypoint(const state::Waypoint& goal) {
  std::uniform_real_distribution<double> bias_distribution(0, 1.0);

  if (bias_distribution(rng) < 0.1) {
    return goal;
  }

  return state::Waypoint(x_distribution(rng), y_distribution(rng));
}
