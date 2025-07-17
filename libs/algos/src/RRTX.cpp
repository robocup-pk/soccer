#include <random>
#include <algorithm>
#include "RRTX.h"
#include "SoccerField.h"

namespace algos {

// Random number generator
std::mt19937 rng(std::random_device{}());
std::uniform_real_distribution<double> x_distribution(0, vis::SoccerField::GetInstance().width_mm);
std::uniform_real_distribution<double> y_distribution(0, vis::SoccerField::GetInstance().height_mm);

// Constants
constexpr double step_size = 1.0;
constexpr double goal_sample_rate = 0.1;
constexpr double radius = 50.0;

// ----- Helper Functions -----

static double Distance(const state::Waypoint& a, const state::Waypoint& b) {
  return (a - b).Norm();
}

static state::Waypoint Extend(const state::Waypoint& from, const state::Waypoint& to) {
  state::Waypoint dir = to - from;
  double dist = dir.Norm();
  if (dist <= step_size) return to;
  dir.Normalize();
  return from + dir * step_size;
}

static state::Waypoint Sample(const state::Waypoint& goal) {
  if (std::uniform_real_distribution<double>(0.0, 1.0)(rng) < goal_sample_rate)
    return goal;
  return { x_distribution(rng), y_distribution(rng) };
}

static int Nearest(const std::vector<NodeRRTX>& tree, const state::Waypoint& wp) {
  int nearest_idx = 0;
  double min_dist = Distance(tree[0].wp, wp);
  for (int i = 1; i < tree.size(); ++i) {
    double dist = Distance(tree[i].wp, wp);
    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = i;
    }
  }
  return nearest_idx;
}

static std::vector<int> Near(const std::vector<NodeRRTX>& tree, const state::Waypoint& wp) {
  std::vector<int> near;
  for (int i = 0; i < tree.size(); ++i) {
    if (Distance(tree[i].wp, wp) <= radius)
      near.push_back(i);
  }
  return near;
}

static state::Path ReconstructPath(const std::vector<NodeRRTX>& tree, int goal_idx) {
  state::Path path;
  for (int curr = goal_idx; curr != -1; curr = tree[curr].parent_idx)
    path.push_back(tree[curr].wp);
  std::reverse(path.begin(), path.end());
  return path;
}

// ----- Main RRT-X Function -----

state::Path FindSinglePath_RRTX(const state::Waypoint& start, const state::Waypoint& goal) {
  std::vector<NodeRRTX> tree;
  tree.emplace_back(start, -1, 0.0);

  const state::Waypoint goal_tolerance(0.2, 0.2);
  constexpr int max_iterations = 500;

  for (int i = 0; i < max_iterations; ++i) {
    state::Waypoint rand_wp = Sample(goal);
    int nearest_idx = Nearest(tree, rand_wp);
    state::Waypoint new_wp = Extend(tree[nearest_idx].wp, rand_wp);

    double new_cost = tree[nearest_idx].cost + Distance(tree[nearest_idx].wp, new_wp);
    std::vector<int> near_nodes = Near(tree, new_wp);

    int best_parent = nearest_idx;
    for (int idx : near_nodes) {
      double alt_cost = tree[idx].cost + Distance(tree[idx].wp, new_wp);
      if (alt_cost < new_cost) {
        new_cost = alt_cost;
        best_parent = idx;
      }
    }

    int new_idx = tree.size();
    tree.emplace_back(new_wp, best_parent, new_cost);

    // Rewire
    for (int idx : near_nodes) {
      double alt_cost = tree[new_idx].cost + Distance(tree[new_idx].wp, tree[idx].wp);
      if (alt_cost < tree[idx].cost) {
        tree[idx].parent_idx = new_idx;
        tree[idx].cost = alt_cost;
      }
    }

    // Goal reached (within tolerance)
    state::Waypoint delta = new_wp - goal;
    if (std::abs(delta.x) <= goal_tolerance.x && std::abs(delta.y) <= goal_tolerance.y)
      return ReconstructPath(tree, new_idx);
  }

  return {};
}

}  // namespace algos
