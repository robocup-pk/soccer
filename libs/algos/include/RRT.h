#ifndef RRT_H
#define RRT_H

#include <random>
#include <vector>
#include <cmath>
#include <algorithm>

#include "Waypoint.h"

namespace algos {

struct Node {
  state::Waypoint wp;
  int parent_index;

  Node(state::Waypoint& wp, int parent_index) : wp(wp), parent_index(parent_index) {}
};

state::Path FindSinglePath(const state::Waypoint& start, const state::Waypoint& goal);
state::Path ReconstructPath(const int goal_idx, const state::Waypoint& goal,
                            const std::vector<algos::Node>& path);
state::Waypoint Extend(const state::Waypoint& from, const state::Waypoint& to);
int FindNearestWaypointIdx(const state::Waypoint& random_wp, const std::vector<algos::Node>& path);
state::Waypoint FindRandomWaypoint(const state::Waypoint& goal);

extern std::mt19937 rng;
extern std::uniform_real_distribution<double> x_distribution;
extern std::uniform_real_distribution<double> y_distribution;
extern double step_size;
extern std::vector<algos::Node> path;
extern state::Waypoint goal_tolerance;

}  // namespace algos

#endif  // RRT_H