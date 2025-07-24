#ifndef RRT_H
#define RRT_H

#include <random>
#include <vector>
#include <cmath>
#include <algorithm>

#include "Waypoint.h"

namespace algos {

struct NodeRRT {
  state::Waypoint wp;
  int parent_index;

  NodeRRT(const state::Waypoint& wp, int parent_index)
      : wp(wp), parent_index(parent_index) {}
};

struct RRTParams {
  int max_iterations = 500;
  double step_size = 100.0;       // step size in mm
  double goal_bias = 0.5;         // probability of sampling the goal
  double goal_tolerance = 150.0;  // distance in mm to consider goal reached
  bool use_rrt_star = false;      // enable RRT* for optimal paths
  double near_radius = 200.0;     // radius for rewiring in RRT*
};

RRTParams DefaultRRTParams();

state::Path FindSinglePath(const state::Waypoint& start, const state::Waypoint& goal,
                          const RRTParams& params = DefaultRRTParams());
state::Path ReconstructPath(const int goal_idx, const state::Waypoint& goal,
                            const std::vector<algos::NodeRRT>& path);
state::Waypoint Extend(const state::Waypoint& from, const state::Waypoint& to, double step);
int FindNearestWaypointIdx(const state::Waypoint& random_wp,
                           const std::vector<algos::NodeRRT>& path);
state::Waypoint FindRandomWaypoint(const state::Waypoint& goal, double bias);
state::Path SmoothPath(const state::Path& original_path);

extern std::mt19937 rng;
extern std::uniform_real_distribution<double> x_distribution;
extern std::uniform_real_distribution<double> y_distribution;

}  // namespace algos

#endif  // RRT_H