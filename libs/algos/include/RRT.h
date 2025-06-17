#ifndef RRT_H
#define RRT_H

#include "Waypoint.h"

namespace algos {

struct Node {
  state::Waypoint wp;
  state::Waypoint* parent;

  Node(state::Waypoint& wp, state::Waypoint* parent) : wp(wp), parent(parent) {}
};

state::Path FindSinglePath(const state::Waypoint& start, const state::Waypoint& goal);

}  // namespace algos

#endif  // RRT_H