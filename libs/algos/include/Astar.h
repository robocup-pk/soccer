#ifndef AS_H
#define AS_H

#include <random>
#include <vector>
#include <cmath>
#include <algorithm>

#include "Waypoint.h"

namespace algos {
struct Node {
  float x, y;
  float g, h;
  state::Waypoint parentIndex;
};
struct pair_hash {
  std::size_t operator()(const std::pair<float, float>& p) const {
    return std::hash<float>()(p.first) ^ (std::hash<float>()(p.second) << 1);
  }
};
float heuristic(const Node& a, const Node& b);

}
#endif  // AS_H