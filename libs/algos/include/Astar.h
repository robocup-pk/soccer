#ifndef AS_H
#define AS_H

#include <random>
#include <vector>
#include <cmath>
#include <algorithm>

#include "Waypoint.h"
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
float heuristic(const Node& a, const Node& b) {
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
};
#endif  // AS_H