#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <functional>
#include <algorithm>
#include <limits>

#include "Waypoint.h"
#include "SoccerObject.h"

namespace algos {

struct Node {
  int x, y;  // Grid coordinates
  double f, g, h;
  Node* parent;

  Node(int x = 0, int y = 0) : x(x), y(y), f(0), g(0), h(0), parent(nullptr) {}

  bool operator==(const Node& other) const { return x == other.x && y == other.y; }

  bool operator<(const Node& other) const {
    return f > other.f;  // For priority_queue, lower f has higher priority
  }
};

struct NodeHash {
  std::size_t operator()(const Node& node) const {
    return std::hash<int>()(node.x) ^ (std::hash<int>()(node.y) << 1);
  }
};

class AStar {
 public:
  AStar(float gridResolution = 0.05f);  // 5cm grid resolution

  // Main planning function
  state::Path findPath(const state::Waypoint& start, const state::Waypoint& goal,
                       const std::vector<state::SoccerObject>& obstacles);

  // Update the obstacle grid
  void updateObstacles(const std::vector<state::SoccerObject>& obstacles);

 private:
  // Grid properties
  float gridResolution;
  int gridWidth, gridHeight;
  float fieldWidth, fieldHeight;
  float fieldMinX, fieldMinY;

  // Obstacle grid (true = obstacle)
  std::vector<std::vector<bool>> obstacleGrid;

  // Convert between world coordinates and grid coordinates
  Node worldToGrid(const state::Waypoint& wp) const;
  state::Waypoint gridToWorld(const Node& node) const;

  // Heuristic function (Euclidean distance)
  float heuristic(const Node& a, const Node& b) const;

  // Check if a grid node is valid (within bounds and not obstructed)
  bool isValid(const Node& node) const;

  // Check if a path between two nodes is clear
  bool isPathClear(const Node& start, const Node& end) const;

  // Get neighboring nodes
  std::vector<Node> getNeighbors(const Node& node) const;

  // Smooth path using simple line-of-sight algorithm
  state::Path smoothPath(const state::Path& path) const;
};

}  // namespace algos

#endif  // ASTAR_SSL_H