#include "Algos.h"
#include "Astar.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <algorithm>
#include <set>
#include <functional>

namespace algos {

state::Path Astar(const state::Waypoint& start, const state::Waypoint& goal) {
  auto cmp = [](const Node& a, const Node& b) { return a.g + a.h > b.g + b.h; };
  std::priority_queue<Node, std::vector<Node>, decltype(cmp)> openSet(cmp);
  std::unordered_map<std::pair<float, float>, Node, pair_hash> closedSet;

  Node startNode;
  startNode.x = start.x;
  startNode.y = start.y;
  startNode.g = 0.0f;
  Node goalNode;
  goalNode.x = goal.x;
  goalNode.y = goal.y;
  startNode.h = heuristic(startNode, goalNode);
  startNode.parentIndex = state::Waypoint(-1, -1, 0);

  openSet.push(startNode);

  float neighbours[8][2] = {
      {0.2, 0},     // Right
      {0.0, 0.2},   // Up
      {-0.2, 0},    // Left
      {0, -0.2},    // Down
      {0.2, 0.2},   // Up-Right
      {-0.2, 0.2},  // Up-Left
      {0.2, -0.2},  // Down-Right
      {-0.2, -0.2}  // Down-Left
  };
  while (!openSet.empty()) {
    Node current = openSet.top();
    openSet.pop();
    // std::cout << fabs(current.x - goalNode.x) << ' ' << fabs(current.y - goalNode.y) <<
    // std::endl;
    if (fabs(current.x - goalNode.x) <= 0.3 && fabs(current.y - goalNode.y) <= 0.3) {
      // Reconstruct path
      // std::cout << 1 << std::endl;
      std::vector<state::Waypoint> path = {(state::Waypoint(goalNode.x, goalNode.y, 0.0))};
      while (current.parentIndex.x != -1) {
        path.push_back(state::Waypoint(current.x, current.y, 0));
        auto it = closedSet.find({current.parentIndex.x, current.parentIndex.y});
        if (it == closedSet.end()) break;
        current = it->second;
      }
      std::reverse(path.begin(), path.end());
      return path;
    }
    closedSet[{current.x, current.y}] = current;
    for (const auto& neighbour : neighbours) {
      Node nextNode;
      nextNode.x = current.x + neighbour[0];
      nextNode.y = current.y + neighbour[1];
      nextNode.g = current.g + 0.2;
      nextNode.h = heuristic(nextNode, goalNode);
      nextNode.parentIndex = state::Waypoint(current.x, current.y, 0);

      if (closedSet.find({nextNode.x, nextNode.y}) != closedSet.end()) {
        continue;
      }
      openSet.push(nextNode);
    }
  }

  return {state::Waypoint(goalNode.x, goalNode.y, 0.0)};  // Placeholder return value
}

state::Path Dstar(const state::Waypoint& start, const state::Waypoint& goal) {
  // Placeholder for D* algorithm implementation

  return {state::Waypoint(goal.x, goal.y, 0.0)};
}
}  // namespace algos