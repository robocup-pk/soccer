#include <queue>
#include <cmath>
#include <iostream>

#include "AStar.h"
#include "SoccerField.h"
#include "AlgoConstants.h"

namespace algos {

AStar::AStar(float gridResolution) : gridResolution(gridResolution) {
  fieldWidth = vis::SoccerField::GetInstance().playing_area_width_mm / 1000.0f;
  fieldHeight = vis::SoccerField::GetInstance().playing_area_height_mm / 1000.0f;

  fieldMinX = -fieldWidth / 2.0f;
  fieldMinY = -fieldHeight / 2.0f;

  // Initialize grid dimensions
  gridWidth = static_cast<int>(std::ceil(fieldWidth / gridResolution));
  gridHeight = static_cast<int>(std::ceil(fieldHeight / gridResolution));

  // Initialize obstacle grid
  obstacleGrid.resize(gridWidth, std::vector<bool>(gridHeight, false));
}

void AStar::updateObstacles(const std::vector<state::SoccerObject>& obstacles) {
  // Clear previous obstacles
  for (int x = 0; x < gridWidth; ++x) {
    for (int y = 0; y < gridHeight; ++y) {
      obstacleGrid[x][y] = false;
    }
  }

  // Mark new obstacles
  for (const auto& obstacle : obstacles) {
    // Convert obstacle position to grid coordinates
    Node obstacleNode =
        worldToGrid(state::Waypoint(obstacle.position[0], obstacle.position[1], 0.0f));

    // Calculate obstacle radius in grid cells
    int radiusCells = static_cast<int>(std::ceil(obstacle.radius_m / gridResolution));

    // Mark cells within obstacle radius as occupied
    for (int dx = -radiusCells; dx <= radiusCells; ++dx) {
      for (int dy = -radiusCells; dy <= radiusCells; ++dy) {
        int x = obstacleNode.x + dx;
        int y = obstacleNode.y + dy;

        if (x >= 0 && x < gridWidth && y >= 0 && y < gridHeight) {
          // Check if within circular radius
          float dist = std::sqrt(dx * dx + dy * dy) * gridResolution;
          if (dist <= obstacle.radius_m + cfg::AStarConstants::obstacle_safety_margin) {  // Small safety margin
            obstacleGrid[x][y] = true;
          }
        }
      }
    }
  }
}

Node AStar::worldToGrid(const state::Waypoint& wp) const {
  int x = static_cast<int>((wp.x - fieldMinX) / gridResolution);
  int y = static_cast<int>((wp.y - fieldMinY) / gridResolution);

  // Clamp to grid bounds
  x = std::max(0, std::min(gridWidth - 1, x));
  y = std::max(0, std::min(gridHeight - 1, y));

  return Node(x, y);
}

state::Waypoint AStar::gridToWorld(const Node& node) const {
  float x = fieldMinX + (node.x + 0.5f) * gridResolution;
  float y = fieldMinY + (node.y + 0.5f) * gridResolution;
  return state::Waypoint(x, y, 0.0f);
}

float AStar::heuristic(const Node& a, const Node& b) const {
  // Euclidean distance
  float dx = (a.x - b.x) * gridResolution;
  float dy = (a.y - b.y) * gridResolution;
  return std::sqrt(dx * dx + dy * dy);
}

bool AStar::isValid(const Node& node) const {
  // Check if within bounds
  if (node.x < 0 || node.x >= gridWidth || node.y < 0 || node.y >= gridHeight) {
    return false;
  }

  // Check if not obstructed
  return !obstacleGrid[node.x][node.y];
}

std::vector<Node> AStar::getNeighbors(const Node& node) const {
  std::vector<Node> neighbors;
  neighbors.reserve(8);  // Reserve for 8 possible neighbors

  // Check all 8 directions
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;  // Skip current node

      Node neighbor(node.x + dx, node.y + dy);

      if (isValid(neighbor)) {
        neighbors.push_back(neighbor);
      }
    }
  }

  return neighbors;
}

state::Path AStar::findPath(const state::Waypoint& start, const state::Waypoint& goal,
                            const std::vector<state::SoccerObject>& obstacles) {
  // Update obstacles
  updateObstacles(obstacles);

  // Convert start and goal to grid coordinates
  Node startNode = worldToGrid(start);
  Node goalNode = worldToGrid(goal);

  // Check if start or goal is in obstacle
  if (!isValid(startNode) || !isValid(goalNode)) {
    return state::Path();
  }

  // Priority queue for open set
  std::priority_queue<Node> openSet;

  // Maps for storing g-scores and nodes
  std::unordered_map<Node, float, NodeHash> gScore;
  std::unordered_map<Node, Node*, NodeHash> nodeMap;

  // Initialize start node
  Node* startPtr = new Node(startNode.x, startNode.y);
  startPtr->g = 0;
  startPtr->h = heuristic(*startPtr, goalNode);
  startPtr->f = startPtr->g + startPtr->h;

  openSet.push(*startPtr);
  gScore[*startPtr] = 0;
  nodeMap[*startPtr] = startPtr;

  while (!openSet.empty()) {
    // Get node with lowest f-score
    Node current = openSet.top();
    openSet.pop();

    // Check if we reached the goal
    if (current.x == goalNode.x && current.y == goalNode.y) {
      // Reconstruct path
      state::Path path;
      Node* node = nodeMap[current];

      while (node != nullptr) {
        path.push_back(gridToWorld(*node));
        node = node->parent;
      }

      // Reverse to get start-to-goal order
      std::reverse(path.begin(), path.end());

      // Clean up
      for (auto& pair : nodeMap) {
        delete pair.second;
      }

      // Smooth path
      return smoothPath(path);
    }

    // Get neighbors
    std::vector<Node> neighbors = getNeighbors(current);

    for (const auto& neighbor : neighbors) {
      // Calculate tentative g-score
      float tentativeG = gScore[current] + heuristic(current, neighbor);

      // Check if we found a better path to this neighbor
      if (gScore.find(neighbor) == gScore.end() || tentativeG < gScore[neighbor]) {
        // Update neighbor node
        Node* neighborPtr;
        if (nodeMap.find(neighbor) == nodeMap.end()) {
          neighborPtr = new Node(neighbor.x, neighbor.y);
          nodeMap[neighbor] = neighborPtr;
        } else {
          neighborPtr = nodeMap[neighbor];
        }

        // Update scores and parent
        neighborPtr->g = tentativeG;
        neighborPtr->h = heuristic(*neighborPtr, goalNode);
        neighborPtr->f = neighborPtr->g + neighborPtr->h;
        neighborPtr->parent = nodeMap[current];

        // Update gScore
        gScore[neighbor] = tentativeG;

        // Add to open set
        openSet.push(*neighborPtr);
      }
    }
  }

  // Clean up
  for (auto& pair : nodeMap) {
    delete pair.second;
  }

  // No path found
  return state::Path();
}

state::Path AStar::smoothPath(const state::Path& path) const {
  if (path.size() < 3) {
    return path;
  }

  state::Path smoothed;
  smoothed.push_back(path[0]);

  size_t lastVisible = 0;

  for (size_t i = 1; i < path.size() - 1; ++i) {
    // Check if we can skip this point
    Node start = worldToGrid(smoothed.back());
    Node end = worldToGrid(path[i + 1]);

    if (!isPathClear(start, end)) {
      // Can't skip, add the current point
      smoothed.push_back(path[i]);
      lastVisible = i;
    }
  }

  // Add the goal
  smoothed.push_back(path.back());

  return smoothed;
}

bool AStar::isPathClear(const Node& start, const Node& end) const {
  // Bresenham's line algorithm to check if path is clear
  int x0 = start.x;
  int y0 = start.y;
  int x1 = end.x;
  int y1 = end.y;

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  while (true) {
    // Check if current cell is obstructed
    if (x0 < 0 || x0 >= gridWidth || y0 < 0 || y0 >= gridHeight || obstacleGrid[x0][y0]) {
      return false;
    }

    if (x0 == x1 && y0 == y1) {
      break;
    }

    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }

  return true;
}

}  // namespace algos