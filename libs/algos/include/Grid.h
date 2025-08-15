#ifndef GRID_H
#define GRID_H

#include "Waypoint.h"

#include <vector>

namespace algos {
struct Vertex;  // Forward declaration

class SpatialGrid {
 public:
  SpatialGrid(double cell_size);
  void AddVertex(int vertex_idx, state::Waypoint& wp);
  void RemoveVertex(int vertex_idx, state::Waypoint& wp);
  void UpdateVertex(int vertex_idx, state::Waypoint& old_wp, state::Waypoint& new_wp);

  // Query operations
  int FindNearest(state::Waypoint& query_point, std::vector<algos::Vertex>& vertices);
  std::vector<int> FindNear(state::Waypoint& query_point, double radius,
                            std::vector<algos::Vertex>& vertices);
  std::vector<int> GetCandidatesInRadius(state::Waypoint& center, double radius);

  // Edge management for collision optimization
  void AddEdge(int edge_id, state::Waypoint& start, state::Waypoint& end);
  void RemoveEdge(int edge_id, state::Waypoint& start, state::Waypoint& end);
  std::vector<int> GetEdgesIntersectingRegion(state::Waypoint& center, double radius);

  // Utility functions
  void Clear();
  std::pair<int, int> WorldToGrid(state::Waypoint& wp);
  bool IsValidGridPos(int x, int y);
  std::vector<std::pair<int, int>> GetCellsInRadius(state::Waypoint& center, double radius);

 private:
  struct GridCell {
    std::vector<int> vertex_indices;
    std::vector<int> edge_indices;  // For edge-obstacle collision optimization

    void clear() {
      vertex_indices.clear();
      edge_indices.clear();
    }

    void addVertex(int idx) { vertex_indices.push_back(idx); }

    void removeVertex(int idx) {
      auto it = std::find(vertex_indices.begin(), vertex_indices.end(), idx);
      if (it != vertex_indices.end()) {
        std::swap(*it, vertex_indices.back());
        vertex_indices.pop_back();
      }
    }
  };

  double cell_size;
  std::vector<std::vector<GridCell>> grid;
  double field_width, field_height;
  int grid_width, grid_height;
  double origin_x, origin_y;
};

}  // namespace algos

#endif  // GRID_H