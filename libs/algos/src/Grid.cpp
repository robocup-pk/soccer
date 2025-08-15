#include "Grid.h"
#include "Waypoint.h"
#include "RRTX.h"
#include "SoccerField.h"

algos::SpatialGrid::SpatialGrid(double cell_sz) : cell_size(cell_sz) {
  field_width = vis::SoccerField::GetInstance().playing_area_width_mm / 1000.0;
  field_height = vis::SoccerField::GetInstance().playing_area_height_mm / 1000.0;

  grid_width = static_cast<int>(std::ceil(field_width / cell_size));
  grid_height = static_cast<int>(std::ceil(field_height / cell_size));

  // Mapping grid coordinates to world coordinates
  origin_x = -field_width / 2.0;
  origin_y = -field_height / 2.0;

  // Initialize grid
  grid.resize(grid_width, std::vector<GridCell>(grid_height));
}

std::pair<int, int> algos::SpatialGrid::WorldToGrid(state::Waypoint& wp) {
  int x = static_cast<int>((wp.x - origin_x) / cell_size);
  int y = static_cast<int>((wp.y - origin_y) / cell_size);

  // Clamp to valid range
  x = std::max(0, std::min(x, grid_width - 1));
  y = std::max(0, std::min(y, grid_height - 1));

  return {x, y};
}

int algos::SpatialGrid::FindNearest(state::Waypoint& query_point, std::vector<Vertex>& vertices) {
  auto [query_x, query_y] = WorldToGrid(query_point);

  int best_idx = -1;
  double best_dist = std::numeric_limits<double>::infinity();

  // Expanding square search pattern
  for (int radius = 0; radius < std::max(grid_width, grid_height); ++radius) {
    bool found_in_ring = false;

    for (int dx = -radius; dx <= radius; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        // Only check ring cells, not interior
        if (radius > 0 && std::abs(dx) < radius && std::abs(dy) < radius) continue;

        int x = query_x + dx;
        int y = query_y + dy;

        if (!IsValidGridPos(x, y)) continue;

        // Check all vertices in this cell
        for (int vertex_idx : grid[x][y].vertex_indices) {
          if (vertex_idx >= vertices.size() || !vertices[vertex_idx].alive) continue;
          if (vertices[vertex_idx].wp == query_point) continue;

          double dist = (query_point - vertices[vertex_idx].wp).Norm();
          if (dist < best_dist) {
            best_dist = dist;
            best_idx = vertex_idx;
            found_in_ring = true;
          }
        }
      }
    }

    // If we found vertices in this ring, and the ring is complete,
    // we can guarantee this is the nearest
    if (found_in_ring && radius > 0) {
      double ring_min_dist = radius * cell_size;
      if (best_dist <= ring_min_dist) {
        break;  // Guaranteed nearest found
      }
    }
  }

  return best_idx;
}

std::vector<int> algos::SpatialGrid::FindNear(state::Waypoint& query_point, double radius,
                                              std::vector<Vertex>& vertices) {
  std::vector<int> result;

  // Get cells that intersect the query radius
  std::vector<std::pair<int, int>> cells = GetCellsInRadius(query_point, radius);

  for (auto [x, y] : cells) {
    for (int vertex_idx : grid[x][y].vertex_indices) {
      if (vertex_idx >= vertices.size() || !vertices[vertex_idx].alive) continue;
      if (vertices[vertex_idx].wp == query_point) continue;

      double dist = (query_point - vertices[vertex_idx].wp).Norm();
      if (dist <= radius) {
        result.push_back(vertex_idx);
      }
    }
  }

  return result;
}

std::vector<std::pair<int, int>> algos::SpatialGrid::GetCellsInRadius(state::Waypoint& center,
                                                                      double radius) {
  std::vector<std::pair<int, int>> cells;

  auto [center_x, center_y] = WorldToGrid(center);
  int cell_radius = static_cast<int>(std::ceil(radius / cell_size));

  for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
    for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
      int x = center_x + dx;
      int y = center_y + dy;

      if (IsValidGridPos(x, y)) {
        cells.push_back({x, y});
      }
    }
  }

  return cells;
}

void algos::SpatialGrid::AddVertex(int vertex_idx, state::Waypoint& wp) {
  auto [x, y] = WorldToGrid(wp);
  if (IsValidGridPos(x, y)) {
    grid[x][y].addVertex(vertex_idx);
  }
}

void algos::SpatialGrid::RemoveVertex(int vertex_idx, state::Waypoint& wp) {
  auto [x, y] = WorldToGrid(wp);
  if (IsValidGridPos(x, y)) {
    grid[x][y].removeVertex(vertex_idx);
  }
}

void algos::SpatialGrid::UpdateVertex(int vertex_idx, state::Waypoint& old_wp,
                                      state::Waypoint& new_wp) {
  auto [old_x, old_y] = WorldToGrid(old_wp);
  auto [new_x, new_y] = WorldToGrid(new_wp);

  if (old_x != new_x || old_y != new_y) {
    if (IsValidGridPos(old_x, old_y)) {
      grid[old_x][old_y].removeVertex(vertex_idx);
    }
    if (IsValidGridPos(new_x, new_y)) {
      grid[new_x][new_y].addVertex(vertex_idx);
    }
  }
}

void algos::SpatialGrid::Clear() {
  for (auto& row : grid) {
    for (auto& cell : row) {
      cell.clear();
    }
  }
}

bool algos::SpatialGrid::IsValidGridPos(int x, int y) {
  return x >= 0 && x < grid_width && y >= 0 && y < grid_height;
}