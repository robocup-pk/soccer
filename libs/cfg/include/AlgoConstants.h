#ifndef ALGO_CONSTANTS_H
#define ALGO_CONSTANTS_H

namespace cfg {

struct RRTXConstants {
  static const int max_vertices_size;
  static const double movement_threshold;
  static const double goal_bias;
  static const double epsilon;
  static const double delta;
  static const double gamma;
  static const double grid_cell_size;
};

struct AStarConstants {
  static const double grid_resolution;
  static const double obstacle_safety_margin;
  static const double movement_threshold;
};

}  // namespace cfg

#endif  // ALGO_CONSTANTS_H