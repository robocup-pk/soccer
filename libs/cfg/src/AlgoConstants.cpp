#include "AlgoConstants.h"

namespace cfg {

// RRTX
const int RRTXConstants::max_vertices_size = 100;
const double RRTXConstants::movement_threshold = 0.05;
const double RRTXConstants::goal_bias = 0.2;
const double RRTXConstants::epsilon = 0.1;
const double RRTXConstants::delta = 0.5;
const double RRTXConstants::gamma = 1.5;
const double RRTXConstants::grid_cell_size = 0.2;

// A*
const double AStarConstants::grid_resolution = 0.05;
const double AStarConstants::obstacle_safety_margin = 0.1;
const double AStarConstants::movement_threshold = 0.05;

}  // namespace cfg
