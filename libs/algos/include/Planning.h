#ifndef PLANNING_H
#define PLANNING_H

#include "Waypoint.h"
#include "SoccerObject.h"
#include <vector>

namespace algos {
std::vector<state::Path> ComputeRobotPaths(std::vector<state::SoccerObject>& soccer_objects);
bool TryToScore(state::SoccerObject soccer_object);
}  // namespace algos
#endif  // PLANNING_H