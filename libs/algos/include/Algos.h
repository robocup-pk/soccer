#ifndef ALGOS_H
#define ALGOS_H

#include <vector>

#include "Waypoint.h"
#include "SoccerObject.h"

namespace algos {

enum class AlgoName { RRTX, ASTAR };

state::Waypoint FindDirectionVector(const state::Waypoint& start, const state::Waypoint& goal);
state::Path Astar(const state::Waypoint& start, const state::Waypoint& goal);
state::Waypoint SelectGoal(std::vector<state::SoccerObject>& soccer_objects);
state::Path PlanPath(AlgoName algo_name, std::vector<state::SoccerObject>& soccer_objects);

}  // namespace algos
#endif  // ALGOS_H