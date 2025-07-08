#ifndef ALGOS_H
#define ALGOS_H

#include "Waypoint.h"
#include <vector>

namespace algos {

state::Waypoint FindDirectionVector(const state::Waypoint& start, const state::Waypoint& goal);
state::Path Astar(const state::Waypoint& start, const state::Waypoint& goal);
}  // namespace algos
#endif  // ALGOS_H