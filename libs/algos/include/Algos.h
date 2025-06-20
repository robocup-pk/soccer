#ifndef ALGOS_H
#define ALGOS_H

#include "Waypoint.h"

namespace algos {

state::Waypoint FindDirectionVector(const state::Waypoint& start, const state::Waypoint& goal);
}

#endif // ALGOS_H