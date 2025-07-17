#ifndef ALGOS_RRTX_H
#define ALGOS_RRTX_H

#include <vector>
#include "Waypoint.h"

namespace algos {

struct NodeRRTX {
    state::Waypoint wp;
    int parent_idx;
    double cost;
    NodeRRTX(const state::Waypoint& wp_, int parent_idx_, double cost_)
        : wp(wp_), parent_idx(parent_idx_), cost(cost_) {}
};

state::Path FindSinglePath_RRTX(const state::Waypoint& start, const state::Waypoint& goal);

}

#endif