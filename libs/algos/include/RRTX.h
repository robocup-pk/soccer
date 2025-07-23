#ifndef ALGOS_RRTX_H
#define ALGOS_RRTX_H

#include <vector>
#include <queue>
#include <utility>
#include <limits>
#include "Waypoint.h"
// #include "Path.h"  

namespace algos {

struct NodeRRTX {
    state::Waypoint wp;
    int parent_idx;
    double g;
    double rhs;
    std::vector<int> neighbors;
};

class RRTX {
public:
    RRTX(const state::Waypoint& start, const state::Waypoint& goal);

    void SampleAndExpand();
    void UpdateRRTX();
    void ComputeShortPath();
    void InvalidateEdges(const state::Waypoint& moved_object_pos);
    void SetGoal(const state::Waypoint& new_goal);
    void SetStart(const state::Waypoint& new_start);
    state::Path ReconstructPath();

private:
    void UpdateVertex(int idx);
    state::Waypoint Sample(const state::Waypoint& goal, double angle);
    int Nearest(const std::vector<NodeRRTX>& tree, const state::Waypoint& wp);
    state::Waypoint Extend(const state::Waypoint& from, const state::Waypoint& to);
    double Distance(const state::Waypoint& a, const state::Waypoint& b);
    std::vector<int> Near(const std::vector<NodeRRTX>& tree, const state::Waypoint& wp);
    void AddNode(const state::Waypoint& wp);

    int start_idx;
    int goal_idx;
    state::Waypoint start;
    state::Waypoint goal;

    std::vector<NodeRRTX> nodes;

    std::priority_queue<std::pair<double, int>,
                        std::vector<std::pair<double, int>>,
                        std::greater<std::pair<double, int>>> pq;

    double goal_sample_rate = 0.1;
    double step_size = 0.3;
    double radius = 0.3;
};

}  // namespace algos

namespace algo {
    state::Path FindSinglePath_RRTX(state::Waypoint start, state::Waypoint goal);
}

#endif  // ALGOS_RRTX_H
