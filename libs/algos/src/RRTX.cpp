#include <random>
#include <algorithm>
#include "RRTX.h"
#include <limits>
#include <vector>
#include <iostream>
#include "SoccerField.h"


namespace {
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> x_dist(-2037.84f, 2037.84f);
    std::uniform_real_distribution<float> y_dist(-1450.0f, 1450.0f);
    std::uniform_real_distribution<double> prob_dist(0.0, 1.0);
}

namespace algos {
  
  RRTX::RRTX(const state::Waypoint& start_, const state::Waypoint& goal_, double step_size_, double radius_)
    : start(start_), goal(goal_), step_size(step_size_), radius(radius_) {
    AddNode(start);
    start_idx = 0;

    nodes[start_idx].rhs = 0;  
    pq.push({0, start_idx});  

    AddNode(goal);
    goal_idx = 1;
}


// ----- Helper Functions -----

// Private functions for RRTX algorithm
state::Waypoint RRTX::Sample(const state::Waypoint& goal, double angle) {
    // Goal biasing
    
    std::uniform_real_distribution<double> prob_dist(0.0, 1.0);
    if (prob_dist(rng) < goal_sample_rate){
        return state::Waypoint{goal.x, goal.y, angle};}

    // Define field bounds
    float x_min = -2.03784f, x_max =  2.03784f;
    float y_min = -1.45f,    y_max =  1.45f;

    // Sample uniformly in range
    std::uniform_real_distribution<float> x_dist(x_min, x_max);
    std::uniform_real_distribution<float> y_dist(y_min, y_max);

    float x = x_dist(rng);
    float y = y_dist(rng);

    return state::Waypoint{x, y, angle};
}



 int RRTX::Nearest(const std::vector<NodeRRTX>& tree, const state::Waypoint& wp) {
    int nearest_idx = 0;
    double min_dist = Distance(tree[0].wp, wp);
    for (int i = 1; i < tree.size(); ++i) {
      double dist = Distance(tree[i].wp, wp);
      if (dist < min_dist) {
        min_dist = dist;
        nearest_idx = i;
      }
    }
    return nearest_idx;
  }

  
  state::Waypoint RRTX::Extend(const state::Waypoint& from, const state::Waypoint& to) {
    state::Waypoint dir = to - from;
    double dist = dir.Norm();
    if (dist <= step_size) return to;
    dir.Normalize();
    state::Waypoint new_wp;
    new_wp.x = from.x + dir.x * step_size;
    new_wp.y = from.y + dir.y * step_size;
    new_wp.angle = from.angle; // or whatever makes sense
    new_wp.angle = from.angle; // Ensure angle matches
    return new_wp;
  }
  
   double RRTX::Distance(const state::Waypoint& a, const state::Waypoint& b) {
    return (a - b).Norm();
  }


   std::vector<int> RRTX::Near(const std::vector<NodeRRTX>& tree, const state::Waypoint& wp) {
    std::vector<int> near;
    for (int i = 0; i < tree.size(); ++i) {
      if (Distance(tree[i].wp, wp) <= radius)
        near.push_back(i);
    }
    return near;
  }
  void RRTX::AddNode(const state::Waypoint& wp) {
      NodeRRTX node;
      node.wp = wp;
      node.g = std::numeric_limits<double>::infinity();
      node.rhs = std::numeric_limits<double>::infinity();
      node.parent_idx = -1;

      nodes.push_back(node);
      int idx = static_cast<int>(nodes.size() - 1);

      if (node.rhs != std::numeric_limits<double>::infinity()) pq.push({node.rhs, idx});

  }

  // Public function for RRTX algorithm     

void RRTX::SampleAndExpand() {
    state::Waypoint x_rand = Sample(goal, 0.2); // high goal bias

    int nearest_idx = Nearest(nodes, x_rand);
    state::Waypoint x_nearest = nodes[nearest_idx].wp;

    state::Waypoint x_new = Extend(x_nearest, x_rand);

    if (Distance(x_nearest, x_new) < 1e-3) return;

    AddNode(x_new);
    double connect_radius=0.2f;
    if (Distance(x_new, goal) < connect_radius) {
    // Add an edge between new node and goal
    nodes.back().neighbors.push_back(goal_idx);
    nodes[goal_idx].neighbors.push_back(nodes.size() - 1);

    // Call UpdateVertex to allow path propagation
    UpdateVertex(goal_idx);

}

    int new_idx = nodes.size() - 1;

    nodes[new_idx].g = std::numeric_limits<double>::infinity();
    nodes[new_idx].rhs = std::numeric_limits<double>::infinity();

    std::vector<int> near_idxs = Near(nodes, x_new);
    nodes[new_idx].neighbors = near_idxs;

    for (int near_idx : near_idxs) {
        double alt = nodes[near_idx].g + Distance(nodes[near_idx].wp, x_new);
        if (alt < nodes[new_idx].rhs) {
            nodes[new_idx].rhs = alt;
            nodes[new_idx].parent_idx = near_idx;
        }
    }

    pq.push({nodes[new_idx].rhs, new_idx});

    // Force goal connection
    if (Distance(x_new, nodes[goal_idx].wp) < radius) {
        double alt = nodes[new_idx].g + Distance(x_new, nodes[goal_idx].wp);
        if (alt < nodes[goal_idx].rhs) {
            nodes[goal_idx].rhs = alt;
            nodes[goal_idx].parent_idx = new_idx;
            pq.push({nodes[goal_idx].rhs, goal_idx});
            std::cout << "Goal connected!" << std::endl;
            UpdateRRTX();
        }
    }
    // std::cout << "[Sample] Total Nodes: " << nodes.size() << std::endl;
}



  void RRTX::UpdateRRTX() {
      while (!pq.empty()) {
          auto [rhs_cost, idx] = pq.top();
          pq.pop();

          NodeRRTX& node = nodes[idx];

          if (node.g > node.rhs) {
              node.g = node.rhs;

              for (int neighbor_idx : node.neighbors) {
                  NodeRRTX& neighbor = nodes[neighbor_idx];
                  double alt = node.g + Distance(node.wp, neighbor.wp);

                  if (alt < neighbor.rhs) {
                      neighbor.rhs = alt;
                      neighbor.parent_idx = idx;
                      pq.push({neighbor.rhs, neighbor_idx});
                  }
              }
          }
          else if (node.g < node.rhs) {
              node.g = std::numeric_limits<double>::infinity();

              for (int neighbor_idx : node.neighbors) {
                  NodeRRTX& neighbor = nodes[neighbor_idx];
                  double alt = neighbor.g + Distance(neighbor.wp, node.wp);

                  if (alt < node.rhs) {
                      node.rhs = alt;
                      node.parent_idx = neighbor_idx;
                  }
              }

              pq.push({node.rhs, idx});
          }
      }
  }
void RRTX::UpdateVertex(int idx) {
    if (idx == start_idx) return; 

    double min_rhs = std::numeric_limits<double>::infinity();
    int best_parent = -1;

    for (int neighbor : nodes[idx].neighbors) {
        double cost = nodes[neighbor].g + Distance(nodes[neighbor].wp, nodes[idx].wp);
        if (cost < min_rhs) {
            min_rhs = cost;
            best_parent = neighbor;
        }
    }

    nodes[idx].rhs = min_rhs;
    nodes[idx].parent_idx = best_parent;

    pq.push({min_rhs, idx}); 
}

void RRTX::ComputeShortPath() {
  while (
    !pq.empty() &&
    (nodes[start_idx].g != nodes[start_idx].rhs ||
     pq.top().first < std::min(nodes[start_idx].g, nodes[start_idx].rhs))
  ) {
    UpdateRRTX();
  }

}


void RRTX::InvalidateEdges(const state::Waypoint& moved_object_pos) {
    for (int i = 0; i < nodes.size(); ++i) {
        auto& node = nodes[i];
        if (Distance(node.wp, moved_object_pos) < radius) {
            node.g = std::numeric_limits<double>::infinity();
            node.rhs = std::numeric_limits<double>::infinity();
            node.parent_idx = -1;
            pq.push({node.rhs, i});  
        }
    }
    UpdateRRTX();  
}


state::Path RRTX::ReconstructPath() {
    state::Path path; 

    int current_idx = goal_idx;

    if (nodes[goal_idx].g == std::numeric_limits<double>::infinity()) {
        std::cout << "No path found!" << std::endl;
        return path; 
    }

    while (current_idx != -1) {
        path.push_back(nodes[current_idx].wp);
        current_idx = nodes[current_idx].parent_idx;
    }
    std::reverse(path.begin(), path.end());
    std::cout<<"this is right path"<<path<<std::endl;
    return path;
}
void RRTX::SetStart(const state::Waypoint& new_start) {
    start = new_start;

    // Reset and reinitialize tree
    nodes.clear();
    while (!pq.empty()) pq.pop();

    AddNode(start);
    start_idx = 0;

    nodes[start_idx].rhs = 0;
    pq.push({0, start_idx});

    // Re-add goal
    AddNode(goal);
    goal_idx = 1;
}
void RRTX::SetGoal(const state::Waypoint& new_goal) {
    goal = new_goal;

    // Optional: update goal node directly or find nearest
    nodes[goal_idx].wp = goal;
}

state::Path FindSinglePath_RRTX(state::Waypoint start, state::Waypoint goal, double step_size, double radius) {
    RRTX rrtx_planner(start, goal, step_size, radius);
    
    // Run multiple iterations to build the tree and find a path
    const int max_iterations = 1000;
    for (int i = 0; i < max_iterations; ++i) {
        rrtx_planner.SampleAndExpand();
        rrtx_planner.UpdateRRTX();
        
        // Check if path found every 50 iterations
        if (i % 50 == 0) {
            state::Path path = rrtx_planner.ReconstructPath();
            if (!path.empty()) {
                std::cout << "[RRTX] Path found after " << i+1 << " iterations with " << path.size() << " waypoints" << std::endl;
                return path;
            }
        }
    }
    
    // Final attempt to reconstruct path
    state::Path final_path = rrtx_planner.ReconstructPath();
    if (final_path.empty()) {
        std::cout << "[RRTX] No path found after " << max_iterations << " iterations" << std::endl;
    }
    return final_path;
}

}  // namespace algos