#ifndef RRTX_H
#define RRTX_H

#include <vector>
#include <Eigen/Dense>
#include <queue>
#include <unordered_set>
#include <random>
#include <algorithm>
#include "Waypoint.h"

namespace algos {

struct Vertex {
  state::Waypoint wp;          // v ∈ X
  int parent_idx;              // p_T⁺(v) — parent in shortest-path tree T
  double g;                    // g(v) — ε-consistent cost-to-goal
  double lmc;                  // lmc(v) — look-ahead estimate of cost-to-goal
  std::vector<int> N_plus_0;   // N₀⁺(v) — original outgoing neighbors
  std::vector<int> N_plus_r;   // Nᵣ⁺(v) — running outgoing neighbors
  std::vector<int> N_minus_0;  // N₀⁻(v) — original incoming neighbors
  std::vector<int> N_minus_r;  // Nᵣ⁻(v) — running incoming neighbors
  std::vector<int> C_minus_T;  // C_T⁻(v) — children in tree T

  Vertex()
      : parent_idx(-1),
        g(std::numeric_limits<double>::infinity()),
        lmc(std::numeric_limits<double>::infinity()) {}
};

class RRTX {
 public:
  RRTX(const state::Waypoint& x_start, const state::Waypoint& x_goal, double epsilon = 0.1);
  // Core functions
  int Extend(state::Waypoint v_new, double r);               // Algorithm 2
  void CullNeighbors(int v_idx, double r);                   // Algorithm 3
  void RewireNeighbors(int v_idx);                           // Algorithm 4
  void ReduceInconsistency();                                // Algorithm 5
  void FindParent(Vertex &v_new, const std::vector<int>& U);  // Algorithm 6

  // Obstacle handling
  void UpdateObstacles();       // Algorithm 7
  void PropogateDescendants();  // Algorithm 8
  void AddNewObstacle();
  void RemoveObstacle();

  // Utility functions
  void VerifyQueue(int v_idx);   // Algorithm 12
  void UpdateLMC(int v_idx);     // Algorithm 13
  void VerifyOrphan(int v_idx);  // Algorithm 9
  void MakeParentOf(int parent_idx, int child_idx);

  // Sampling and nearest neighbor
  state::Waypoint RandomNode();
  int Nearest(const state::Waypoint& wp);
  std::vector<int> Near(const state::Waypoint& wp, double r);
  state::Waypoint Saturate(const state::Waypoint& v, const state::Waypoint& v_nearest);

  // Distance and trajectory functions
  double d_pi(const state::Waypoint& a, const state::Waypoint& b);  // trajectory distance
  bool TrajectoryValid(const state::Waypoint& a, const state::Waypoint& b);

  // Tree reconstruction
  state::Path ReconstructPath();

  // Dynamic environment support
  // void InvalidateEdges(const std::vector<state::Waypoint>& obstacles);
  void UpdateRobotPosition(const state::Waypoint& new_pos);

  // Parameter updates
  double ShrinkingBallRadius();
  bool KeyLess(const std::pair<double, double>& key1, const std::pair<double, double>& key2);
  std::pair<double, double> getKey(int v_idx);

  void PlanStep();
  bool SolutionExists();
  double GetSolutionCost();
  bool IsInVertices(state::Waypoint v_new);

  void UpdateGoal(const state::Waypoint& new_goal);

  // Core data structures
  std::vector<Vertex> Vertices;   // vertex set
  std::unordered_set<int> V_c_T;  // orphan nodes V^c_T

  // Priority queue Q with keys (min(g(v), lmc(v)), g(v))
  std::priority_queue<std::pair<std::pair<double, double>, int>,
                      std::vector<std::pair<std::pair<double, double>, int>>, std::greater<>>
      Q;


  // Goal and start vertices
  int v_goal_idx;
  int v_start_idx;
  int v_bot_idx;  // current robot position

  // Algorithm parameters
  double epsilon;  // ε-consistency parameter
  double delta;    // step size for saturate function
  double gamma;    // RRT* shrinking ball parameter
  int n_samples;   // current number of samples

  // Environment
  std::vector<state::Waypoint> obstacles;

  // Helper functions
  bool IsInObstacle(const state::Waypoint& wp);
  bool IsObstaclesChanged();
  bool IsRobotPoseChanged();

  std::vector<int> getNeighbors(int v_idx);     // N(v) = N⁺(v) ∪ N⁻(v)
  std::vector<int> getOutNeighbors(int v_idx);  // N⁺(v) = N₀⁺(v) ∪ Nᵣ⁺(v)
  std::vector<int> getInNeighbors(int v_idx);   // N⁻(v) = N₀⁻(v) ∪ Nᵣ⁻(v)
};

// Convenience function
// state::Path FindSinglePath_RRTX(state::Waypoint x_init, state::Waypoint x_goal);

}  // namespace algos

#endif  // RRTX_H