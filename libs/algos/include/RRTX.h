#ifndef RRTX_H
#define RRTX_H

#include <vector>
#include <Eigen/Dense>
#include <queue>
#include <unordered_set>
#include <set>
#include <random>
#include <algorithm>

#include "Waypoint.h"
#include "SoccerObject.h"
#include "Grid.h"
#include "PQueue.h"

namespace algos {

struct Vertex {
  state::Waypoint wp;                 // v ∈ X
  int parent_idx;                     // p_T⁺(v) — parent in shortest-path tree T
  double g;                           // g(v) — ε-consistent cost-to-goal
  double lmc;                         // lmc(v) — look-ahead estimate of cost-to-goal
  bool alive;                         // true if vertex is alive, false if it has been removed
  std::unordered_set<int> N_plus_0;   // N₀⁺(v) — original outgoing neighbors
  std::unordered_set<int> N_plus_r;   // Nᵣ⁺(v) — running outgoing neighbors
  std::unordered_set<int> N_minus_0;  // N₀⁻(v) — original incoming neighbors
  std::unordered_set<int> N_minus_r;  // Nᵣ⁻(v) — running incoming neighbors
  std::unordered_set<int> C_minus_T;  // C_T⁻(v) — children in tree T

  Vertex()
      : parent_idx(-1),
        g(std::numeric_limits<double>::infinity()),
        lmc(std::numeric_limits<double>::infinity()),
        alive(true) {}
};

class RRTX {
 public:
  RRTX(const state::Waypoint& x_start, const state::Waypoint& x_goal);
  void PlanStep();  // Main planning step

  // Core functions
  int Extend(state::Waypoint v_new, double r);
  void CullNeighbors(int v_idx, double r);
  void RewireNeighbors(int v_idx);
  void ReduceInconsistency();
  void FindParent(Vertex& v_new, const std::vector<int>& U);

  // Obstacle handling
  void UpdateObstacles(std::vector<state::SoccerObject>& robots);
  void PropogateDescendants();
  void AddNewObstacle(state::SoccerObject& obstacle);
  void RemoveObstacle(state::SoccerObject& obstacle);

  // Utility functions
  void VerifyQueue(int v_idx);
  void UpdateLMC(int v_idx);
  void VerifyOrphan(int v_idx);
  void MakeParentOf(int parent_idx, int child_idx);

  // Sampling and nearest neighbor
  state::Waypoint RandomNode();
  int Nearest(state::Waypoint& wp);
  std::vector<int> Near(state::Waypoint& wp, double r);
  state::Waypoint Saturate(const state::Waypoint& v, const state::Waypoint& v_nearest);

  // Distance and trajectory functions
  double d_pi(const state::Waypoint& a, const state::Waypoint& b);  // trajectory distance
  bool TrajectoryValid(state::Waypoint& a, state::Waypoint& b);

  // Tree reconstruction
  state::Path ReconstructPath();

  // Dynamic environment support
  void UpdateRobotPosition(state::Waypoint& new_pos);

  // Parameter updates
  double ShrinkingBallRadius();
  bool KeyLess(const std::pair<double, double>& key1, const std::pair<double, double>& key2);
  std::pair<double, double> getKey(int v_idx);

  // Helper Functions
  std::set<std::pair<int, int>> GetEdgesIntersectingObstacle(state::SoccerObject& obstacle);
  bool IsTrajectoryBlockedByObstacle(state::Waypoint& from, state::Waypoint& to,
                                     state::SoccerObject& obstacle);
  bool SolutionExists();
  bool IsPathToGoalValid();
  bool IsInVertices(state::Waypoint v_new);
  std::vector<state::SoccerObject> FindVanishedObstacles(
      std::vector<state::SoccerObject>& new_obstacles);
  std::vector<state::SoccerObject> FindAppearedObstacles(
      std::vector<state::SoccerObject>& new_obstacles);
  double PerpendicularDistanceToLineSegment(state::Waypoint& point, state::Waypoint& line_start,
                                            state::Waypoint& line_end);
  void RemoveEdgeConnection(int v_idx, int u_idx);
  bool IsRobotOnEdge(int v_idx, int u_idx);
  void ClearRobotPath();
  bool ObstaclesEqual(state::SoccerObject& obs1, state::SoccerObject& obs2);
  bool IsInObstacle(state::Waypoint &wp);
  bool HasObstaclesChanged(const std::vector<state::SoccerObject>& current_obstacles);
  void ValidateRobotPath();
  std::vector<int> getOutNeighbors(int v_idx);  // N⁺(v) = N₀⁺(v) ∪ Nᵣ⁺(v)
  void PruneInvalidNeighbors(int v_idx);
  bool IsPathValid(state::Path& path);

   // Optimizations
  int AddVertex(state::Waypoint& wp);
  void RemoveVertex(int v_idx);
  void LimitTreeSize();

  // Core data structures
  std::vector<Vertex> Vertices;  // vertex set
  std::unique_ptr<SpatialGrid> spatial_grid;
  std::vector<int> free_list;     // list of free vertex indices
  std::unordered_set<int> V_c_T;  // orphan nodes V^c_T

  // Priority queue Q with keys (min(g(v), lmc(v)), g(v))
  algos::PriorityQueue Q;

  // Goal and start vertices
  int v_goal_idx;
  int v_start_idx;
  int v_bot_idx;  // current robot position

  // Algorithm parameters
  double epsilon;  // ε-consistency parameter
  double delta;    // step size for saturate function
  double gamma;    // RRT* shrinking ball parameter
  int n_samples;   // current number of samples
  double r;        // shrinking ball radius

  // Environment
  std::vector<state::SoccerObject> current_obstacles;

  int current_robot_id;
  state::Waypoint robot_pos;
};
}  // namespace algos

#endif  // RRTX_H