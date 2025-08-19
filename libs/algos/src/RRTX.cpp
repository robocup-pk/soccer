#include "RRTX.h"
#include "SoccerField.h"
#include "SoccerObject.h"
#include "Kinematics.h"
#include "Grid.h"

#include <random>

namespace {
std::mt19937 rng(std::random_device{}());

// Dynamic distributions based on actual field dimensions
std::uniform_real_distribution<float> GetXDistribution() {
  double field_width = vis::SoccerField::GetInstance().playing_area_width_mm / 1000.0;
  const double margin = 0.1;  // 10cm margin from boundaries
  return std::uniform_real_distribution<float>(-field_width / 2 + margin,
                                               field_width / 2 - margin);
}

std::uniform_real_distribution<float> GetYDistribution() {
  double field_height = vis::SoccerField::GetInstance().playing_area_height_mm / 1000.0;
  const double margin = 0.1;  // 10cm margin from boundaries
  return std::uniform_real_distribution<float>(-field_height / 2 + margin,
                                               field_height / 2 - margin);
}

std::uniform_real_distribution<double> prob_dist(0.0, 1.0);
}  // namespace

algos::RRTX::RRTX(const state::Waypoint& x_start, const state::Waypoint& x_goal, double eps)
    : epsilon(eps), delta(0.5), gamma(1.5), n_samples(2) {
  Vertices.clear();

  // Goal vertex at index 0 - this is the ROOT of shortest path tree
  Vertices.emplace_back();
  Vertices[0].wp = x_goal;
  Vertices[0].g = 0.0;          // Goal has zero cost-to-goal
  Vertices[0].lmc = 0.0;        // Goal is consistent
  Vertices[0].parent_idx = -1;  // Root has no parent
  v_goal_idx = 0;

  // Start vertex at index 1 - initially unreachable
  Vertices.emplace_back();
  Vertices[1].wp = x_start;
  Vertices[1].g = std::numeric_limits<double>::infinity();
  Vertices[1].lmc = std::numeric_limits<double>::infinity();
  Vertices[1].parent_idx = -1;  // No parent initially
  v_start_idx = 1;
  v_bot_idx = v_start_idx;  // Robot starts at start position

  // Clear orphan set
  V_c_T.clear();
  vertices_in_queue.clear();
  robot_pos = x_start;
  spatial_grid = std::make_unique<algos::SpatialGrid>(0.2);
  spatial_grid->AddVertex(v_goal_idx, Vertices[v_goal_idx].wp);
  spatial_grid->AddVertex(v_start_idx, Vertices[v_start_idx].wp);

  free_list.clear();
}

void algos::RRTX::PlanStep() {
  double r = ShrinkingBallRadius();

  // Sample random node
  state::Waypoint v_rand = RandomNode();
  int nearest_idx = Nearest(v_rand);

  // Saturate (limit step size)
  state::Waypoint v_new_wp = Saturate(v_rand, Vertices[nearest_idx].wp);

  int v_new_idx = -1;

  // Check if new node is valid
  if (!IsInObstacle(v_new_wp)) {
    v_new_idx = Extend(v_new_wp, r);
  }

  if (v_new_idx != -1) {
    RewireNeighbors(v_new_idx);
    // Reduce inconsistency
    ReduceInconsistency();
  }
}

int algos::RRTX::Extend(state::Waypoint v_new_wp, double r) {
  std::vector<int> v_near = Near(v_new_wp, r);

  if (IsInVertices(v_new_wp)) {
    return -1;
  }

  Vertex v_new;
  v_new.wp = v_new_wp;
  v_new.g = std::numeric_limits<double>::infinity();
  v_new.lmc = std::numeric_limits<double>::infinity();
  v_new.parent_idx = -1;

  FindParent(v_new, v_near);

  if (v_new.parent_idx == -1) return -1;

  int v_new_idx = AddVertex(v_new.wp);
  Vertices[v_new_idx].lmc = v_new.lmc;
  Vertices[v_new_idx].g = v_new.g;
  Vertices[v_new_idx].parent_idx = v_new.parent_idx;
  Vertices[v_new_idx].wp = v_new.wp;

  Vertices[Vertices[v_new_idx].parent_idx].C_minus_T.insert(v_new_idx);

  for (int u_idx : v_near) {
    if (TrajectoryValid(Vertices[v_new_idx].wp, Vertices[u_idx].wp)) {
      Vertices[v_new_idx].N_plus_0.insert(u_idx);
      Vertices[u_idx].N_minus_r.insert(v_new_idx);
    }
    if (TrajectoryValid(Vertices[u_idx].wp, Vertices[v_new_idx].wp)) {
      Vertices[u_idx].N_plus_r.insert(v_new_idx);
      Vertices[v_new_idx].N_minus_0.insert(u_idx);
    }
  }
  n_samples++;
  return v_new_idx;
}

void algos::RRTX::CullNeighbors(int v_idx, double r) {
  auto& N_plus_r = Vertices[v_idx].N_plus_r;
  std::vector<int> to_remove;
  for (int u_idx : N_plus_r) {
    if (r < d_pi(Vertices[v_idx].wp, Vertices[u_idx].wp) && Vertices[v_idx].parent_idx != u_idx) {
      to_remove.push_back(u_idx);
    }
  }
  for (int u_idx : to_remove) {
    N_plus_r.erase(u_idx);
    Vertices[u_idx].N_minus_r.erase(v_idx);
  }
}

void algos::RRTX::RewireNeighbors(int v_idx) {
  Vertex& v = Vertices[v_idx];

  // Line 1: Check ε-consistency condition
  if (v.g - v.lmc <= epsilon) return;

  // Line 2: Cull neighbors outside shrinking ball radius
  const double r = ShrinkingBallRadius();
  CullNeighbors(v_idx, r);

  // Lines 3-7: Process all in-neighbors except parent
  // Process original in-neighbors (N_minus_0)
  for (int u_idx : v.N_minus_0) {
    if (u_idx != v.parent_idx) {
      const double new_lmc = d_pi(Vertices[u_idx].wp, v.wp) + v.lmc;

      if (Vertices[u_idx].lmc > new_lmc) {
        // Line 5: Update LMC
        Vertices[u_idx].lmc = new_lmc;

        // Line 6: Rewire parent
        MakeParentOf(v_idx, u_idx);

        // Line 7: Verify queue if inconsistent
        if (Vertices[u_idx].g - Vertices[u_idx].lmc > epsilon) {
          VerifyQueue(u_idx);
        }
      }
    }
  }

  // Process running in-neighbors (N_minus_r)
  for (int u_idx : v.N_minus_r) {
    if (u_idx != v.parent_idx) {
      const double edge_cost = d_pi(Vertices[u_idx].wp, v.wp);
      const double new_lmc = edge_cost + v.lmc;

      if (Vertices[u_idx].lmc > new_lmc) {
        // Line 5: Update LMC
        Vertices[u_idx].lmc = new_lmc;

        // Line 6: Rewire parent
        MakeParentOf(v_idx, u_idx);

        // Line 7: Verify queue if inconsistent
        if (Vertices[u_idx].g - Vertices[u_idx].lmc > epsilon) {
          VerifyQueue(u_idx);
        }
      }
    }
  }
}

void algos::RRTX::ReduceInconsistency() {
  while (!Q.empty() && (KeyLess(Q.top().first, getKey(v_bot_idx)) ||
                        Vertices[v_bot_idx].lmc != Vertices[v_bot_idx].g ||
                        Vertices[v_bot_idx].g == std::numeric_limits<double>::infinity() ||
                        vertices_in_queue.count(v_bot_idx))) {
    auto top = Q.top();
    Q.pop();
    int v_idx = top.second;

    vertices_in_queue.erase(v_idx);

    auto current_key = getKey(v_idx);
    if (KeyLess(top.first, current_key) || KeyLess(current_key, top.first)) {
      continue;  // Skip stale entry, process next
    }

    if (Vertices[v_idx].g - Vertices[v_idx].lmc > epsilon) {
      UpdateLMC(v_idx);
      RewireNeighbors(v_idx);
    }

    Vertices[v_idx].g = Vertices[v_idx].lmc;
  }
}

void algos::RRTX::FindParent(Vertex& v_new, const std::vector<int>& U) {
  double r = ShrinkingBallRadius();

  for (int u_idx : U) {
    double dist = d_pi(v_new.wp, Vertices[u_idx].wp);

    // In RRT-X: u becomes parent of v if path v→u→goal is better than current path v→goal
    // We check if going through u gives a better cost-to-goal for v
    if (dist <= r && v_new.lmc > Vertices[u_idx].lmc + dist &&
        TrajectoryValid(v_new.wp, Vertices[u_idx].wp)) {
      v_new.parent_idx = u_idx;
      v_new.lmc = Vertices[u_idx].lmc + dist;
    }
  }
}

void algos::RRTX::UpdateObstacles(std::vector<state::SoccerObject>& new_obstacles) {
  // Compute deltas
  std::vector<state::SoccerObject> vanished = FindVanishedObstacles(new_obstacles);
  std::vector<state::SoccerObject> appeared = FindAppearedObstacles(new_obstacles);
  auto moved_pairs = FindMovedObstacles(new_obstacles);  // pairs of (old,new)

  bool any_change = !vanished.empty() || !appeared.empty() || !moved_pairs.empty();

  // Process moves: first block newly-colliding edges, then free previously-blocked edges
  for (auto& pair : moved_pairs) {
    state::SoccerObject& old_obs = pair.first;
    state::SoccerObject& new_obs = pair.second;
    AddNewObstacle(new_obs);  // remove edges that now collide
    RemoveObstacle(old_obs);  // restore edges that are now free
  }

  // Process full removal/additions
  for (auto& obs : vanished) RemoveObstacle(obs);
  for (auto& obs : appeared) AddNewObstacle(obs);

  // Update state, propagate, reduce inconsistency
  current_obstacles = new_obstacles;

  if (any_change) {
    PropogateDescendants();
    // Drive inconsistency reduction; stop when robot is epsilon-consistent or queue is stable
    ReduceInconsistency();
  }

  ValidateRobotPath();
}

void algos::RRTX::ValidateRobotPath() {
  if (Vertices[v_bot_idx].g < std::numeric_limits<double>::infinity()) {
    // Robot thinks it has a path - validate it
    if (!IsPathToGoalValid()) {
      // Path is invalid - clear it and trigger replanning
      ClearRobotPath();
    }
  }
}

void algos::RRTX::PropogateDescendants() {
  std::unordered_set<int> to_add;
  for (int v_idx : V_c_T) {
    if (!Vertices[v_idx].alive) continue;
    for (int child_idx : Vertices[v_idx].C_minus_T) {
      if (child_idx < 0 || child_idx >= static_cast<int>(Vertices.size()) ||
          !Vertices[child_idx].alive)
        continue;
      to_add.insert(child_idx);
    }
  }
  V_c_T.insert(to_add.begin(), to_add.end());

  for (int v_idx : V_c_T) {
    std::vector<int> neighbors = getOutNeighbors(v_idx);
    if (Vertices[v_idx].parent_idx != -1) neighbors.push_back(Vertices[v_idx].parent_idx);

    for (int u_idx : neighbors) {
      if (V_c_T.count(u_idx) == 0) {
        Vertices[u_idx].g = std::numeric_limits<double>::infinity();
        VerifyQueue(u_idx);
      }
    }
  }

  for (int v_idx : V_c_T) {
    Vertices[v_idx].g = std::numeric_limits<double>::infinity();
    Vertices[v_idx].lmc = std::numeric_limits<double>::infinity();

    if (Vertices[v_idx].parent_idx != -1) {
      Vertices[Vertices[v_idx].parent_idx].C_minus_T.erase(v_idx);
      Vertices[v_idx].parent_idx = -1;
    }
  }

  V_c_T.clear();
}

void algos::RRTX::VerifyOrphan(int v_idx) {
  vertices_in_queue.erase(v_idx);
  V_c_T.insert(v_idx);
}

void algos::RRTX::RemoveObstacle(state::SoccerObject& obstacle) {
  std::set<std::pair<int, int>> EO = GetEdgesIntersectingObstacle(obstacle);

  // Keep only edges not blocked by any remaining obstacle (truly free now)
  std::set<std::pair<int, int>> free_edges;
  for (const auto& edge : EO) {
    int v_idx = edge.first;
    int u_idx = edge.second;

    bool still_blocked = false;
    for (auto& other_obstacle : current_obstacles) {
      if (other_obstacle.name == obstacle.name &&
          (other_obstacle.position - obstacle.position).norm() < 0.05) {
        continue;
      }
      if (IsTrajectoryBlockedByObstacle(Vertices[v_idx].wp, Vertices[u_idx].wp, other_obstacle)) {
        still_blocked = true;
        break;
      }
    }
    if (!still_blocked) free_edges.insert(edge);
  }

  // Re-add freed edges (as running neighbors) and enqueue endpoints
  for (const auto& edge : free_edges) {
    int v_idx = edge.first;
    int u_idx = edge.second;

    // Prune stale blocked edges around endpoints before reconnecting
    PruneInvalidNeighbors(v_idx);
    PruneInvalidNeighbors(u_idx);
    // Reconnect both directions if valid
    if (TrajectoryValid(Vertices[v_idx].wp, Vertices[u_idx].wp)) {
      Vertices[v_idx].N_plus_r.insert(u_idx);
      Vertices[u_idx].N_minus_r.insert(v_idx);
    }
    if (TrajectoryValid(Vertices[u_idx].wp, Vertices[v_idx].wp)) {
      Vertices[u_idx].N_plus_r.insert(v_idx);
      Vertices[v_idx].N_minus_r.insert(u_idx);
    }
    UpdateLMC(v_idx);
    if (std::abs(Vertices[v_idx].lmc - Vertices[v_idx].g) > epsilon) VerifyQueue(v_idx);
    UpdateLMC(u_idx);
    if (std::abs(Vertices[u_idx].lmc - Vertices[u_idx].g) > epsilon) VerifyQueue(u_idx);
  }
}

std::set<int> algos::RRTX::GetVerticesWithEdgesInObstacle(std::set<std::pair<int, int>>& edges) {
  std::set<int> vertices;  // This is VO
  for (const auto& edge : edges) {
    vertices.insert(edge.first);   // Add source vertex
    vertices.insert(edge.second);  // Add destination vertex (both endpoints affected)
  }
  return vertices;
}

std::set<std::pair<int, int>> algos::RRTX::GetEdgesIntersectingObstacle(
    state::SoccerObject& obstacle) {
  std::set<std::pair<int, int>> intersecting_edges;  // This is EO

  // Check all vertices and their connections
  for (size_t i = 0; i < Vertices.size(); ++i) {
    if (!Vertices[i].alive) continue;
    // Check parent edge (tree edge)
    if (Vertices[i].parent_idx != -1) {
      if (IsTrajectoryBlockedByObstacle(Vertices[i].wp, Vertices[Vertices[i].parent_idx].wp,
                                        obstacle)) {
        intersecting_edges.insert({static_cast<int>(i), Vertices[i].parent_idx});
      }
    }

    // Check original outgoing neighbors (N₀⁺)
    for (int neighbor_idx : Vertices[i].N_plus_0) {
      if (IsTrajectoryBlockedByObstacle(Vertices[i].wp, Vertices[neighbor_idx].wp, obstacle)) {
        intersecting_edges.insert({static_cast<int>(i), neighbor_idx});
      }
    }

    // Check running outgoing neighbors (Nᵣ⁺)
    for (int neighbor_idx : Vertices[i].N_plus_r) {
      if (IsTrajectoryBlockedByObstacle(Vertices[i].wp, Vertices[neighbor_idx].wp, obstacle)) {
        intersecting_edges.insert({static_cast<int>(i), neighbor_idx});
      }
    }
  }

  return intersecting_edges;
}

bool algos::RRTX::IsTrajectoryBlockedByObstacle(state::Waypoint& from, state::Waypoint& to,
                                                state::SoccerObject& obstacle) {
  // Calculate path length
  double total_radius = (2 * obstacle.radius_m);  // Robot Radius + Obstacle(i-e Robot) Radius

  // Create obstacle center as waypoint for your function
  state::Waypoint obstacle_center(obstacle.position[0], obstacle.position[1], 0.0);
  double distance = PerpendicularDistanceToLineSegment(obstacle_center, from, to);

  return distance <= total_radius;
}

void algos::RRTX::AddNewObstacle(state::SoccerObject& new_obstacle) {
  // Line 2: EO ← {(v, u) ∈ E : π(v, u) ∩ O ≠ ∅} (Find intersecting edges)
  std::set<std::pair<int, int>> EO = GetEdgesIntersectingObstacle(new_obstacle);

  // Lines 3-6: Process each intersecting edge
  for (const auto& edge : EO) {
    int v_idx = edge.first;
    int u_idx = edge.second;

    // Line 4: dπ(v, u) ← ∞
    RemoveEdgeConnection(v_idx, u_idx);

    // Line 5: if p+T(v) = u then verrifyOrphan(v)
    if (Vertices[v_idx].parent_idx == u_idx) {
      VerifyOrphan(v_idx);
    }

    // Line 6: if vbot ∈ π(v, u) then πbot = ∅
    if (IsRobotOnEdge(v_idx, u_idx)) {
      ClearRobotPath();
    }

    // Prune stale invalid neighbors around the impacted endpoints
    PruneInvalidNeighbors(v_idx);
    PruneInvalidNeighbors(u_idx);
  }
}

double algos::RRTX::PerpendicularDistanceToLineSegment(state::Waypoint& point,
                                                       state::Waypoint& line_start,
                                                       state::Waypoint& line_end) {
  // Vector from line_start to line_end
  double dx = line_end.x - line_start.x;
  double dy = line_end.y - line_start.y;

  // If line segment is essentially a point
  if (dx * dx + dy * dy < 1e-10) {
    return sqrt((point.x - line_start.x) * (point.x - line_start.x) +
                (point.y - line_start.y) * (point.y - line_start.y));
  }

  // Parameter t for closest point on line segment
  double t = ((point.x - line_start.x) * dx + (point.y - line_start.y) * dy) / (dx * dx + dy * dy);

  // Clamp t to [0,1] to stay within line segment
  t = std::max(0.0, std::min(1.0, t));

  // Closest point on line segment
  double closest_x = line_start.x + t * dx;
  double closest_y = line_start.y + t * dy;

  // Distance from point to closest point on segment
  return sqrt((point.x - closest_x) * (point.x - closest_x) +
              (point.y - closest_y) * (point.y - closest_y));
}

inline void eraseNeighbor(std::unordered_set<int>& neighbors, int idx) { neighbors.erase(idx); }

void algos::RRTX::RemoveEdgeConnection(int v_idx, int u_idx) {
  auto& v = Vertices[v_idx];
  auto& u = Vertices[u_idx];

  eraseNeighbor(v.N_plus_0, u_idx);
  eraseNeighbor(v.N_plus_r, u_idx);
  eraseNeighbor(u.N_minus_0, v_idx);
  eraseNeighbor(u.N_minus_r, v_idx);

  bool vChanged = false;
  bool uChanged = false;

  if (v.parent_idx == u_idx) {
    eraseNeighbor(u.C_minus_T, v_idx);
    v.parent_idx = -1;
    v.g = std::numeric_limits<double>::infinity();
    v.lmc = std::numeric_limits<double>::infinity();
    vChanged = true;
  }

  if (u.parent_idx == v_idx) {
    eraseNeighbor(v.C_minus_T, u_idx);
    u.parent_idx = -1;
    u.g = std::numeric_limits<double>::infinity();
    u.lmc = std::numeric_limits<double>::infinity();
    uChanged = true;
  }

  if (vChanged) {
    VerifyOrphan(v_idx);
    VerifyQueue(v_idx);
  }
  if (uChanged) {
    VerifyOrphan(u_idx);
    VerifyQueue(u_idx);
  }
}

bool algos::RRTX::IsRobotOnEdge(int v_idx, int u_idx) {
  auto& robot_pos = Vertices[v_bot_idx].wp;
  auto& v_pos = Vertices[v_idx].wp;
  auto& u_pos = Vertices[u_idx].wp;

  double distance = PerpendicularDistanceToLineSegment(robot_pos, v_pos, u_pos);
  double robot_radius = cfg::SystemConfig::robot_size_m[0] / 2.0;  // Assuming square robot

  return distance <= robot_radius;  // Robot body intersects edge
}

void algos::RRTX::ClearRobotPath() {
  // Mark robot as needing replanning
  Vertices[v_bot_idx].g = std::numeric_limits<double>::infinity();
  Vertices[v_bot_idx].lmc = std::numeric_limits<double>::infinity();
  Vertices[v_bot_idx].parent_idx = -1;

  // Add to verification queue
  VerifyQueue(v_bot_idx);
}

void algos::RRTX::UpdateLMC(int v_idx) {
  Vertex& v = Vertices[v_idx];
  if (v_idx == v_goal_idx) {
    v.lmc = 0;
    return;
  }

  // Remove any neighbors that have become invalid due to obstacle changes
  PruneInvalidNeighbors(v_idx);

  const double r = ShrinkingBallRadius();
  CullNeighbors(v_idx, r);

  v.lmc = std::numeric_limits<double>::infinity();
  int best_parent_idx = -1;
  double best_cost = v.lmc;

  // Original neighbors
  std::vector<int> to_prune_0;
  for (int u_idx : v.N_plus_0) {
    if (V_c_T.count(u_idx) || Vertices[u_idx].parent_idx == v_idx) continue;
    if (!TrajectoryValid(v.wp, Vertices[u_idx].wp)) {
      to_prune_0.push_back(u_idx);
      continue;
    }
    const double cost = d_pi(v.wp, Vertices[u_idx].wp) + Vertices[u_idx].lmc;
    if (cost < best_cost) {
      best_cost = cost;
      best_parent_idx = u_idx;
    }
  }
  for (int u_idx : to_prune_0) {
    v.N_plus_0.erase(u_idx);
    Vertices[u_idx].N_minus_0.erase(v_idx);
  }

  // Running neighbors
  std::vector<int> to_prune_r;
  for (int u_idx : v.N_plus_r) {
    if (V_c_T.count(u_idx) || Vertices[u_idx].parent_idx == v_idx) continue;
    if (!TrajectoryValid(v.wp, Vertices[u_idx].wp)) {
      to_prune_r.push_back(u_idx);
      continue;
    }
    const double cost = d_pi(v.wp, Vertices[u_idx].wp) + Vertices[u_idx].lmc;
    if (cost < best_cost) {
      best_cost = cost;
      best_parent_idx = u_idx;
    }
  }
  for (int u_idx : to_prune_r) {
    v.N_plus_r.erase(u_idx);
    Vertices[u_idx].N_minus_r.erase(v_idx);
  }

  if (best_parent_idx != -1) {
    v.lmc = best_cost;
    MakeParentOf(best_parent_idx, v_idx);
  }
}

bool algos::RRTX::KeyLess(const std::pair<double, double>& key1,
                          const std::pair<double, double>& key2) {
  return (key1.first < key2.first) || (key1.first == key2.first && key1.second < key2.second);
}

void algos::RRTX::VerifyQueue(int v_idx) {
  if (Q.size() > 200) {
    CleanupQueue();
  }

  if (v_idx < 0 || v_idx >= static_cast<int>(Vertices.size()) || !Vertices[v_idx].alive) return;
  if (!vertices_in_queue.count(v_idx)) {
    vertices_in_queue.insert(v_idx);
    Q.push({getKey(v_idx), v_idx});
  }
}

void algos::RRTX::CleanupQueue() {
  std::vector<std::pair<std::pair<double, double>, int>> valid_entries;
  vertices_in_queue.clear();

  // Extract all entries and check if they're still relevant
  while (!Q.empty()) {
    auto entry = Q.top();
    Q.pop();
    int v_idx = entry.second;
    if (v_idx < 0 || v_idx >= static_cast<int>(Vertices.size()) || !Vertices[v_idx].alive) {
      continue;  // drop dead/stale
    }
    auto current_key = getKey(v_idx);
    if (!KeyLess(entry.first, current_key) && !KeyLess(current_key, entry.first)) {
      valid_entries.push_back(entry);
      vertices_in_queue.insert(v_idx);
    }
  }

  // Rebuild queue with only valid entries
  for (const auto& entry : valid_entries) {
    Q.push(entry);
  }
}

std::pair<double, double> algos::RRTX::getKey(int v_idx) {
  return {std::min(Vertices[v_idx].g, Vertices[v_idx].lmc), Vertices[v_idx].g};
}

void algos::RRTX::MakeParentOf(int parent_idx, int child_idx) {
  for (int idx = parent_idx; idx != -1; idx = Vertices[idx].parent_idx) {
    if (idx == child_idx) return;  // Cycle
  }

  Vertex& child = Vertices[child_idx];

  if (child.parent_idx != -1) {
    Vertices[child.parent_idx].C_minus_T.erase(child_idx);
  }

  // Update parent
  child.parent_idx = parent_idx;
  Vertices[parent_idx].C_minus_T.insert(child_idx);
}

std::vector<int> algos::RRTX::getInNeighbors(int v_idx) {
  std::vector<int> neighbors;
  neighbors.reserve(Vertices[v_idx].N_minus_0.size() + Vertices[v_idx].N_minus_r.size());
  for (int u : Vertices[v_idx].N_minus_0) {
    if (u >= 0 && u < static_cast<int>(Vertices.size()) && Vertices[u].alive)
      neighbors.push_back(u);
  }
  for (int u : Vertices[v_idx].N_minus_r) {
    if (u >= 0 && u < static_cast<int>(Vertices.size()) && Vertices[u].alive)
      neighbors.push_back(u);
  }
  return neighbors;
}

std::vector<int> algos::RRTX::getOutNeighbors(int v_idx) {
  std::vector<int> neighbors;
  neighbors.reserve(Vertices[v_idx].N_plus_0.size() + Vertices[v_idx].N_plus_r.size());
  for (int u : Vertices[v_idx].N_plus_0) {
    if (u >= 0 && u < static_cast<int>(Vertices.size()) && Vertices[u].alive)
      neighbors.push_back(u);
  }
  for (int u : Vertices[v_idx].N_plus_r) {
    if (u >= 0 && u < static_cast<int>(Vertices.size()) && Vertices[u].alive)
      neighbors.push_back(u);
  }
  return neighbors;
}

void algos::RRTX::PruneInvalidNeighbors(int v_idx) {
  if (v_idx < 0 || v_idx >= static_cast<int>(Vertices.size()) || !Vertices[v_idx].alive) return;
  auto& v = Vertices[v_idx];
  std::vector<int> rm0;
  std::vector<int> rmr;
  for (int u : v.N_plus_0) {
    if (u < 0 || u >= static_cast<int>(Vertices.size()) || !Vertices[u].alive) {
      rm0.push_back(u);
      continue;
    }
    if (!TrajectoryValid(v.wp, Vertices[u].wp)) rm0.push_back(u);
  }
  for (int u : v.N_plus_r) {
    if (u < 0 || u >= static_cast<int>(Vertices.size()) || !Vertices[u].alive) {
      rmr.push_back(u);
      continue;
    }
    if (!TrajectoryValid(v.wp, Vertices[u].wp)) rmr.push_back(u);
  }
  for (int u : rm0) {
    v.N_plus_0.erase(u);
    if (u >= 0 && u < static_cast<int>(Vertices.size())) Vertices[u].N_minus_0.erase(v_idx);
  }
  for (int u : rmr) {
    v.N_plus_r.erase(u);
    if (u >= 0 && u < static_cast<int>(Vertices.size())) Vertices[u].N_minus_r.erase(v_idx);
  }
}

bool algos::RRTX::TrajectoryValid(state::Waypoint& a, state::Waypoint& b) {
  // Check endpoints first
  if (IsInObstacle(a) || IsInObstacle(b)) {
    return false;
  }

  // Check for collisions with all current obstacles along the path
  for (auto& obstacle : current_obstacles) {
    if (IsTrajectoryBlockedByObstacle(a, b, obstacle)) {
      return false;
    }
  }

  return true;
}

bool algos::RRTX::IsInObstacle(const state::Waypoint& wp) {
  double field_width = vis::SoccerField::GetInstance().playing_area_width_mm / 1000.0;
  double field_height = vis::SoccerField::GetInstance().playing_area_height_mm / 1000.0;
  const double boundary_margin = 0.05;

  bool out_of_bounds =
      (wp.x < (-field_width / 2 + boundary_margin) || wp.x > (field_width / 2 - boundary_margin) ||
       wp.y < (-field_height / 2 + boundary_margin) ||
       wp.y > (field_height / 2 - boundary_margin));

  if (out_of_bounds) return true;

  state::SoccerObject temp_robot("temp_check", Eigen::Vector3d(wp.x, wp.y, wp.angle),
                                 cfg::SystemConfig::robot_size_m,  // Use robot size
                                 Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 1.0);
  for (auto& obstacle : current_obstacles) {
    if (kin::CheckCircularCollision(temp_robot, obstacle)) {
      return true;
    }
  }

  return false;
}

double algos::RRTX::ShrinkingBallRadius() {
  double x_range = 2.03784f * 2;    // 4.07568
  double y_range = 1.45f * 2;       // 2.9
  double area = x_range * y_range;  // state space volume
  return gamma * std::pow(std::log(n_samples + 1) / (n_samples + 1), 1.0 / 2.0) * std::sqrt(area);
}

std::vector<int> algos::RRTX::Near(state::Waypoint& wp, double r) {
  return spatial_grid->FindNear(wp, r, Vertices);
}

int algos::RRTX::Nearest(state::Waypoint& wp) { return spatial_grid->FindNearest(wp, Vertices); }

double algos::RRTX::d_pi(const state::Waypoint& a, const state::Waypoint& b) {
  return (a - b).Norm();
}

state::Waypoint algos::RRTX::Saturate(const state::Waypoint& v, const state::Waypoint& v_nearest) {
  state::Waypoint diff = v - v_nearest;
  double dist = diff.Norm();

  if (dist > delta) {
    diff = diff * (delta / dist);
    return v_nearest + diff;
  }

  return v;
}

state::Waypoint algos::RRTX::RandomNode() {
  if (prob_dist(rng) < 0.2) {
    return Vertices[v_goal_idx].wp;
  }

  auto x_dist = GetXDistribution();
  auto y_dist = GetYDistribution();

  state::Waypoint random_node;
  int max_attempts = 100;

  for (int attempt = 0; attempt < max_attempts; ++attempt) {
    random_node.x = x_dist(rng);
    random_node.y = y_dist(rng);
    random_node.angle = 0.0;

    if (!IsInObstacle(random_node)) {
      return random_node;
    }
  }

  return Vertices[v_goal_idx].wp;
}

state::Path algos::RRTX::ReconstructPath() {
  state::Path path;

  // Check if robot has finite cost-to-goal (solution exists)
  if (Vertices[v_bot_idx].g >= std::numeric_limits<double>::infinity()) {
    return path;  // No path exists
  }

  // In goal-rooted tree: follow parent pointers FROM robot TO goal
  std::vector<state::Waypoint> waypoints;
  int current_idx = v_bot_idx;

  // Add robot's current position as starting point
  waypoints.push_back(robot_pos);

  // Follow parent pointers from robot towards goal
  while (current_idx != -1 && current_idx != v_goal_idx) {
    int parent_idx = Vertices[current_idx].parent_idx;

    if (parent_idx != -1) {
      // Validate edge from current to parent
      if (!TrajectoryValid(Vertices[current_idx].wp, Vertices[parent_idx].wp)) {
        // Path is invalid, return empty path to trigger replanning
        return state::Path();
      }

      // Add parent waypoint
      waypoints.push_back(Vertices[parent_idx].wp);
    }

    current_idx = parent_idx;
  }

  // Check if we successfully reached the goal
  if (current_idx == v_goal_idx) {
    // Convert waypoints to path
    for (const auto& wp : waypoints) {
      path.push_back(wp);
    }
  }

  return path;
}

bool algos::RRTX::IsPathValid(state::Path& path) {
  if (path.empty()) return false;  // No path to validate

  // Check if path starts from robot pose
  if ((path[0] - robot_pos).Norm() > 0.05) {
    return false;
  }

  // Check if path ends at goal
  if ((path.back() - Vertices[v_goal_idx].wp).Norm() > 0.05) {
    return false;
  }

  // Check path continuity (consecutive waypoints should be reasonably close)
  const double MAX_SEGMENT_LENGTH = delta * 2.0;  // Based on step size limit
  for (size_t i = 0; i < path.size() - 1; ++i) {
    double segment_length = (path[i + 1] - path[i]).Norm();
    if (segment_length > MAX_SEGMENT_LENGTH) {
      return false;
    }
  }

  return true;  // All segments are valid
}

bool algos::RRTX::SolutionExists() {
  return Vertices[v_bot_idx].g < std::numeric_limits<double>::infinity();
}

bool algos::RRTX::IsPathToGoalValid() {
  int current_idx = v_bot_idx;

  // Trace path and validate key edges
  while (current_idx != -1 && current_idx != v_goal_idx) {
    int parent_idx = Vertices[current_idx].parent_idx;

    if (parent_idx != -1) {
      // Quick trajectory validity check
      if (!TrajectoryValid(Vertices[current_idx].wp, Vertices[parent_idx].wp)) {
        return false;
      }
    }

    current_idx = parent_idx;
  }

  return current_idx == v_goal_idx;  // Reached goal successfully
}

double algos::RRTX::GetSolutionCost() {
  // Return cost from current robot position to goal
  return Vertices[v_bot_idx].g;
}

void algos::RRTX::UpdateRobotPosition(state::Waypoint& new_pos) {
  double movement = (robot_pos - new_pos).Norm();
  const double ROBOT_MOVEMENT_THRESHOLD = 0.05;

  if (movement < ROBOT_MOVEMENT_THRESHOLD) {
    return;
  }

  // Store old position for spatial grid update
  state::Waypoint old_pos = robot_pos;
  robot_pos = new_pos;

  // Option 1: Update existing robot vertex position
  if (v_bot_idx >= 0 && v_bot_idx < Vertices.size() && Vertices[v_bot_idx].alive) {
    // Update spatial grid
    spatial_grid->UpdateVertex(v_bot_idx, old_pos, new_pos);

    // Update vertex position
    Vertices[v_bot_idx].wp = new_pos;

    // Keep neighbor sets consistent around the robot
    PruneInvalidNeighbors(v_bot_idx);

    // Recalculate LMC and trigger replanning if needed
    UpdateLMC(v_bot_idx);
    if (Vertices[v_bot_idx].g - Vertices[v_bot_idx].lmc > epsilon) {
      VerifyQueue(v_bot_idx);
    }
  } else {
    // Option 2: Find nearest vertex and update v_bot_idx
    int nearest_idx = Nearest(new_pos);
    if (nearest_idx != -1) {
      v_bot_idx = nearest_idx;
      // The robot position and vertex position will be slightly different
      // This is acceptable as robot_pos represents actual position
    }
  }
}

bool algos::RRTX::HasObstaclesChanged(const std::vector<state::SoccerObject>& new_obstacles) {
  // Compare new obstacles with current obstacles (not previous!)
  if (new_obstacles.size() != current_obstacles.size()) {
    return true;  // Different number of obstacles
  }

  const double MOVEMENT_THRESHOLD = 0.05;  // 5cm threshold

  for (unsigned int i = 0; i < new_obstacles.size(); i++) {
    if ((new_obstacles[i].position - current_obstacles[i].position).norm() > MOVEMENT_THRESHOLD) {
      return true;  // Found movement > threshold
    }
  }

  return false;  // No significant changes
}

bool algos::RRTX::IsRobotPoseChanged() { return false; }

bool algos::RRTX::IsInVertices(state::Waypoint v_new_wp) {
  const double EPSILON = 1e-6;  // Small threshold for position equality
  std::vector<int> nearby_indices = spatial_grid->FindNear(v_new_wp, EPSILON, Vertices);

  // Check if any nearby vertex has exactly the same waypoint
  for (int idx : nearby_indices) {
    if (Vertices[idx].alive && Vertices[idx].wp == v_new_wp) {
      return true;
    }
  }
  return false;
}

std::vector<std::pair<state::SoccerObject, state::SoccerObject>> algos::RRTX::FindMovedObstacles(
    std::vector<state::SoccerObject>& new_obstacles) {
  std::vector<std::pair<state::SoccerObject, state::SoccerObject>> moved;

  const double MOVEMENT_THRESHOLD = 0.05;  // 5cm threshold

  for (auto& new_obs : new_obstacles) {
    for (auto& current_obs : current_obstacles) {
      if (new_obs.name == current_obs.name) {
        double distance = (new_obs.position - current_obs.position).norm();
        if (distance > MOVEMENT_THRESHOLD) {
          // Same obstacle, but moved significantly
          moved.push_back({current_obs, new_obs});
        }
        break;  // Found matching obstacle
      }
    }
  }

  return moved;
}

std::vector<state::SoccerObject> algos::RRTX::FindVanishedObstacles(
    std::vector<state::SoccerObject>& new_obstacles) {
  std::vector<state::SoccerObject> vanished;

  // Check each current obstacle to see if it's missing in new_obstacles
  for (auto& current_obs : current_obstacles) {
    bool found = false;
    for (auto& new_obs : new_obstacles) {
      if (ObstaclesEqual(current_obs, new_obs)) {
        found = true;
        break;
      }
    }
    if (!found) {
      vanished.push_back(current_obs);
    }
  }
  return vanished;
}

std::vector<state::SoccerObject> algos::RRTX::FindAppearedObstacles(
    std::vector<state::SoccerObject>& new_obstacles) {
  std::vector<state::SoccerObject> appeared;

  // Check each new obstacle to see if it's missing in current_obstacles
  for (auto& new_obs : new_obstacles) {
    bool found = false;
    for (auto& current_obs : current_obstacles) {
      if (ObstaclesEqual(current_obs, new_obs)) {
        found = true;
        break;
      }
    }
    if (!found) {
      appeared.push_back(new_obs);
    }
  }
  return appeared;
}

bool algos::RRTX::ObstaclesEqual(state::SoccerObject& obs1, state::SoccerObject& obs2) {
  // Use name comparison or position threshold
  return (obs1.name == obs2.name) &&
         ((obs1.position - obs2.position).norm() < 0.01);  // 1cm threshold
}

int algos::RRTX::AddVertex(state::Waypoint& wp) {
  int idx;
  if (!free_list.empty()) {
    idx = free_list.back();
    free_list.pop_back();
    Vertex& v = Vertices[idx];
    v.wp = wp;
    v.parent_idx = -1;
    v.alive = true;
    v.N_plus_0.clear();
    v.N_plus_r.clear();
    v.N_minus_0.clear();
    v.N_minus_r.clear();
    v.C_minus_T.clear();
    v.g = std::numeric_limits<double>::infinity();
    v.lmc = std::numeric_limits<double>::infinity();
  } else {
    Vertex v;
    v.wp = wp;
    Vertices.push_back(std::move(v));
    idx = static_cast<int>(Vertices.size()) - 1;
  }
  spatial_grid->AddVertex(idx, wp);

  return idx;
}

void algos::RRTX::RemoveVertex(int idx) {
  if (idx < 0 || idx >= static_cast<int>(Vertices.size())) return;
  Vertex& v = Vertices[idx];
  if (!v.alive) return;

  spatial_grid->RemoveVertex(idx, v.wp);

  v.alive = false;

  // Remove this idx from any neighbor/children sets of neighbours.
  for (int u : v.N_plus_0) {
    Vertices[u].N_minus_0.erase(idx);
  }
  for (int u : v.N_plus_r) {
    Vertices[u].N_minus_r.erase(idx);
  }
  for (int u : v.N_minus_0) {
    Vertices[u].N_plus_0.erase(idx);
  }
  for (int u : v.N_minus_r) {
    Vertices[u].N_plus_r.erase(idx);
  }

  // Remove parent-child relation if any
  if (v.parent_idx != -1) {
    Vertices[v.parent_idx].C_minus_T.erase(idx);
    v.parent_idx = -1;
  }
  // Remove children links to avoid dangling child entries
  for (int child : v.C_minus_T) {
    Vertices[child].parent_idx = -1;
    // mark child inconsistent; they'll be handled by VerifyQueue/PropagateDescendants
    Vertices[child].g = std::numeric_limits<double>::infinity();
    Vertices[child].lmc = std::numeric_limits<double>::infinity();
    VerifyQueue(child);
    VerifyOrphan(child);
  }
  v.N_plus_0.clear();
  v.N_plus_r.clear();
  v.N_minus_0.clear();
  v.N_minus_r.clear();
  v.C_minus_T.clear();

  free_list.push_back(idx);
}