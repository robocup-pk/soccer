#include "RRTX.h"
#include "SoccerField.h"
#include "SoccerObject.h"
#include "Kinematics.h"

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
    : epsilon(eps), delta(0.5), gamma(0.5), n_samples(2) {
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
}

void algos::RRTX::PlanStep(std::vector<state::SoccerObject>& soccer_objects) {
  double r = ShrinkingBallRadius();

  if (HasObstaclesChanged(soccer_objects)) {
    UpdateObstacles(soccer_objects);
  }

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

  Vertex v_new;
  v_new.wp = v_new_wp;
  v_new.g = std::numeric_limits<double>::infinity();
  v_new.lmc = std::numeric_limits<double>::infinity();
  v_new.parent_idx = -1;

  FindParent(v_new, v_near);

  if (v_new.parent_idx == -1) return -1;

  int v_new_idx = Vertices.size();
  Vertices.push_back(v_new);

  Vertices[Vertices[v_new_idx].parent_idx].C_minus_T.push_back(v_new_idx);

  for (int u_idx : v_near) {
    if (TrajectoryValid(Vertices[v_new_idx].wp, Vertices[u_idx].wp)) {
      Vertices[v_new_idx].N_plus_0.push_back(u_idx);
      Vertices[u_idx].N_minus_r.push_back(v_new_idx);
    }
    if (TrajectoryValid(Vertices[u_idx].wp, Vertices[v_new_idx].wp)) {
      Vertices[u_idx].N_plus_r.push_back(v_new_idx);
      Vertices[v_new_idx].N_minus_0.push_back(u_idx);
    }
  }
  n_samples++;
  return v_new_idx;
}

void algos::RRTX::CullNeighbors(int v_idx, double r) {
  auto& N_plus_r = Vertices[v_idx].N_plus_r;
  for (auto it = N_plus_r.begin(); it != N_plus_r.end();) {
    int u_idx = *it;
    if (r < d_pi(Vertices[v_idx].wp, Vertices[u_idx].wp) && Vertices[v_idx].parent_idx != u_idx) {
      it = N_plus_r.erase(it);
      auto& N_minus_r_u = Vertices[u_idx].N_minus_r;
      N_minus_r_u.erase(std::remove(N_minus_r_u.begin(), N_minus_r_u.end(), v_idx),
                        N_minus_r_u.end());
    } else {
      ++it;
    }
  }
}

void algos::RRTX::RewireNeighbors(int v_idx) {
  if (Vertices[v_idx].g - Vertices[v_idx].lmc > epsilon) {
    double r = ShrinkingBallRadius();
    CullNeighbors(v_idx, r);

    std::vector<int> in_neighbors = getInNeighbors(v_idx);
    for (int u_idx : in_neighbors) {
      if (u_idx != Vertices[v_idx].parent_idx) {
        if (Vertices[u_idx].lmc >
            d_pi(Vertices[v_idx].wp, Vertices[u_idx].wp) + Vertices[v_idx].lmc) {
          Vertices[u_idx].lmc = d_pi(Vertices[v_idx].wp, Vertices[u_idx].wp) + Vertices[v_idx].lmc;

          MakeParentOf(v_idx, u_idx);

          if (Vertices[u_idx].g - Vertices[u_idx].lmc > epsilon) {
            VerifyQueue(u_idx);
          }
        }
      }
    }
  }
}

void algos::RRTX::ReduceInconsistency() {
  while (!Q.empty() && (KeyLess(Q.top().first, getKey(v_bot_idx)) ||
                        Vertices[v_bot_idx].lmc != Vertices[v_bot_idx].g ||
                        Vertices[v_bot_idx].g == std::numeric_limits<double>::infinity())) {
    auto top = Q.top();
    Q.pop();
    int v_idx = top.second;

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
  previous_obstacles = current_obstacles;

  // Detect vanished and appeared obstacles
  std::vector<state::SoccerObject> vanished_obstacles = FindVanishedObstacles(new_obstacles);
  std::vector<state::SoccerObject> appeared_obstacles = FindAppearedObstacles(new_obstacles);

  // Lines 1-4: Handle vanished obstacles
  if (!vanished_obstacles.empty()) {             // Line 1: if ∃O : O ∈ O ∧ O has vanished
    for (auto& obstacle : vanished_obstacles) {  // Line 2: forall O : O ∈ O ∧ O has vanished do
      RemoveObstacle(obstacle);                  // Line 3: removeObstacle(O)
    }
    ReduceInconsistency();  // Line 4: reduceInconsistency()
  }

  // Lines 5-10: Handle appeared obstacles
  if (!appeared_obstacles.empty()) {             // Line 5: if ∃O : O ∉ O ∧ O has appeared
    for (auto& obstacle : appeared_obstacles) {  // Line 6: forall O : O ∉ O ∧ O has appeared do
      AddNewObstacle(obstacle);                  // Line 7: addNewObstacle(O)
    }
    PropogateDescendants();  // Line 8: propogateDescendants()
    VerifyQueue(v_bot_idx);  // Line 9: verrifyQueue(vbot)
    ReduceInconsistency();   // Line 10: reduceInconsistency()
  }

  // Update obstacle tracking
  current_obstacles = new_obstacles;
}

void algos::RRTX::PropogateDescendants() {
  std::unordered_set<int> to_add;
  for (int v_idx : V_c_T) {
    for (int child_idx : Vertices[v_idx].C_minus_T) {
      to_add.insert(child_idx);
    }
  }
  V_c_T.insert(to_add.begin(), to_add.end());

  for (int v_idx : V_c_T) {
    std::vector<int> neighbors = getOutNeighbors(v_idx);
    if (Vertices[v_idx].parent_idx != -1) neighbors.push_back(Vertices[v_idx].parent_idx);

    for (int u_idx : neighbors) {
      if (V_c_T.find(u_idx) == V_c_T.end()) {
        Vertices[u_idx].g = std::numeric_limits<double>::infinity();
        VerifyQueue(u_idx);
      }
    }
  }

  for (int v_idx : V_c_T) {
    Vertices[v_idx].g = std::numeric_limits<double>::infinity();
    Vertices[v_idx].lmc = std::numeric_limits<double>::infinity();

    if (Vertices[v_idx].parent_idx != -1) {
      auto& parent_children = Vertices[Vertices[v_idx].parent_idx].C_minus_T;
      parent_children.erase(std::remove(parent_children.begin(), parent_children.end(), v_idx),
                            parent_children.end());
      Vertices[v_idx].parent_idx = -1;
    }
  }

  V_c_T.clear();
}

void algos::RRTX::VerifyOrphan(int v_idx) {
  std::priority_queue<std::pair<std::pair<double, double>, int>,
                      std::vector<std::pair<std::pair<double, double>, int>>, std::greater<>>
      newQ;

  while (!Q.empty()) {
    auto entry = Q.top();
    Q.pop();
    if (entry.second != v_idx) {
      newQ.push(entry);
    }
  }

  Q = std::move(newQ);
  V_c_T.insert(v_idx);
}

void algos::RRTX::RemoveObstacle(state::SoccerObject& obstacle) {
  // Line 1: Find edges intersecting with this obstacle
  std::set<std::pair<int, int>> EO = GetEdgesIntersectingObstacle(obstacle);

  // Line 3: Remove edges that still intersect with other obstacles
  std::set<std::pair<int, int>> free_edges;
  for (const auto& edge : EO) {
    int v_idx = edge.first;
    int u_idx = edge.second;

    bool still_blocked = false;
    // Check against all remaining obstacles
    for (auto& other_obstacle : current_obstacles) {
      if (other_obstacle.name == obstacle.name &&
          (other_obstacle.position - obstacle.position).norm() < 0.05) {
        continue;  // Don't check against the obstacle being removed
      }

      if (IsTrajectoryBlockedByObstacle(Vertices[v_idx].wp, Vertices[u_idx].wp, other_obstacle)) {
        still_blocked = true;
        break;
      }
    }

    if (!still_blocked) {
      free_edges.insert(edge);
    }
  }

  // Update EO to contain only truly free edges
  EO = free_edges;

  std::set<int> VO = GetVerticesWithEdgesInObstacle(EO);

  for (int v_idx : VO) {  // Line 5: forall v ∈ VO do
    // Line 8: updateLMC(v)
    UpdateLMC(v_idx);

    // Line 9: if lmc(v) ≠ g(v) then verifyQueue(v)
    if (std::abs(Vertices[v_idx].lmc - Vertices[v_idx].g) > epsilon) {
      VerifyQueue(v_idx);
    }
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

bool algos::RRTX::IsTrajectoryBlockedByObstacle(const state::Waypoint& from,
                                                const state::Waypoint& to,
                                                state::SoccerObject& obstacle) {
  // Calculate path length
  double dx = to.x - from.x;
  double dy = to.y - from.y;
  double path_length = sqrt(dx * dx + dy * dy);

  // Calculate number of samples based on robot radius
  int num_samples = static_cast<int>(ceil(path_length / obstacle.radius_m)) + 1;

  // Ensure minimum sampling
  num_samples = std::max(num_samples, 3);  // At least start, middle, end

  for (int i = 0; i <= num_samples; ++i) {
    double t = static_cast<double>(i) / num_samples;
    state::Waypoint sample_point;
    sample_point.x = from.x + t * dx;
    sample_point.y = from.y + t * dy;
    sample_point.angle = from.angle + t * (to.angle - from.angle);

    // Create temp robot for collision check
    state::SoccerObject temp_robot(
        "temp_traj", Eigen::Vector3d(sample_point.x, sample_point.y, sample_point.angle),
        Eigen::Vector2d(obstacle.radius_m * 2, obstacle.radius_m * 2), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), 1.0);

    if (kin::CheckCircularCollision(temp_robot, obstacle)) {
      return true;
    }
  }
  return false;
}

void algos::RRTX::AddNewObstacle(state::SoccerObject& new_obstacle) {
  // Line 2: EO ← {(v, u) ∈ E : π(v, u) ∩ O ≠ ∅} (Find intersecting edges)
  std::set<std::pair<int, int>> EO = GetEdgesIntersectingObstacle(new_obstacle);
  // YOU ALREADY HAVE GetEdgesIntersectingObstacle - REUSE IT!

  // Lines 3-6: Process each intersecting edge
  for (const auto& edge : EO) {
    int v_idx = edge.first;
    int u_idx = edge.second;

    // Line 4: dπ(v, u) ← ∞
    RemoveEdgeConnection(v_idx, u_idx);

    // Line 5: if p+T(v) = u then verrifyOrphan(v)
    if (Vertices[v_idx].parent_idx == u_idx) {
      VerifyOrphan(v_idx);  // YOU ALREADY HAVE THIS
    }

    // Line 6: if vbot ∈ π(v, u) then πbot = ∅
    if (IsRobotOnEdge(v_idx, u_idx)) {
      ClearRobotPath();  // Need to implement this
    }
  }
}

void algos::RRTX::RemoveEdgeConnection(int v_idx, int u_idx) {
  // Remove u from v's outgoing neighbors (both original and running)
  auto& N_plus_0_v = Vertices[v_idx].N_plus_0;
  N_plus_0_v.erase(std::remove(N_plus_0_v.begin(), N_plus_0_v.end(), u_idx), N_plus_0_v.end());

  auto& N_plus_r_v = Vertices[v_idx].N_plus_r;
  N_plus_r_v.erase(std::remove(N_plus_r_v.begin(), N_plus_r_v.end(), u_idx), N_plus_r_v.end());

  // Remove v from u's incoming neighbors (both original and running)
  auto& N_minus_0_u = Vertices[u_idx].N_minus_0;
  N_minus_0_u.erase(std::remove(N_minus_0_u.begin(), N_minus_0_u.end(), v_idx), N_minus_0_u.end());

  auto& N_minus_r_u = Vertices[u_idx].N_minus_r;
  N_minus_r_u.erase(std::remove(N_minus_r_u.begin(), N_minus_r_u.end(), v_idx), N_minus_r_u.end());

  // If this was a parent-child relationship, break it
  if (Vertices[v_idx].parent_idx == u_idx) {
    // Remove v from u's children
    auto& children = Vertices[u_idx].C_minus_T;
    children.erase(std::remove(children.begin(), children.end(), v_idx), children.end());

    // Mark v as having no parent
    Vertices[v_idx].parent_idx = -1;

    // Mark v as inconsistent (cost to goal becomes infinite)
    Vertices[v_idx].g = std::numeric_limits<double>::infinity();
    Vertices[v_idx].lmc = std::numeric_limits<double>::infinity();
  }

  if (Vertices[u_idx].parent_idx == v_idx) {
    // Remove u from v's children
    auto& children = Vertices[v_idx].C_minus_T;
    children.erase(std::remove(children.begin(), children.end(), u_idx), children.end());

    // Mark u as having no parent
    Vertices[u_idx].parent_idx = -1;

    // Mark u as inconsistent
    Vertices[u_idx].g = std::numeric_limits<double>::infinity();
    Vertices[u_idx].lmc = std::numeric_limits<double>::infinity();
  }
}

bool algos::RRTX::IsRobotOnEdge(int v_idx, int u_idx) {
  // Check if robot position lies on the edge between v and u
  const auto& robot_pos = Vertices[v_bot_idx].wp;
  const auto& v_pos = Vertices[v_idx].wp;
  const auto& u_pos = Vertices[u_idx].wp;

  // Simple check: is robot close to the line segment?
  double edge_length = d_pi(v_pos, u_pos);
  double dist_v_robot = d_pi(v_pos, robot_pos);
  double dist_u_robot = d_pi(u_pos, robot_pos);

  // Robot is on edge if dist_v_robot + dist_u_robot ≈ edge_length
  return std::abs((dist_v_robot + dist_u_robot) - edge_length) < 0.05;
}

void algos::RRTX::ClearRobotPath() {
  // Option 1: Mark robot as needing replanning
  Vertices[v_bot_idx].g = std::numeric_limits<double>::infinity();
  Vertices[v_bot_idx].lmc = std::numeric_limits<double>::infinity();
  Vertices[v_bot_idx].parent_idx = -1;

  // Option 2: Add to verification queue
  VerifyQueue(v_bot_idx);
}

void algos::RRTX::UpdateLMC(int v_idx) {
  double r = ShrinkingBallRadius();
  CullNeighbors(v_idx, r);

  Vertices[v_idx].lmc = std::numeric_limits<double>::infinity();
  int best_parent_idx = -1;

  std::vector<int> out_neighbors = getOutNeighbors(v_idx);
  for (int u_idx : out_neighbors) {
    if (V_c_T.find(u_idx) == V_c_T.end() && Vertices[u_idx].parent_idx != v_idx) {
      double new_cost = d_pi(Vertices[v_idx].wp, Vertices[u_idx].wp) + Vertices[u_idx].lmc;
      if (Vertices[v_idx].lmc > new_cost) {
        Vertices[v_idx].lmc = new_cost;
        best_parent_idx = u_idx;
      }
    }
  }

  if (best_parent_idx != -1) {
    MakeParentOf(best_parent_idx, v_idx);
  }
}

bool algos::RRTX::KeyLess(const std::pair<double, double>& key1,
                          const std::pair<double, double>& key2) {
  return (key1.first < key2.first) || (key1.first == key2.first && key1.second < key2.second);
}

void algos::RRTX::VerifyQueue(int v_idx) {
  auto key = getKey(v_idx);
  Q.push({key, v_idx});
}

std::pair<double, double> algos::RRTX::getKey(int v_idx) {
  return {std::min(Vertices[v_idx].g, Vertices[v_idx].lmc), Vertices[v_idx].g};
}

void algos::RRTX::MakeParentOf(int parent_idx, int child_idx) {
  if (Vertices[child_idx].parent_idx != -1) {
    int old_parent = Vertices[child_idx].parent_idx;
    auto& children = Vertices[old_parent].C_minus_T;
    children.erase(std::remove(children.begin(), children.end(), child_idx), children.end());
  }

  // Set new parent
  Vertices[child_idx].parent_idx = parent_idx;
  Vertices[parent_idx].C_minus_T.push_back(child_idx);
}

std::vector<int> algos::RRTX::getInNeighbors(int v_idx) {
  std::vector<int> neighbors = Vertices[v_idx].N_minus_0;
  neighbors.insert(neighbors.end(), Vertices[v_idx].N_minus_r.begin(),
                   Vertices[v_idx].N_minus_r.end());
  return neighbors;
}

std::vector<int> algos::RRTX::getOutNeighbors(int v_idx) {
  std::vector<int> neighbors = Vertices[v_idx].N_plus_0;
  neighbors.insert(neighbors.end(), Vertices[v_idx].N_plus_r.begin(),
                   Vertices[v_idx].N_plus_r.end());
  return neighbors;
}

bool algos::RRTX::TrajectoryValid(const state::Waypoint& a, const state::Waypoint& b) {
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

  for (auto& obstacle : current_obstacles) {
    state::SoccerObject temp_robot("temp_check", Eigen::Vector3d(wp.x, wp.y, wp.angle),
                                   Eigen::Vector2d(0.1, 0.1),  // Use robot size
                                   Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 1.0);

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

std::vector<int> algos::RRTX::Near(const state::Waypoint& wp, double r) {
  std::vector<int> near_vertices;
  for (size_t i = 0; i < Vertices.size(); ++i) {
    if (d_pi(wp, Vertices[i].wp) <= r) {
      near_vertices.push_back(i);
    }
  }
  return near_vertices;
}

int algos::RRTX::Nearest(const state::Waypoint& wp) {
  int nearest_idx = 0;
  double min_dist = d_pi(wp, Vertices[0].wp);

  for (size_t i = 1; i < Vertices.size(); ++i) {
    double dist = d_pi(wp, Vertices[i].wp);
    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = i;
    }
  }

  return nearest_idx;
}

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

  // Check if we have a solution from current robot position
  if (Vertices[v_bot_idx].g >= std::numeric_limits<double>::infinity()) {
    return path;  // Return empty path if no solution
  }

  // In RRT-X: follow parent chain from current robot position TO goal
  int current_idx = v_bot_idx;

  while (current_idx != -1) {
    path.push_back(Vertices[current_idx].wp);

    // Break if we reach goal
    if (current_idx == v_goal_idx) {
      break;
    }

    // Follow parent chain toward goal
    current_idx = Vertices[current_idx].parent_idx;

    // Safety check to prevent infinite loops
    if (path.size() > Vertices.size()) {
      std::cout << "[RRTX] Path reconstruction failed - possible cycle detected" << std::endl;
      return state::Path();  // Return empty path
    }
  }

  // Path now goes from current robot position to goal following parent pointers
  return path;
}

bool algos::RRTX::SolutionExists() {
  // In RRT-X, solution exists if robot can reach goal (finite cost-to-goal)
  return Vertices[v_bot_idx].g < std::numeric_limits<double>::infinity();
}

double algos::RRTX::GetSolutionCost() {
  // Return cost from current robot position to goal
  return Vertices[v_bot_idx].g;
}

void algos::RRTX::UpdateRobotPosition(const state::Waypoint& new_pos) {
  Vertices[v_bot_idx].wp = new_pos;
}

bool algos::RRTX::HasObstaclesChanged(const std::vector<state::SoccerObject>& new_obstacles) {
  // Compare new obstacles with current obstacles (not previous!)
  if (new_obstacles.size() != current_obstacles.size()) {
    return true;  // Different number of obstacles
  }

  const double MOVEMENT_THRESHOLD = 0.05;  // 5cm threshold

  // Use name-based comparison instead of index-based
  for (const auto& new_obs : new_obstacles) {
    bool found = false;
    for (const auto& current_obs : current_obstacles) {  // Changed from previous_obstacles
      if (new_obs.name == current_obs.name) {
        double distance = (new_obs.position - current_obs.position).norm();
        if (distance > MOVEMENT_THRESHOLD) {
          return true;  // Found movement > threshold
        }
        found = true;
        break;
      }
    }
    if (!found) {
      return true;  // New obstacle appeared
    }
  }

  // Check for vanished obstacles
  for (const auto& current_obs : current_obstacles) {
    bool found = false;
    for (const auto& new_obs : new_obstacles) {
      if (current_obs.name == new_obs.name) {
        found = true;
        break;
      }
    }
    if (!found) {
      return true;  // Obstacle vanished
    }
  }

  return false;  // No significant changes
}

bool algos::RRTX::IsRobotPoseChanged() { return false; }

bool algos::RRTX::IsInVertices(state::Waypoint v_new_wp) {
  for (int i = Vertices.size() - 1; i >= 0; i--) {
    if (Vertices[i].wp == v_new_wp) return true;
  }
  return false;
}

void algos::RRTX::UpdateGoal(const state::Waypoint& new_goal) {
  // Check if goal actually changed
  if (Vertices[v_goal_idx].wp == new_goal) {
    return;
  }

  // Update goal position
  Vertices[v_goal_idx].wp = new_goal;

  // Mark all vertices as inconsistent since the goal changed
  // This forces recalculation of all cost-to-goal values
  for (size_t i = 0; i < Vertices.size(); ++i) {
    if (i != v_goal_idx) {
      // Invalidate all costs except goal
      Vertices[i].g = std::numeric_limits<double>::infinity();
      Vertices[i].lmc = std::numeric_limits<double>::infinity();

      // Add to priority queue for reprocessing
      VerifyQueue(i);
    }
  }

  // Clear all parent-child relationships except goal
  for (size_t i = 0; i < Vertices.size(); ++i) {
    if (i != v_goal_idx) {
      // Remove from old parent's children list
      if (Vertices[i].parent_idx != -1) {
        auto& parent_children = Vertices[Vertices[i].parent_idx].C_minus_T;
        parent_children.erase(std::remove(parent_children.begin(), parent_children.end(), i),
                              parent_children.end());
      }
      Vertices[i].parent_idx = -1;
      Vertices[i].C_minus_T.clear();
    }
  }

  // Goal vertex properties remain the same
  Vertices[v_goal_idx].g = 0.0;
  Vertices[v_goal_idx].lmc = 0.0;
  Vertices[v_goal_idx].parent_idx = -1;

  // Process inconsistencies to rebuild tree with new goal
  ReduceInconsistency();
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
         ((obs1.position - obs2.position).norm() < 0.05);  // 5cm threshold
}