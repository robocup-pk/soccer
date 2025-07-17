#include <vector>
#include <queue>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <algorithm>
#include <set>
#include <functional>

#include "Algos.h"
#include "Utils.h"
#include "RRTX.h"

namespace algos {

state::Waypoint FindDirectionVector(const state::Waypoint& start, const state::Waypoint& goal) {
  state::Waypoint direction_vector = goal - start;
  return direction_vector.Normalize();
}

state::Waypoint SelectGoal(std::vector<state::SoccerObject>& soccer_objects) {
  state::Waypoint goal_pos;

  float dx = soccer_objects[0].position[0] - soccer_objects[1].position[0];
  float dy = soccer_objects[0].position[1] - soccer_objects[1].position[1];
  float angle_to_ball = util::WrapAngle(std::atan2(dy, dx));  // result is in radians

  // Is ball is attached, go to the field's goal
  // Otherwise, the ball position is our goal
  std::cout << "a\n";
  bool bot_has_ball = (soccer_objects[0].is_attached == 1);
  goal_pos = bot_has_ball
                 ? state::Waypoint(1.7, -0.09f, 1.0f)  // Go to goal if bot has ball
                 : state::Waypoint(soccer_objects[0].position[0], soccer_objects[0].position[1],
                                   angle_to_ball);  // Chase ball

  std::cout << "b\n";
  // Can make any area as goal, based on some logic
  // ...

  return goal_pos;
}

state::Path PlanPath(AlgoName algo_name, std::vector<state::SoccerObject>& soccer_objects) {
  state::Path path;

  Eigen::Vector3d& bot_pos3d = soccer_objects[1].position;
  state::Waypoint bot_pos(bot_pos3d[0], bot_pos3d[1], 0.0f);
  state::Waypoint goal_pos(SelectGoal(soccer_objects));
  std::cout << "c\n";

  switch (algo_name) {
    case AlgoName::RRTX:
      path = algos::FindSinglePath_RRTX(bot_pos, goal_pos);
      std::cout << "Path: " << path << std::endl;
      std::cout << "d\n";
      break;
    case AlgoName::ASTAR:
      // path =;
      break;
  }

  return path;
}

}  // namespace algos