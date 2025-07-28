#ifndef GAME_H
#define GAME_H

#include <vector>

#include "SoccerObject.h"

namespace ref {

class Game {
 public:
  static void MoveToFormation(std::vector<Eigen::Vector3d> team_one_formation,
                              std::vector<Eigen::Vector3d> team_two_formation,
                              std::vector<state::SoccerObject>& soccer_objects);
};

};  // namespace ref

#endif  // GAME_Ha