#include "Game.h"

void ref::Game::MoveToFormation(std::vector<Eigen::Vector3d> team_one_formation,
                                std::vector<Eigen::Vector3d> team_two_formation,
                                std::vector<state::SoccerObject>& soccer_objects) {
  if (soccer_objects.size() > 12) {
    for (int i = 0; i < soccer_objects.size() / 2; i++) {
      soccer_objects[i].position = cfg::SystemConfig::team_one_start_formation[i];
      soccer_objects[i + soccer_objects.size() / 2].position =
          cfg::SystemConfig::team_two_start_formation[i];
    }
  }
}
