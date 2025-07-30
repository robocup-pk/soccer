#include "Game.h"
#include "SoccerField.h"
#include <iostream>
#include "GLSimulation.h"

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

void ref::Game::DoGoals(std::vector<state::SoccerObject>& soccer_objects) {
  for (auto& obj : soccer_objects) {
    if (obj.name == "ball") {
      double left_goal_x1 = -(((vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f) +
                               vis::SoccerField::GetInstance().goal_width_mm)) /
                            1000.0f;
      double left_goal_x2 =
          -((vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f)) / 1000.0f;

      double left_goal_y1 = (vis::SoccerField::GetInstance().goal_height_mm / 2.0f) / 1000.0f;
      double left_goal_y2 = -((vis::SoccerField::GetInstance().goal_height_mm / 2.0f) / 1000.0f);

      double right_goal_x1 =
          (vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f) / 1000.0f;
      double right_goal_x2 = ((vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f) +
                              vis::SoccerField::GetInstance().goal_width_mm) /
                             1000.0f;

      double right_goal_y1 = (vis::SoccerField::GetInstance().goal_height_mm / 2.0f) / 1000.0f;
      double right_goal_y2 = -((vis::SoccerField::GetInstance().goal_height_mm / 2.0f) / 1000.0f);

      bool goal_scored = false;
      float left = obj.position[0];
      float right = obj.position[0] + obj.size[0];
      float top = obj.position[1] + obj.size[1];
      float bottom = obj.position[1];

      if ((left >= left_goal_x1 && left <= left_goal_x2 && top <= left_goal_y1 &&
           bottom >= left_goal_y2) ||
          (right >= left_goal_x1 && right <= left_goal_x2 && top >= left_goal_y1 &&
           bottom <= left_goal_y2)) {
        team_two_score++;
        goal_scored = true;
        team_with_ball = 1;
        std::cout << "[ref::Game::DoGoals] RED TEAM SCORED! "
                  << "red team score: " << team_two_score << std::endl;
      } else if (right >= right_goal_x1 && right <= right_goal_x2 && top <= right_goal_y1 &&
                 bottom >= right_goal_y2) {
        team_one_score++;
        goal_scored = true;
        team_with_ball = 2;
        std::cout << "[ref::Game::DoGoals] YELLOW TEAM SCORED! "
                  << "yellow team score: " << team_one_score << std::endl;
      }

      if (goal_scored) {
        if (obj.is_attached) {
          obj.is_attached = false;
          state::SoccerObject* robot = obj.attached_to;
          if (robot) {
            robot->is_attached = false;
            robot->attached_to = nullptr;
          }
          obj.attached_to = nullptr;
        }

        obj.position = Eigen::Vector3d::Zero();
        obj.velocity = Eigen::Vector3d::Zero();

        state = Game::Kickoff;
        std::cout << "[ref::Game::DoGoals] GAME STATE IS KICKOFF" << std::endl;
        state_start_ball_pos = soccer_objects[soccer_objects.size() - 1].position;
        vis::team_one_selected_player = 0;
        vis::team_two_selected_player = cfg::SystemConfig::num_robots / 2;

        if (team_with_ball == 1) {
          std::cout << "[ref::Game::DoGoals] team one has the ball" << std::endl;
          cfg::SystemConfig::team_one_start_formation[0] = Eigen::Vector3d(
              -1 * cfg::SystemConfig::ball_radius_m - cfg::SystemConfig::robot_size_m[0] / 2.0,
              0.0f, 0);
          cfg::SystemConfig::team_two_start_formation[0] = Eigen::Vector3d(0.5f, 0.0f, M_PI);
        } else if (team_with_ball == 2) {
          std::cout << "[ref::Game::DoGoals] team two has the ball" << std::endl;
          cfg::SystemConfig::team_two_start_formation[0] = Eigen::Vector3d(
              cfg::SystemConfig::ball_radius_m + cfg::SystemConfig::robot_size_m[0] / 2.0, 0.0f,
              M_PI);
          cfg::SystemConfig::team_one_start_formation[0] = Eigen::Vector3d(-0.5f, 0.0f, 0);
        }

        ref::Game::MoveToFormation(cfg::SystemConfig::team_one_start_formation,
                                   cfg::SystemConfig::team_two_start_formation, soccer_objects);

        for (auto& obj : soccer_objects) {
          obj.velocity = Eigen::Vector3d(0, 0, 0);
        }
      }

      break;
    }
  }
}

void ref::Game::UpdateGameState(std::vector<state::SoccerObject>& soccer_objects) {
  if (state == Kickoff) {
    Eigen::Vector3d ball_displacement =
        state_start_ball_pos - soccer_objects[soccer_objects.size() - 1].position;
    if (ball_displacement.norm() > 0.05) {
      state = Run;
      std::cout << "[ref::Game::UpdateGameState] GAME STATE IS RUN" << std::endl;
    }
  }
}

ref::Game::Game() { state = Kickoff; }