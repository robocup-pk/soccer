#include "Game.h"
#include "SoccerField.h"
#include <iostream>
#include "GLSimulation.h"

#include "Kinematics.h"
#include "Dimensions.h"

void ref::Game::SetUpKickOff(std::vector<state::SoccerObject>& soccer_objects) {
  if (soccer_objects.size() > 12) {
    // figure out formations
    std::vector<Eigen::Vector3d> team_one_kickoff_formation =
        cfg::SystemConfig::team_one_start_formation;
    std::vector<Eigen::Vector3d> team_two_kickoff_formation =
        cfg::SystemConfig::team_two_start_formation;
    if (team_with_ball == 1) {
      team_one_kickoff_formation[0] = Eigen::Vector3d(
          -1 * cfg::SystemConfig::ball_radius_m - cfg::SystemConfig::robot_size_m[0] / 2.0, 0.0f,
          0);
      team_two_kickoff_formation[0] = Eigen::Vector3d(0.5f, 0.0f, M_PI);
      soccer_objects[soccer_objects.size() - 1].attached_to =
          &soccer_objects[cfg::SystemConfig::team_one_kicker];
      kin::HandleBallSticking(soccer_objects[cfg::SystemConfig::team_one_kicker],
                              soccer_objects[soccer_objects.size() - 1]);
    } else if (team_with_ball == 2) {
      team_two_kickoff_formation[0] = Eigen::Vector3d(
          cfg::SystemConfig::ball_radius_m + cfg::SystemConfig::robot_size_m[0] / 2.0, 0.0f, M_PI);
      team_one_kickoff_formation[0] = Eigen::Vector3d(-0.5f, 0.0f, 0);
      soccer_objects[soccer_objects.size() - 1].attached_to =
          &soccer_objects[cfg::SystemConfig::team_two_kicker];
      kin::HandleBallSticking(soccer_objects[cfg::SystemConfig::team_two_kicker],
                              soccer_objects[soccer_objects.size() - 1]);
    }

    // now move the positions
    for (int i = 0; i < soccer_objects.size() / 2; i++) {
      soccer_objects[i].position = team_one_kickoff_formation[i];
      soccer_objects[i + soccer_objects.size() / 2].position = team_two_kickoff_formation[i];
    }

    for (auto& obj : soccer_objects) {
      obj.velocity = Eigen::Vector3d(0, 0, 0);  // set velocities to zero
    }

    // adjust ball position
    if (team_with_ball == 1) {
      soccer_objects[soccer_objects.size() - 1].position = cfg::SystemConfig::init_ball_position;
    } else if (team_with_ball == 2) {
      soccer_objects[soccer_objects.size() - 1].position =
          -1 * cfg::SystemConfig::init_ball_position;
    }

    // resetting state
    for (int i = 0; i < soccer_objects.size(); i++) {
      soccer_objects[i].is_selected_player = false;
      soccer_objects[i].was_given_speeding_foul_in_stop = false;
    }

    vis::team_one_selected_player = 0;
    vis::team_two_selected_player = cfg::SystemConfig::num_robots / 2;
    vis::last_kick_valid = false;

    state_start_ball_pos = soccer_objects[soccer_objects.size() - 1].position;
    last_valid_pos = soccer_objects[soccer_objects.size() - 1].position;
    prev_state = Game::Kickoff;
    state = Game::Kickoff;
    kicker_released = false;
    nonkicking_robot_intercepted = false;
    last_attached_pos = cfg::SystemConfig::init_ball_position;
    std::cout << "[ref::Game::SetUpKickoff] GAME STATE IS KICKOFF" << std::endl;
  }
}

void ref::Game::SetUpFreeKick(std::vector<state::SoccerObject>& soccer_objects,
                              Eigen::Vector3d set_pos) {
  // stop all motion
  for (auto& obj : soccer_objects) {
    obj.velocity = Eigen::Vector3d(0, 0, 0);  // set velocities to zero
  }

  if (last_bot_touched_ball.team_id == 1) {
    team_with_ball = 2;
  } else if (last_bot_touched_ball.team_id == 2) {
    team_with_ball = 1;
  }

  // set the ball position
  soccer_objects[soccer_objects.size() - 1].position = set_pos;

  // setting up robot positions
  std::vector<Eigen::Vector3d> team_one_freekick_formation =
      cfg::SystemConfig::team_one_start_formation;
  std::vector<Eigen::Vector3d> team_two_freekick_formation =
      cfg::SystemConfig::team_two_start_formation;

  if (team_with_ball == 1) {
    team_one_freekick_formation[0] = Eigen::Vector3d(
        set_pos[0] - cfg::SystemConfig::ball_radius_m - cfg::SystemConfig::robot_size_m[0] / 2.0,
        set_pos[1], 0);
  } else if (team_with_ball == 2) {
    team_two_freekick_formation[0] = Eigen::Vector3d(
        set_pos[0] + cfg::SystemConfig::ball_radius_m + cfg::SystemConfig::robot_size_m[0] / 2.0,
        set_pos[1], M_PI);
  }

  // moving the robots to those positions
  for (int i = 0; i < soccer_objects.size() / 2; i++) {
    soccer_objects[i].position = team_one_freekick_formation[i];
    soccer_objects[i + soccer_objects.size() / 2].position = team_two_freekick_formation[i];
  }

  // fix sticking
  if (team_with_ball == 1) {
    soccer_objects[soccer_objects.size() - 1].attached_to =
        &soccer_objects[cfg::SystemConfig::team_one_kicker];
    kin::HandleBallSticking(soccer_objects[cfg::SystemConfig::team_one_kicker],
                            soccer_objects[soccer_objects.size() - 1]);
  } else if (team_with_ball == 2) {
    soccer_objects[soccer_objects.size() - 1].attached_to =
        &soccer_objects[cfg::SystemConfig::team_two_kicker];
    kin::HandleBallSticking(soccer_objects[cfg::SystemConfig::team_two_kicker],
                            soccer_objects[soccer_objects.size() - 1]);
  }

  // resetting state
  for (int i = 0; i < soccer_objects.size(); i++) {
    soccer_objects[i].is_selected_player = false;
    soccer_objects[i].was_given_speeding_foul_in_stop = false;
  }

  vis::team_one_selected_player = 0;
  vis::team_two_selected_player = cfg::SystemConfig::num_robots / 2;
  vis::last_kick_valid = false;
  state_start_ball_pos = soccer_objects[soccer_objects.size() - 1].position;
  last_valid_pos = soccer_objects[soccer_objects.size() - 1].position;
  prev_state = Game::FreeKick;
  state = Game::FreeKick;
  kicker_released = false;
  nonkicking_robot_intercepted = false;
  displacement = 0;
  last_attached_pos = set_pos;
  bot_got_foul_for_overdribbling = false;
  std::cout << "[ref::Game::SetUpFreekick] GAME STATE IS FREEKICK" << std::endl;
}

void ref::Game::CheckCollisions(std::vector<state::SoccerObject>& soccer_objects) {
  if (was_ball_prev_attached && !soccer_objects[soccer_objects.size() - 1].is_attached) {
    was_ball_prev_attached = false;
    kicker_released = true;
  } else if (soccer_objects[soccer_objects.size() - 1].is_attached && !was_ball_prev_attached) {
    last_attached_pos = soccer_objects[soccer_objects.size() - 1].position;
    was_ball_prev_attached = true;

    if (state != ref::Game::FreeKick) {
      bot_got_foul_for_overdribbling = false;
      displacement = 0;
    }
  }

  if (soccer_objects[soccer_objects.size() - 1].is_attached) {
    state::SoccerObject& robot = *soccer_objects[soccer_objects.size() - 1].attached_to;
    last_bot_touched_ball = robot;
    if (kicker_released) {
      if (team_with_ball == 1 &&
          (robot.role != state::SoccerObject::Kicker ||
           robot.team_id == 2)) {  // valid interceptions are all nonkickers and all team 2 robots
        nonkicking_robot_intercepted = true;
      } else if (team_with_ball == 2 &&
                 (robot.role != state::SoccerObject::Kicker || robot.team_id == 1)) {
        nonkicking_robot_intercepted = true;
      }
      kin::UpdateAttachedBallPosition(robot, soccer_objects[soccer_objects.size() - 1]);
    }
  }
}

void ref::Game::DoGoals(std::vector<state::SoccerObject>& soccer_objects, int scoring_team) {
  if (scoring_team == 1) {
    team_one_score++;
    team_with_ball = 2;
  } else if (scoring_team == 2) {
    team_two_score++;
    team_with_ball = 1;
  }
  if (soccer_objects[soccer_objects.size() - 1].is_attached) {
    soccer_objects[soccer_objects.size() - 1].is_attached = false;
    state::SoccerObject* robot = soccer_objects[soccer_objects.size() - 1].attached_to;
    if (robot) {
      robot->is_attached = false;
      robot->attached_to = nullptr;
    }
    soccer_objects[soccer_objects.size() - 1].attached_to = nullptr;
  }

  ref::Game::SetUpKickOff(soccer_objects);
}

void ref::Game::UpdateGameState(std::vector<state::SoccerObject>& soccer_objects) {
  if (vis::button_pressed == 1) {
    state = Stop;
  } else if (vis::button_pressed == 2) {
    state = Halt;
  } else if (state == Kickoff || state == FreeKick) {
    Eigen::Vector3d ball_displacement =
        state_start_ball_pos - soccer_objects[soccer_objects.size() - 1].position;
    if (ball_displacement.norm() >
        0.2) {  // should be 0.05 for game rules but this makes vis easy rn
      prev_state = state;
      state = Run;
      std::cout << "[ref::Game::UpdateGameState] GAME STATE IS RUN" << std::endl;
    }
  }
}

bool ref::Game::CheckCollisionWithWall(state::SoccerObject& obj) {
  float half_width = (vis::SoccerField::GetInstance().width_mm / 2) / 1000.0f;
  float half_height = (vis::SoccerField::GetInstance().height_mm / 2) / 1000.0f;

  float left = obj.position[0];
  float right = obj.position[0] + obj.size[0];
  float top = obj.position[1] + obj.size[1];
  float bottom = obj.position[1];

  return (left <= -half_width || right >= half_width || top >= half_height ||
          bottom <= -half_height);
}

ref::Game::Game(std::vector<state::SoccerObject>& soccer_objects) {
  state = Kickoff;
  state_start_ball_pos = soccer_objects[soccer_objects.size() - 1].position;
  last_bot_touched_ball = soccer_objects[cfg::SystemConfig::team_one_kicker];
  last_valid_pos = soccer_objects[soccer_objects.size() - 1].position;
}