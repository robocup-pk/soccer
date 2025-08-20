#include <iostream>

#include "AutoRef.h"
#include "Kinematics.h"
#include "SoccerField.h"
#include "Game.h"
#include "GLSimulation.h"
#include "Dimensions.h"

// GAME EVENTS

// Violated KickOff Configuration
/*
  When the kick-off command is issued, all robots have to move to their own half of the field
  excluding the center circle. However, one robot of the attacking team is also allowed to be
  inside the whole center circle. This robot will be referred to as the kicker. No robot is allowed
  to touch the ball.
*/

bool ref::AutoRef::ViolatedKickOffSetUp(std::vector<state::SoccerObject>& soccer_objects,
                                        int team_id, Game& g) {
  if (g.state != ref::Game::Kickoff) {
    return false;
  }

  if (team_id == 1) {
    for (int i = 0; i < soccer_objects.size(); i++) {
      if (soccer_objects[i].team_id == 1 &&
          (soccer_objects[i].position[0] + cfg::SystemConfig::robot_size_m[0] / 2.0 > 0 ||
           (g.team_with_ball == 1 && RobotInCenterCircle(soccer_objects[i].position) &&
            i != cfg::SystemConfig::team_one_kicker) ||
           (g.team_with_ball != 1 && RobotInCenterCircle(soccer_objects[i].position)))) {
        std::cout << "[ref::ViolatedKickOffSetUp] team 1 violated kickoff start arrangement"
                  << std::endl;
        return true;
      }
    }
  } else {
    for (int i = 0; i < soccer_objects.size(); i++) {
      if (soccer_objects[i].team_id == 2 &&
          (soccer_objects[i].position[0] - cfg::SystemConfig::robot_size_m[0] / 2.0 < 0 ||
           (g.team_with_ball == 2 && RobotInCenterCircle(soccer_objects[i].position) &&
            i != cfg::SystemConfig::team_two_kicker) ||
           (g.team_with_ball != 2 && RobotInCenterCircle(soccer_objects[i].position)))) {
        std::cout << "[ref::ViolatedKickOffSetUp] team 2 violated kickoff start arrangement"
                  << std::endl;
        return true;
      }
    }
  }

  return false;
}

/* helper for ViolateKickOffSetUp */
bool ref::AutoRef::RobotInCenterCircle(Eigen::Vector3d v) {
  double x = v[0];
  double y = v[1];
  double r_1 = cfg::SystemConfig::robot_size_m[0] / 2.0;
  double r_2 = 0.2;

  if (std::sqrt(std::abs(x * x) + std::abs(y * y)) <= r_1 + r_2) {
    return true;
  }
  return false;
}

/* All of these rules come from the 2025 RoboCup Manual
 * https://robocup-ssl.github.io/ssl-rules/sslrules.pdf
 */

// #1 ATTACKER DOUBLE TOUCHED BALL

/* When the ball is brought into play following a kick-off or free kick, the kicker is not
allowed to touch the ball until it has been touched by another robot or the game has been
stopped. The ball must have moved at least 0.05 meters to be considered as in play. A double
touch results in a stop followed by a free kick from the same ball position. */

int ref::AutoRef::AttackerDoubleTouchedBall(std::vector<state::SoccerObject>& soccer_objects,
                                            Game& g) {
  if (g.state == ref::Game::Run &&
      (g.prev_state == ref::Game::Kickoff || g.prev_state == ref::Game::FreeKick)) {
    if (g.team_with_ball == 1) {
      if (soccer_objects[cfg::SystemConfig::team_one_kicker].is_attached && g.kicker_released &&
          !g.nonkicking_robot_intercepted) {
        std::cout << "[AutoRef::AttackerDoubleTouchedBall] *YELLOW* double touch foul"
                  << std::endl;
        return 1;
      }
    } else if (g.team_with_ball == 2) {
      if (soccer_objects[cfg::SystemConfig::team_two_kicker].is_attached && g.kicker_released &&
          !g.nonkicking_robot_intercepted) {
        std::cout << "[AutoRef::AttackerDoubleTouchedBall] *RED* Double Touch Foul" << std::endl;
        return 2;
      }
    }
  }
  return 0;
}

// #2 POSSIBLE GOAL

/* A team scores a goal when the ball fully enters the opponent goal between the goal posts,
provided that: • The team did not exceed the allowed number of robots when the ball entered the
goal.
• The height of the ball did not exceed 0.15 meters after the last touch of the teams
robots.
• The team did not commit any non stopping foul in the last two seconds before the ball
entered the goal.
 During penalty kicks, more specific rules apply. If the goal is considered
invalid, the game will be continued as if the ball crossed the goal line outside the goal. */

int ref::AutoRef::CheckGoal(std::vector<state::SoccerObject>& soccer_objects, Game& g) {
  for (auto& obj : soccer_objects) {
    if (obj.name == "ball") {
      double left_goal_x1 = (-vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f +
                             vis::SoccerField::GetInstance().goal_width_mm / 2.0f) /
                            1000.0f;
      double left_goal_x2 =
          -((vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f)) / 1000.0f;

      double goal_y1 = (vis::SoccerField::GetInstance().goal_height_mm / 2.0f) / 1000.0f;
      double goal_y2 = -((vis::SoccerField::GetInstance().goal_height_mm / 2.0f) / 1000.0f);

      double right_goal_x1 = (vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f -
                              vis::SoccerField::GetInstance().goal_width_mm / 2.0f) /
                             1000.0f;
      double right_goal_x2 =
          vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f / 1000.0f;

      float left = obj.position[0] - cfg::SystemConfig::ball_radius_m;
      float right = obj.position[0] + cfg::SystemConfig::ball_radius_m;
      float top = obj.position[1] + cfg::SystemConfig::ball_radius_m;
      float bottom = obj.position[1] - cfg::SystemConfig::ball_radius_m;

      if (left < left_goal_x1 && bottom <= goal_y1 && top >= goal_y2) {
        return 2;
      } else if (right > right_goal_x1 && bottom <= goal_y1 && top >= goal_y2) {
        return 1;
      }
    }
  }
  return 0;
}

// #3 BALL LEFT FIELD TOUCH LINE

/* Touch lines are the long field lines at both sides of the playing field.

The ball has to be placed 0.2 meters perpendicular to the touch line where the ball crossed the
touch line. Its distance to the goal lines must be at least 0.2 meters.
After the ball has been placed, a free kick is awarded to the opponent of the team that last
touched the ball before it left the field.

and // #7 BOUNDARY CROSSING
 A robot must not kick the ball over the field boundary such that the ball leaves the field.*/

bool ref::AutoRef::BallLeftFieldTouchLine(std::vector<state::SoccerObject>& soccer_objects,
                                          Game& g) {
  if (IsOutsidePlayingFieldTouchLines(soccer_objects[soccer_objects.size() - 1]) &&
      !soccer_objects[soccer_objects.size() - 1].is_attached) {
    if (g.last_bot_touched_ball.team_id == 1) {
      team_one_foul_counter++;
      std::cout << "[ref::AutoRef::BallLeftFieldGoalLines] *YELLOW* boundary crossing detected. "
                   "foul count: "
                << team_one_foul_counter << std::endl;
    } else if (g.last_bot_touched_ball.team_id == 2) {
      team_two_foul_counter++;
      std::cout << "[ref::AutoRef::BallLeftFieldGoalLines] *RED* boundary crossing detected. "
                   "foul count: "
                << team_two_foul_counter << std::endl;
    }
    return true;
  }
  return false;
}

/* Helper for ref::BallLeftFieldTouchLine */
bool ref::AutoRef::IsOutsidePlayingFieldTouchLines(state::SoccerObject& obj) {
  float half_height = (vis::SoccerField::GetInstance().playing_area_height_mm / 2.0f) / 1000.0f;

  float top = obj.position[1] + obj.size[1] / 2.0;
  float bottom = obj.position[1] - obj.size[1] / 2.0;

  return (top > half_height || bottom < -1 * half_height);
}

// #4 BALL LEFT FIELD GOAL LINE + #5 AIMLESS KICK

/*
Goal lines are the short field lines at both ends of the playing field. The ball has to be placed
0.2 meters from the closest touch line and 1 meter from the goal line. After the ball has been
placed, a free kick is awarded to the opponent of the team that last touched the ball before it
left the field.

The ball has to be placed at the position from where the ball was kicked (see the free kick rules
for the exact ball position rules). After the ball has been placed, a free kick is awarded to the
opponent of the team that last touched the ball before it left the field. A kick is aimless
when after the ball touched a robot, it subsequently crossed the halfline and then its opponent’s
goal line outside the goal without touching another robot. A kick-off
and

// #7 BOUNDARY CROSSING

A robot must not kick the ball over the field boundary such that the ball leaves the field. */

std::vector<bool> ref::AutoRef::BallLeftFieldGoalLines(
    std::vector<state::SoccerObject>& soccer_objects, Game& g) {
  std::vector<bool> b(5, false);  // 0 = aimless kick no goal, 1 = aimless kick, 2 = boundary
                                  // crossing, 3 = crossed left, 4 = crossed right

  int scoring_team = ref::AutoRef::CheckGoal(soccer_objects, g);

  float half_width = (vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f) / 1000.0f;
  float left = soccer_objects[soccer_objects.size() - 1].position[0] -
               soccer_objects[soccer_objects.size() - 1].size[0] / 2.0;
  float right = soccer_objects[soccer_objects.size() - 1].position[0] +
                soccer_objects[soccer_objects.size() - 1].size[0] / 2.0;
  bool crossed_from_left = left <= -half_width;
  bool crossed_from_right = right >= half_width;
  bool crossed = crossed_from_left || crossed_from_right;
  b[3] = crossed_from_left;
  b[4] = crossed_from_right;

  if (scoring_team == 1) {
    if (vis::last_ball_kick_pos[0] < 0) {
      std::cout << "[ref::BallLeftFieldGoalLines] *YELLOW* aimless kick detected. No goal."
                << std::endl;
      b[0] = true;
    }
  } else if (scoring_team == 2) {
    if (vis::last_ball_kick_pos[0] > 0) {
      std::cout << "[ref::BallLeftFieldGoalLines] *RED* aimless kick detected. No goal."
                << std::endl;
      b[0] = true;
    }
  } else {
    if (crossed_from_left && vis::last_ball_kick_pos[0] > 0) {
      if (g.last_bot_touched_ball.team_id == 1) {
        std::cout << "[ref::BallLeftFieldGoalLines] *YELLOW* aimless kick detected" << std::endl;
      } else if (g.last_bot_touched_ball.team_id == 2) {
        std::cout << "[ref::BallLeftFieldGoalLines] *RED* aimless kick detected" << std::endl;
      }
      b[1] = true;
    } else if (crossed_from_right && vis::last_ball_kick_pos[0] < 0) {
      if (g.last_bot_touched_ball.team_id == 1) {
        std::cout << "[ref::BallLeftFieldGoalLines] *YELLOW* aimless kick detected" << std::endl;
      } else if (g.last_bot_touched_ball.team_id == 2) {
        std::cout << "[ref::BallLeftFieldGoalLines] *RED* aimless kick detected" << std::endl;
      }
      b[1] = true;
    } else if (crossed) {  // not aimless nor a goal
      // update fouls
      if (g.last_bot_touched_ball.team_id == 1) {
        team_one_foul_counter++;
        std::cout << "[ref::AutoRef::BallLeftFieldGoalLines] *YELLOW* boundary crossing detected. "
                     "foul count: "
                  << team_one_foul_counter << std::endl;
      } else if (g.last_bot_touched_ball.team_id == 2) {
        team_two_foul_counter++;
        std::cout << "[ref::AutoRef::BallLeftFieldGoalLines] *RED* boundary crossing detected. "
                     "foul count: "
                  << team_two_foul_counter << std::endl;
      }
      b[2] = true;
    }
  }
  return b;
}

// #6 DEFENDER IN DEFENSE AREA

/* Robots other than the keeper must maintain best-effort to fully stay outside the own defense
area. Infraction of this rule can be rated as unsporting behavior. If a robot other than the keeper
touches the ball while this robot is entirely inside its own defense area, the game is stopped and
a penalty kick is awarded to the other team. The foul counter is not increased. */

void ref::AutoRef::DefenderInDefenseArea(std::vector<state::SoccerObject>& soccer_objects,
                                         Game& g) {
  // team one check
  float half_width = (vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f) / 1000.0f;
  float half_height = (vis::SoccerField::GetInstance().playing_area_height_mm / 2.0f) / 1000.0f;
  float half_penalty_width =
      (vis::SoccerField::GetInstance().penalty_area_width_mm / 2.0f) / 1000.0f;
  float half_penalty_height =
      (vis::SoccerField::GetInstance().penalty_area_height_mm / 2.0f) / 1000.0f;
  float robo_radius = cfg::SystemConfig::robot_size_m[0] / 2.0;

  for (int i = 0; i < soccer_objects.size() / 2; i++) {
    float x_pos = soccer_objects[i].position[0];
    float y_pos = soccer_objects[i].position[1];
    if (soccer_objects[i].name != "ball" && i != cfg::SystemConfig::team_one_goalie) {
      if (x_pos - robo_radius < -half_width + half_penalty_width &&
          y_pos - robo_radius < half_penalty_height &&
          y_pos + robo_radius > -half_penalty_height && soccer_objects[i].is_attached &&
          !g.was_ball_prev_attached) {
        std::cout << "[ref::DefenderInDefenseArea] team one defender illegaly touched ball in "
                     "penalty box"
                  << std::endl;
        // do penalty kick
      }
    }
  }

  // team two check

  for (int i = soccer_objects.size() / 2; i < soccer_objects.size(); i++) {
    float x_pos = soccer_objects[i].position[0];
    float y_pos = soccer_objects[i].position[1];
    if (soccer_objects[i].name != "ball" && i != cfg::SystemConfig::team_two_goalie) {
      if (x_pos + robo_radius > half_width - half_penalty_width &&
          y_pos - robo_radius < half_penalty_height &&
          y_pos + robo_radius > -half_penalty_height && soccer_objects[i].is_attached &&
          !g.was_ball_prev_attached) {
        std::cout << "[ref::DefenderInDefenseArea] team two defender illegaly touched ball in "
                     "penalty box"
                  << std::endl;
        // do penalty kick
      }
    }
  }
}

// #8 BOT DRIBBLED BALL TOO FAR

/* A robot must not dribble the ball further than 1 meter, measured linearly from the ball location
where the dribbling started. A robot begins dribbling when it makes contact with the ball and stops
dribbling when there is an observable separation between the ball and the robot. */

bool ref::AutoRef::BotDribbledBallTooFar(std::vector<state::SoccerObject>& soccer_objects,
                                         Game& g) {
  if (g.bot_got_foul_for_overdribbling) return false;

  g.displacement =
      (g.last_attached_pos - soccer_objects[soccer_objects.size() - 1].position).norm();
  if (g.displacement <= 1) {
    g.last_valid_pos =
        soccer_objects[soccer_objects.size() - 1]
            .position;  // we need to do this so we can set up the freekick from last valid pos
                        // aka where the player should have legally stopped dribbling
  }
  if (g.displacement > 1 && soccer_objects[soccer_objects.size() - 1].is_attached) {
    if (g.last_bot_touched_ball.team_id == 1) {
      team_one_foul_counter++;
      std::cout << "[ref::AutoRef::BotDribbledBallTooFar] *YELLOW* robot dribbled at least 1 m."
                   " foul count: "
                << team_one_foul_counter << std::endl;
      g.bot_got_foul_for_overdribbling = true;
      return true;
    } else {
      team_two_foul_counter++;
      std::cout << "[ref::AutoRef::BotDribbledBallTooFar] *RED* robot dribbled at least 1 m. foul "
                   "count: "
                << team_two_foul_counter << std::endl;
      g.bot_got_foul_for_overdribbling = true;
      return true;
    }
  }
  return false;
}

// #9 ATTACKER TOUCHED BALL IN OPPONENT DEFENSE AREA

/* The ball must not be touched by a robot, while the robot is partially or fully inside the why a
opponent defense area */

void ref::AutoRef::AttackerDoubleTouchedBallInOpponentDefenseArea(
    std::vector<state::SoccerObject>& soccer_objects, Game& g) {
  float half_width = (vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f) / 1000.0f;
  float half_height = (vis::SoccerField::GetInstance().playing_area_height_mm / 2.0f) / 1000.0f;
  float half_penalty_width =
      (vis::SoccerField::GetInstance().penalty_area_width_mm / 2.0f) / 1000.0f;
  float half_penalty_height =
      (vis::SoccerField::GetInstance().penalty_area_height_mm / 2.0f) / 1000.0f;
  float radius = cfg::SystemConfig::robot_size_m[0] / 2.0;

  for (int i = soccer_objects.size() / 2; i < soccer_objects.size() - 1; i++) {
    float a_right = soccer_objects[i].position[0] + radius;
    float a_left = soccer_objects[i].position[0] - radius;
    float a_bottom = soccer_objects[i].position[1] - radius;
    float a_top = soccer_objects[i].position[1] + radius;

    float b_left = -half_width;
    float b_right = -half_width + half_penalty_width;
    float b_top = half_penalty_height;
    float b_bottom = -half_penalty_height;

    // collision x_axis
    bool collisionX = a_right >= b_left && b_right >= a_left;
    // collision y-axis?
    bool collisionY = a_top >= b_bottom && b_top >= a_bottom;
    // collision only if on both axes
    bool collides = collisionX && collisionY;
    if (soccer_objects[i].name != "ball") {
      if (collides && soccer_objects[i].is_attached && !g.was_ball_prev_attached) {
        team_two_foul_counter++;
        std::cout << "[ref::DefenderInDefenseArea] team two defender illegaly touched ball in "
                     "team one's "
                     "penalty box. foul count: "
                  << team_two_foul_counter << std::endl;
      }
    }
  }

  for (int i = 0; i < soccer_objects.size() / 2; i++) {
    float a_right = soccer_objects[i].position[0] + radius;
    float a_left = soccer_objects[i].position[0] - radius;
    float a_bottom = soccer_objects[i].position[1] - radius;
    float a_top = soccer_objects[i].position[1] + radius;

    float b_left = half_width - half_penalty_width;
    float b_right = half_width;
    float b_top = half_penalty_height;
    float b_bottom = -half_penalty_height;
    // collision x_axis
    bool collisionX = a_right >= b_left && b_right >= a_left;
    // collision y-axis?
    bool collisionY = a_top >= b_bottom && b_top >= a_bottom;
    // collision only if on both axes
    bool collides = collisionX && collisionY;

    if (soccer_objects[i].name != "ball") {
      if (collides && soccer_objects[i].is_attached && !g.was_ball_prev_attached) {
        team_one_foul_counter++;
        std::cout << "[ref::DefenderInDefenseArea] team one defender illegaly touched ball in "
                     "team two's "
                     "penalty box. foul count: "
                  << team_one_foul_counter << std::endl;
      }
    }
  }
}

// #10 BOT KICKED BALL TOO FAST
/* A robot must not accelerate the ball faster than 6.5 meters per second in 3D space. */

void ref::AutoRef::BotKickedBallTooFast(std::vector<state::SoccerObject>& soccer_objects,
                                        Game& g) {
  if (soccer_objects[soccer_objects.size() - 1].velocity.norm() > 6.5) {
    if (g.last_bot_touched_ball.team_id == 1) {
      std::cout << "[ref::AutoRef::BotKickedBallTooFast] team one's robot kicked the ball too "
                   "fast. foul count: "
                << team_one_foul_counter << std::endl;
      team_one_foul_counter++;
    } else if (g.last_bot_touched_ball.team_id == 2) {
      team_two_foul_counter++;
      std::cout << "[ref::AutoRef::BotKickedBallTooFast] team one's robot kicked the ball too "
                   "fast. foul count: "
                << team_two_foul_counter << std::endl;
    }
  }
}

// #11 BOT CRASH UNIQUE + #12 BOT CRASH DRAWN

/* At the moment of collision of two robots of different teams, the difference of the speed vectors
of both robots is taken and projected onto the line that is defined by the position of both robots.
If the length of this projection is greater than 1.5 meters per second, the faster robot committed
a foul. If the absolute robot speed difference is less than 0.3 meters per second, both conduct a
foul.*/

void ref::AutoRef::BotCrashUnique(std::vector<state::SoccerObject>& soccer_objects, Game& g) {
  for (int i = 0; i < soccer_objects.size(); ++i) {
    for (int j = i + 1; j < soccer_objects.size(); ++j) {
      state::SoccerObject& obj1 = soccer_objects[i];
      state::SoccerObject& obj2 = soccer_objects[j];

      if (kin::CheckCircularCollision(obj1, obj2)) {
        if (!(obj1.name == "ball" || obj2.name == "ball")) {
          Eigen::Vector3d pos1 = obj1.position;
          Eigen::Vector3d pos2 = obj2.position;
          Eigen::Vector3d pos_line = obj1.position - obj2.position;
          Eigen::Vector3d pos_line_unit = pos_line / pos_line.norm();

          Eigen::Vector3d vel1 = obj1.velocity;
          Eigen::Vector3d vel2 = obj2.velocity;
          Eigen::Vector3d vel_line = obj1.velocity - obj2.velocity;

          double dot_product = vel_line.dot(pos_line_unit);
          Eigen::Vector3d projection = dot_product * pos_line_unit;

          if (projection.norm() > 1.5) {
            if (std::abs(vel1.norm() - vel2.norm()) < 0.3) {
              // both robots commit a foul
              if (obj1.team_id == 1) {
                team_one_foul_counter++;
                std::cout << "[ref::AutoRef::BotCrashUnique] *YELLOW* (foul counter: "
                          << team_one_foul_counter << ") and ";
              } else if (obj1.team_id == 2) {
                team_two_foul_counter++;
                std::cout << "[ref::AutoRef::BotCrashUnique] *RED* (foul counter: "
                          << team_two_foul_counter << ") and ";
              }

              if (obj2.team_id == 1) {
                team_one_foul_counter++;
                std::cout << "*YELLOW* (foul counter: " << team_one_foul_counter << ") crashed."
                          << std::endl;
              }
              if (obj2.team_id == 2) {
                team_two_foul_counter++;
                std::cout << "*RED* (foul counter: " << team_two_foul_counter << ") crashed."
                          << std::endl;
              }
            } else {
              if (vel1.norm() > vel2.norm()) {
                if (obj1.team_id == 1) {
                  team_one_foul_counter++;
                  std::cout << "[ref::AutoRef::BotCrashUnique] *YELLOW* crashed. foul counter: "
                            << team_one_foul_counter << std::endl;
                } else if (obj1.team_id == 2) {
                  team_two_foul_counter++;
                  std::cout << "[ref::AutoRef::BotCrashUnique] *RED* crashed. foul counter: "
                            << team_two_foul_counter << std::endl;
                }
              } else if (vel2.norm() > vel1.norm()) {
                if (obj2.team_id == 1) {
                  team_one_foul_counter++;
                  std::cout << "[ref::AutoRef::BotCrashUnique] *YELLOW* crashed. foul counter: "
                            << team_one_foul_counter << std::endl;
                } else if (obj2.team_id == 2) {
                  team_two_foul_counter++;
                  std::cout << "[ref::AutoRef::BotCrashUnique] *RED* crashed. foul counter: "
                            << team_two_foul_counter << std::endl;
                }
              }
            }
          }
        }
      }
    }
  }
}

// #13 BOT TOO FAST IN STOP

/* A robot must not move faster than 1.5 meters per second during stop. A violation of this rule is
only counted once per robot and stoppage. There is a grace period of 2 seconds for the robots to
slow down */

void ref::AutoRef::BotTooFastInStop(std::vector<state::SoccerObject>& soccer_objects, Game& g) {
  if (g.state == ref::Game::Stop) {
    for (int i = 0; i < soccer_objects.size() - 1; i++) {
      if (soccer_objects[i].velocity.norm() > 1.2 &&
          !soccer_objects[i].was_given_speeding_foul_in_stop) {
        if (soccer_objects[i].team_id == 1) {
          team_one_foul_counter++;
          std::cout << "*YELLOW* moving too fast in stop. foul count: " << team_one_foul_counter
                    << std::endl;
        } else if (soccer_objects[i].team_id == 2) {
          team_two_foul_counter++;
          std::cout << "*RED* moving too fast in stop. foul count: " << team_two_foul_counter
                    << std::endl;
        }
        soccer_objects[i].was_given_speeding_foul_in_stop = true;
      }
    }
  }
}
