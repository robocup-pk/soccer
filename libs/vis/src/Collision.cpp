#include <iostream>

#include "GLConfig.h"
#include "Collision.h"

void vis::CheckCollision(std::vector<GameObject>& game_objects) {
  // We can have collisions between: robot/robot and robot/ball

  // Let's just do collision between ball and wall:
  for (GameObject& g : game_objects) {
    if (g.name == "ball") {
      int ball_x = g.GetCenterPosition().x;
      int ball_y = g.GetCenterPosition().y;

      // x direction
      if (ball_x + g.size.x / 2 >= GLConfig::window_width_px / 2) {
        if (g.velocity.x > 0) {
          g.velocity.x *= -1;
        }
      }
      if (ball_x - g.size.x / 2 <= -GLConfig::window_width_px / 2) {
        std::cout << "Collision with left wall" << std::endl;
        if (g.velocity.x < 0) {
          g.velocity.x *= -1;
        }
      }

      // y direction
      if (ball_y + g.size.y / 2 >= GLConfig::window_height_px / 2) {
        if (g.velocity.y > 0) {
          g.velocity.y *= -1;
        }
      }
      if (ball_y - g.size.y / 2 <= -GLConfig::window_height_px / 2) {
        if (g.velocity.y < 0) {
          g.velocity.y *= -1;
        }
      }

      break;
    } else if (g.name == "robot0") {
      // std::cout << "robot0: " << g.GetCenterPosition().x << " " << g.GetCenterPosition().y
      //           << std::endl;
    }
  }

  // When ball and robot collide

  for (int i = 0; i < game_objects.size() - 1; ++i) {
    if (game_objects[i].name == "background") continue;
    for (int j = i + 1; j < game_objects.size(); ++j) {
      if (game_objects[j].name == "background") continue;

      // Positions of both the game objects
      glm::vec2 first_object_position = game_objects[i].GetCenterPosition();
      glm::vec2 second_object_position = game_objects[j].GetCenterPosition();

      std::cout << "Check: " << game_objects[i].name << "(" << first_object_position.x << ","
                << first_object_position.y << ")" << " and " << game_objects[j].name << "("
                << second_object_position.x << "," << second_object_position.y << ")" << std::endl;

      // Check if they collide
      bool collision_x = std::fabs(first_object_position.x - second_object_position.x) <
                         (game_objects[i].size.x / 2 + game_objects[j].size.x / 2);
      bool collision_y = std::fabs(first_object_position.y - second_object_position.y) <
                         (game_objects[i].size.y / 2 + game_objects[j].size.y / 2);
      if (collision_x && collision_y) {
        std::cout << "Collision between " << game_objects[i].name << " and "
                  << game_objects[j].name << std::endl;

        if (std::fabs(first_object_position.x - second_object_position.x) <
            std::fabs(first_object_position.y - second_object_position.y)) {
          game_objects[i].velocity.y *= -1;
          game_objects[j].velocity.y *= -1;
        } else {
          game_objects[i].velocity.x *= -1;
          game_objects[j].velocity.x *= -1;
        }
      }
    }
  }
}