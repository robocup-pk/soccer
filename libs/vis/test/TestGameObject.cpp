#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <glm/glm.hpp>
#include "GameObject.h"
#include "SystemConfig.h"
#include "Coordinates.h"
#include "Texture.h"

class GameObjectTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a basic soccer object for testing
    Eigen::Vector3d position(1.0, 2.0, 0.5);
    Eigen::Vector2d size(0.5, 0.5);
    soccer_object = std::make_unique<state::SoccerObject>("test_robot", position, size, 1);

    // Create a basic game object
    game_object = std::make_unique<vis::GameObject>("test_game_obj", position, size);
  }

  void TearDown() override {
    soccer_object.reset();
    game_object.reset();
  }

  std::unique_ptr<state::SoccerObject> soccer_object;
  std::unique_ptr<vis::GameObject> game_object;
};

TEST_F(GameObjectTest, TestConvertEigenVecToGlm) {
  Eigen::Vector2d eigen_2d(1.5, -2.3);
  glm::vec2 glm_2d = vis::ConvertEigenVecToGlm(eigen_2d);
  EXPECT_NEAR(glm_2d.x, 1.5f, 1e-6f);
  EXPECT_NEAR(glm_2d.y, -2.3f, 1e-6f);

  Eigen::Vector3d eigen_3d(1.0, 2.0, 3.0);
  glm::vec3 glm_3d = vis::ConvertEigenVecToGlm(eigen_3d);
  EXPECT_NEAR(glm_3d.x, 1.0f, 1e-6f);
  EXPECT_NEAR(glm_3d.y, 2.0f, 1e-6f);
  EXPECT_NEAR(glm_3d.z, 3.0f, 1e-6f);
}

TEST_F(GameObjectTest, TestGameObjectConstructor) {
  Eigen::Vector3d pos(1.0, 2.0, 0.5);
  Eigen::Vector2d sz(0.5, 0.5);
  Eigen::Vector3d vel(0.1, 0.2, 0.0);
  Eigen::Vector3d acc(0.0, 0.0, 0.0);
  Eigen::Vector3d color(1.0, 0.5, 0.0);

  vis::GameObject obj("TEST_F", pos, sz, vel, acc, 2.0f, vis::Texture2D(false), color);

  // Check unit conversion (meters to mm)
  EXPECT_NEAR(obj.position.x, 1000.0f, 1e-6f);  // 1.0 * 1000
  EXPECT_NEAR(obj.position.y, 2000.0f, 1e-6f);  // 2.0 * 1000
  EXPECT_NEAR(obj.position.z, 0.5, 1e-6f);      // Angle should stay same

  EXPECT_NEAR(obj.size.x, 500.0f, 1e-6f);
  EXPECT_NEAR(obj.size.y, 500.0f, 1e-6f);

  EXPECT_NEAR(obj.mass_kg, 2.0f, 1e-6f);
  EXPECT_NEAR(obj.radius, 250.0f, 1e-6f);
}

TEST_F(GameObjectTest, TestGetCenterPosition) {
  // Set known position and size
  game_object->position = glm::vec3(100.0f, 200.0f, 0.0f);
  game_object->size = glm::vec2(50.0f, 80.0f);

  glm::vec2 center = game_object->GetCenterPosition();

  EXPECT_NEAR(center.x, game_object->position.x + game_object->size.x / 2, 1e-6f);
  EXPECT_NEAR(center.y, game_object->position.y + game_object->size.y / 2, 1e-6f);
}

TEST_F(GameObjectTest, TestAssignmentFromSoccerObject) {
  // Create a SoccerObject with specific values
  Eigen::Vector3d pos(1.0, 2.0, 1.57);  // x, y in meters, angle in radians
  Eigen::Vector2d sz(0.5, 0.5);
  Eigen::Vector3d vel(0.1, -0.2, 0.5);
  Eigen::Vector3d acc(0.01, 0.02, -0.01);
  float mass = 3.0f;
  state::SoccerObject soccer_obj("test_assign", pos, sz, vel, acc, mass);

  // Create a GameObject and assign
  vis::GameObject game_obj("", Eigen::Vector3d::Zero(), Eigen::Vector2d::Zero(),
                           Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 1.0f,
                           vis::Texture2D(false), Eigen::Vector3d(1, 1, 1));
  game_obj = soccer_obj;

  // Expected conversions
  float px_per_m = cfg::Coordinates::px_per_m;                  // 200.0f
  Eigen::Vector3d m_px_coords = cfg::Coordinates::m_px_coords;  // (1, -1, 1)

  // Position: cwiseProduct with m_px_coords, then * px_per_m, z unchanged
  Eigen::Vector3d expected_pos_eigen = pos.cwiseProduct(m_px_coords) * px_per_m;
  EXPECT_NEAR(game_obj.position.x, expected_pos_eigen[0], 1e-3f);
  EXPECT_NEAR(game_obj.position.y, expected_pos_eigen[1], 1e-3f);
  EXPECT_NEAR(game_obj.position.z, pos[2], 1e-6f);  // angle unchanged

  // Velocity: same as position
  Eigen::Vector3d expected_vel_eigen = vel.cwiseProduct(m_px_coords) * px_per_m;
  EXPECT_NEAR(game_obj.velocity.x, expected_vel_eigen[0], 1e-3f);
  EXPECT_NEAR(game_obj.velocity.y, expected_vel_eigen[1], 1e-3f);
  EXPECT_NEAR(game_obj.velocity.z, expected_vel_eigen[2], 1e-3f);

  // Acceleration: same as position
  Eigen::Vector3d expected_acc_eigen = acc.cwiseProduct(m_px_coords) * px_per_m;
  EXPECT_NEAR(game_obj.acceleration.x, expected_acc_eigen[0], 1e-3f);
  EXPECT_NEAR(game_obj.acceleration.y, expected_acc_eigen[1], 1e-3f);
  EXPECT_NEAR(game_obj.acceleration.z, expected_acc_eigen[2], 1e-3f);

  // Size: size * px_per_m
  EXPECT_NEAR(game_obj.size.x, sz[0] * px_per_m, 1e-3f);
  EXPECT_NEAR(game_obj.size.y, sz[1] * px_per_m, 1e-3f);

  // Mass: direct copy
  EXPECT_NEAR(game_obj.mass_kg, mass, 1e-6f);

  // Radius: radius_m * px_per_m
  EXPECT_NEAR(game_obj.radius, soccer_obj.radius_m * px_per_m, 1e-3f);
}