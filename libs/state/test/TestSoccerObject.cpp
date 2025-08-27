#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "SoccerObject.h"
#include "SystemConfig.h"
#include "RobotPositions.h"
#include "RobotManager.h"

class SoccerObjectTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a basic soccer object
    Eigen::Vector3d position(1.0, 2.0, 0.5);
    Eigen::Vector2d size(0.5, 0.5);
    soccer_object = std::make_unique<state::SoccerObject>("test_robot", position, size, 1);
  }

  void TearDown() override { soccer_object.reset(); }

  std::unique_ptr<state::SoccerObject> soccer_object;
};

TEST_F(SoccerObjectTest, TestSoccerObjectCreation) {
  Eigen::Vector3d pos(1.0, 2.0, 0.5);
  Eigen::Vector2d sz(0.5, 0.5);
  state::SoccerObject obj("robot", pos, sz, 1);

  EXPECT_EQ(obj.name, "robot");
  EXPECT_EQ(obj.team_id, 1);
  EXPECT_NEAR(obj.position[0], 1.0, 1e-6);
  EXPECT_NEAR(obj.position[1], 2.0, 1e-6);
  EXPECT_NEAR(obj.position[2], 0.5, 1e-6);
  EXPECT_NEAR(obj.size[0], 0.5, 1e-6);
  EXPECT_NEAR(obj.size[1], 0.5, 1e-6);
}

TEST_F(SoccerObjectTest, TestSoccerObjectInitializationUsingRobotManager) {
  rob::RobotManager robot_manager;
  Eigen::Vector3d vel(0.1, 0.2, 0.0);
  robot_manager.SetBodyVelocity(vel);
  Eigen::Vector3d pose = robot_manager.GetPoseInWorldFrame();
  state::SoccerObject obj(robot_manager);

  EXPECT_EQ(obj.position, pose);
  EXPECT_EQ(obj.velocity, vel);
}

TEST_F(SoccerObjectTest, TestGetPositionAndCenter) {
  Eigen::Vector3d pos = soccer_object->GetPosition();
  EXPECT_NEAR(pos[0], 1.0, 1e-6);
  EXPECT_NEAR(pos[1], 2.0, 1e-6);
  EXPECT_NEAR(pos[2], 0.5, 1e-6);

  Eigen::Vector3d center = soccer_object->GetCenterPosition();
  EXPECT_NEAR(center[0], 1.0, 1e-6);
  EXPECT_NEAR(center[1], 2.0, 1e-6);
}

TEST_F(SoccerObjectTest, TestSetRobotRole) {
  soccer_object->SetRobotRole(state::SoccerObject::Role::Kicker);
  EXPECT_EQ(soccer_object->role, state::SoccerObject::Role::Kicker);

  soccer_object->SetRobotRole(state::SoccerObject::Role::Keeper);
  EXPECT_EQ(soccer_object->role, state::SoccerObject::Role::Keeper);
}

TEST_F(SoccerObjectTest, TestIsPointInFrontSector) {
  // Test point in front
  Eigen::Vector2d point_front(1.5, 2.0);
  EXPECT_TRUE(soccer_object->IsPointInFrontSector(point_front));

  // Test point behind
  Eigen::Vector2d point_behind(0.5, 2.0);
  EXPECT_FALSE(soccer_object->IsPointInFrontSector(point_behind));

  // Test point at side
  Eigen::Vector2d point_side(1.0, 2.5);
  EXPECT_FALSE(soccer_object->IsPointInFrontSector(point_side));
}

TEST_F(SoccerObjectTest, TestMove) {
  // Set initial velocity
  Eigen::Vector3d vel = Eigen::Vector3d(0.1, 0.2, 0.0);
  soccer_object->velocity = vel;
  Eigen::Vector3d initial_pos = soccer_object->position;

  soccer_object->Move(1.0);  // Move for 1 second

  double movement = vel.norm();

  EXPECT_LT(soccer_object->position.norm(), (initial_pos).norm() + movement);
  EXPECT_GT(soccer_object->position.norm(), initial_pos.norm());
}

TEST_F(SoccerObjectTest, TestMassAndSize) {
  EXPECT_NEAR(soccer_object->mass_kg, 1.0, 1e-6);
  EXPECT_NEAR(soccer_object->size[0], 0.5, 1e-6);
  EXPECT_NEAR(soccer_object->size[1], 0.5, 1e-6);
}

TEST(SoccerObjectStaticTest, TestInitSoccerObjects) {
  std::vector<state::SoccerObject> soccer_objects;
  EXPECT_EQ(soccer_objects.size(), 0);

  state::InitSoccerObjects(soccer_objects);
  EXPECT_GT(soccer_objects.size(), 0);

  state::SoccerObject& obj = soccer_objects[0];
  EXPECT_EQ(obj.name, "robot0");

  state::SoccerObject& ball = soccer_objects[soccer_objects.size() - 1];
  EXPECT_EQ(ball.name, "ball");

  EXPECT_EQ(obj.size, cfg::SystemConfig::robot_size_m);
  EXPECT_EQ(ball.radius_m, cfg::SystemConfig::ball_radius_m);

  EXPECT_EQ(obj.position,
            cfg::LeftRobotHomeCoordinates.at(static_cast<cfg::RobotHomePosition>(0)));
  EXPECT_EQ(ball.position, cfg::SystemConfig::init_ball_position);

  EXPECT_EQ(obj.velocity, cfg::SystemConfig::init_robot_velocity_mps);
  EXPECT_EQ(ball.velocity, cfg::SystemConfig::init_ball_velocity_mps);

  EXPECT_EQ(obj.acceleration, cfg::SystemConfig::init_robot_acceleration_mpsps);
  EXPECT_EQ(ball.acceleration, cfg::SystemConfig::init_ball_acceleration_mpsps);

  EXPECT_EQ(obj.mass_kg, cfg::SystemConfig::robot_mass_kg);
  EXPECT_EQ(ball.mass_kg, cfg::SystemConfig::ball_mass_kg);
}