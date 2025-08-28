#include <gtest/gtest.h>

#include "SoccerObject.h"
#include "Kinematics.h"
#include "SoccerField.h"

// Position, Velocity, Acceleration, Size, Mass, Texture

TEST(CollisionTest, TestNoCollision) {
  state::SoccerObject obj1("obj1", Eigen::Vector3d(1, 1, 0), Eigen::Vector2d(5, 5), 10);
  state::SoccerObject obj2("obj2", Eigen::Vector3d(10, 10, 0), Eigen::Vector2d(5, 5), 10);
  EXPECT_FALSE(kin::CheckCircularCollision(obj1, obj2));
}

TEST(CollisionTest, TestJustTouching) {
  state::SoccerObject obj1("obj1", Eigen::Vector3d(0, 0, 0), Eigen::Vector2d(4, 4), 10);
  state::SoccerObject obj2("obj2", Eigen::Vector3d(4, 0, 0), Eigen::Vector2d(4, 4), 10);
  // Center distance = 4, radii = 2 + 2
  EXPECT_TRUE(kin::CheckCircularCollision(obj1, obj2));
}

TEST(CollisionTest, TestOverlapping) {
  state::SoccerObject obj1("obj1", Eigen::Vector3d(0, 0, 0), Eigen::Vector2d(4, 4), 10);
  state::SoccerObject obj2("obj2", Eigen::Vector3d(2, 0, 0), Eigen::Vector2d(4, 4), 10);
  EXPECT_TRUE(kin::CheckCircularCollision(obj1, obj2));
}

TEST(ResolveCircularCollisionTest, TestSymmetricCollisionEqualMass) {
  // Two equal-sized objects, moving directly toward each other with equal speed
  state::SoccerObject obj1("obj1", Eigen::Vector3d(0.0f, 0.0f, 0), Eigen::Vector2d(4.0f, 4.0f), 10,
                           Eigen::Vector3d(2.0f, 0.0f, 0));
  state::SoccerObject obj2("obj2", Eigen::Vector3d(3.0f, 0.0f, 0), Eigen::Vector2d(4.0f, 4.0f), 10,
                           Eigen::Vector3d(-2.0f, 0.0f, 0));

  EXPECT_TRUE(kin::CheckCircularCollision(obj1, obj2));

  kin::ResolveCircularCollision(obj1, obj2);

  // Because masses are equal, they should swap velocities after collision
  EXPECT_NEAR(obj1.velocity[0], -2.0f, 0.01f);
  EXPECT_NEAR(obj2.velocity[0], 2.0f, 0.01f);

  // Y velocities should remain unchanged (no vertical component)
  EXPECT_NEAR(obj1.velocity[1], 0.0f, 0.01f);
  EXPECT_NEAR(obj2.velocity[1], 0.0f, 0.01f);
}

TEST(ResolveCircularCollisionTest, TestAsymmetricMasses) {
  state::SoccerObject obj1("light", Eigen::Vector3d(0.0f, 0.0f, 0), Eigen::Vector2d(4.0f, 4.0f),
                           10, Eigen::Vector3d(5.0f, 0.0f, 0), Eigen::Vector3d(0, 0, 0),
                           1);  // Light (mass 1)
  state::SoccerObject obj2("heavy", Eigen::Vector3d(2.0f, 0.0f, 0), Eigen::Vector2d(4.0f, 4.0f),
                           10, Eigen::Vector3d(0.0f, 0.0f, 0), Eigen::Vector3d(0, 0, 0),
                           100);  // Heavy (mass 100)

  ASSERT_TRUE(kin::CheckCircularCollision(obj1, obj2));

  kin::ResolveCircularCollision(obj1, obj2);

  // Lighter object should bounce back
  EXPECT_LT(obj1.velocity[0], 0.0f);
  // Heavier object should barely move
  EXPECT_NEAR(obj2.velocity[0], 0.0f, 0.5f);
}

TEST(ResolveCircularCollisionTest, TestOverlapResolved) {
  state::SoccerObject obj1("obj1", Eigen::Vector3d(0.0f, 0.0f, 0), Eigen::Vector2d(4.0f, 4.0f),
                           10);
  state::SoccerObject obj2("obj2", Eigen::Vector3d(1.0f, 0.0f, 0), Eigen::Vector2d(4.0f, 4.0f),
                           10);

  ASSERT_TRUE(kin::CheckCircularCollision(obj1, obj2));
  kin::ResolveCircularCollision(obj1, obj2);

  float radius = 2.0f;
  float centerDist = (obj1.GetCenterPosition() - obj2.GetCenterPosition()).norm();

  // Ensure objects are no longer overlapping
  EXPECT_GE(centerDist, 2 * radius - 0.01f);
}

TEST(CollisionTest, TestResolveCollisionWithWall) {
  double left = vis::SoccerField::GetInstance().width_mm / -2000.0f;
  double right = vis::SoccerField::GetInstance().width_mm / 2000.0f;
  double top = vis::SoccerField::GetInstance().height_mm / 2000.0f;
  double bottom = vis::SoccerField::GetInstance().height_mm / -2000.0f;

  std::vector<state::SoccerObject> soccer_objects;
  state::SoccerObject obj1("obj1");
  soccer_objects.push_back(obj1);

  state::SoccerObject& obj = soccer_objects[0];
  Eigen::Vector3d pose = obj.position;

  // Position should remain unchanged (within tolerance)
  kin::ResolveCollisionWithWall(soccer_objects);
  EXPECT_TRUE(obj.position.isApprox(pose, 1e-6));

  // Left wall
  obj.position[0] = left - 0.1f;
  kin::ResolveCollisionWithWall(soccer_objects);
  EXPECT_GE(obj.position[0], left);

  // Right wall
  obj.position[0] = right + 0.1f;
  kin::ResolveCollisionWithWall(soccer_objects);
  EXPECT_LE(obj.position[0], right);

  // Top wall
  obj.position[1] = top + 0.1f;
  kin::ResolveCollisionWithWall(soccer_objects);
  EXPECT_LE(obj.position[1], top);

  // Bottom wall
  obj.position[1] = bottom - 0.1f;
  kin::ResolveCollisionWithWall(soccer_objects);
  EXPECT_GE(obj.position[1], bottom);

  // Top-left corner
  obj.position[0] = left - 0.1f;
  obj.position[1] = top + 0.1f;
  kin::ResolveCollisionWithWall(soccer_objects);
  EXPECT_GE(obj.position[0], left);
  EXPECT_LE(obj.position[1], top);

  // Top-right corner
  obj.position[0] = right + 0.1f;
  obj.position[1] = top + 0.1f;
  kin::ResolveCollisionWithWall(soccer_objects);
  EXPECT_LE(obj.position[0], right);
  EXPECT_LE(obj.position[1], top);

  // Bottom-left corner
  obj.position[0] = left - 0.1f;
  obj.position[1] = bottom - 0.1f;
  kin::ResolveCollisionWithWall(soccer_objects);
  EXPECT_GE(obj.position[0], left);
  EXPECT_GE(obj.position[1], bottom);

  // Bottom-right corner
  obj.position[0] = right + 0.1f;
  obj.position[1] = bottom - 0.1f;
  kin::ResolveCollisionWithWall(soccer_objects);
  EXPECT_LE(obj.position[0], right);
  EXPECT_GE(obj.position[1], bottom);
}

TEST(BoundaryTest, TestIsInsideBoundary) {
  state::SoccerObject obj("robot");
  obj.position = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
  EXPECT_TRUE(kin::IsInsideBoundary(obj));

  auto half_width = (vis::SoccerField::GetInstance().width_mm / 2) / 1000.0f;
  auto half_height = (vis::SoccerField::GetInstance().height_mm / 2) / 1000.0f;

  obj.position[0] = -half_width - 0.2f;
  EXPECT_FALSE(kin::IsInsideBoundary(obj));

  obj.position[0] = half_width + 0.2f;
  EXPECT_FALSE(kin::IsInsideBoundary(obj));

  obj.position[0] = 0;

  obj.position[1] = -half_height - 0.2f;
  EXPECT_FALSE(kin::IsInsideBoundary(obj));

  obj.position[1] = half_height + 0.2f;
  EXPECT_FALSE(kin::IsInsideBoundary(obj));
}

TEST(BoundaryTest, TestClampInsideBoundary) {
  state::SoccerObject obj("robot");
  obj.size = Eigen::Vector2d(0.2, 0.2);  // width & height (m)
  const double tol = 1e-6;

  double half_width = (vis::SoccerField::GetInstance().width_mm / 2.0) / 1000.0;
  double half_height = (vis::SoccerField::GetInstance().height_mm / 2.0) / 1000.0;

  double half_obj_w = obj.size[0] / 2.0;
  double half_obj_h = obj.size[1] / 2.0;

  double min_x = -half_width + half_obj_w;
  double max_x = half_width - half_obj_w;
  double min_y = -half_height + half_obj_h;
  double max_y = half_height - half_obj_h;

  // Place object beyond left boundary
  obj.position = Eigen::Vector3d(min_x - 1.0, 0.0, 0.0);
  kin::ClampInsideBoundary(obj);
  EXPECT_GE(obj.position[0], min_x - tol);
  EXPECT_LE(obj.position[0], max_x + tol);

  // Place object beyond right boundary
  obj.position = Eigen::Vector3d(max_x + 1.0, 0.0, 0.0);
  kin::ClampInsideBoundary(obj);
  EXPECT_LE(obj.position[0], max_x + tol);
  EXPECT_GE(obj.position[0], min_x - tol);

  // Place object beyond bottom boundary
  obj.position = Eigen::Vector3d(0.0, min_y - 1.0, 0.0);
  kin::ClampInsideBoundary(obj);
  EXPECT_GE(obj.position[1], min_y - tol);
  EXPECT_LE(obj.position[1], max_y + tol);

  // Place object beyond top boundary
  obj.position = Eigen::Vector3d(0.0, max_y + 1.0, 0.0);
  kin::ClampInsideBoundary(obj);
  EXPECT_LE(obj.position[1], max_y + tol);
  EXPECT_GE(obj.position[1], min_y - tol);

  // Place object well inside boundary (should remain unchanged)
  obj.position = Eigen::Vector3d(0.0, 0.0, 0.0);
  Eigen::Vector3d prev_position = obj.position;
  kin::ClampInsideBoundary(obj);
  EXPECT_TRUE(obj.position.isApprox(prev_position, 1e-9));
}

TEST(BallHandlingTest, TestIsBallInFrontOfRobot) {
  state::SoccerObject robot("robot");
  robot.size = Eigen::Vector2d(0.3, 0.3);
  robot.position = Eigen::Vector3d(0.0, 0.0, 0.0);  // facing +x

  state::SoccerObject ball("ball");
  ball.size = Eigen::Vector2d(0.2, 0.2);

  // Place ball directly in front of robot
  ball.position = Eigen::Vector3d(1.0, 0.0, 0.0);
  EXPECT_TRUE(kin::IsBallInFrontOfRobot(robot, ball));

  // Place ball behind robot
  ball.position = Eigen::Vector3d(-1.0, 0.0, 0.0);
  EXPECT_FALSE(kin::IsBallInFrontOfRobot(robot, ball));

  // Rotate robot 90 degrees and put ball accordingly
  robot.position[2] = M_PI / 2.0;  // facing +y
  ball.position = Eigen::Vector3d(0.0, 1.0, 0.0);
  EXPECT_TRUE(kin::IsBallInFrontOfRobot(robot, ball));
}

TEST(BallHandlingTest, TestHandleBallStickingAndUpdate) {
  state::SoccerObject robot("robot");
  robot.size = Eigen::Vector2d(0.4, 0.4);
  robot.position = Eigen::Vector3d(0.0, 0.0, 0.0);  // facing +x

  state::SoccerObject ball("ball");
  ball.size = Eigen::Vector2d(0.2, 0.2);
  ball.position = Eigen::Vector3d(1.0, 0.0, 0.0);

  kin::HandleBallSticking(robot, ball);

  // Attachment state
  EXPECT_TRUE(robot.is_attached);
  EXPECT_TRUE(ball.is_attached);
  EXPECT_EQ(ball.attached_to, &robot);
  EXPECT_EQ(robot.attached_to, &ball);

  // Ball should be repositioned to front of robot
  Eigen::Vector3d robot_center = robot.GetCenterPosition();
  float expected_x = robot_center.x() +
                     (std::min(robot.size[0], robot.size[1]) / 2.0f * 0.5f + ball.size[0] / 2.0f);
  EXPECT_NEAR(ball.position.x(), expected_x, 1e-6);
  EXPECT_NEAR(ball.position.y(), 0.0, 1e-6);

  // Ball velocity reset
  EXPECT_TRUE(ball.velocity.isApprox(Eigen::Vector3d(0, 0, 0), 1e-9));
}

TEST(BallHandlingTest, TestDetachBall) {
  state::SoccerObject robot("robot");
  robot.size = Eigen::Vector2d(0.4, 0.4);
  robot.position = Eigen::Vector3d(0.0, 0.0, 0.0);  // facing +x

  state::SoccerObject ball("ball");
  ball.size = Eigen::Vector2d(0.2, 0.2);

  // Attach ball first
  kin::HandleBallSticking(robot, ball);
  ASSERT_TRUE(ball.is_attached);

  // Detach with some velocity
  float detach_vel = 1.5f;
  kin::DetachBall(ball, detach_vel);

  // Attachment state cleared
  EXPECT_FALSE(ball.is_attached);
  EXPECT_FALSE(robot.is_attached);
  EXPECT_EQ(ball.attached_to, nullptr);
  EXPECT_EQ(robot.attached_to, nullptr);

  // Ball should have nonzero velocity along +x
  EXPECT_NEAR(ball.velocity[0], detach_vel, 1e-6);
  EXPECT_NEAR(ball.velocity[1], 0.0, 1e-6);

  // Ball should be positioned in front of robot
  EXPECT_GT(ball.position[0], robot.GetCenterPosition().x());
}