#include <gtest/gtest.h>

#include "GameObject.h"
#include "GLWindow.h"
#include "Collision.h"

// Position, Velocity, Acceleration, Size, Mass, Texture

TEST(CollisionTest, NoCollision) {
  vis::GameObject obj1("obj1", Eigen::Vector3d(1, 1, 0), Eigen::Vector2d(5, 5));
  vis::GameObject obj2("obj2", Eigen::Vector3d(10, 10, 0), Eigen::Vector2d(5, 5));
  EXPECT_FALSE(CheckCircularCollision(obj1, obj2));
}

TEST(CollisionTest, JustTouching) {
  // --->x
  // |
  // y
  vis::GameObject obj1("obj1", Eigen::Vector3d(0, 0, 0), Eigen::Vector2d(4, 4));
  vis::GameObject obj2("obj2", Eigen::Vector3d(4, 0, 0), Eigen::Vector2d(4, 4));
  // Center distance = 4, radii = 2 + 2
  EXPECT_TRUE(CheckCircularCollision(obj1, obj2));
}

TEST(CollisionTest, Overlapping) {
  vis::GameObject obj1("obj1", Eigen::Vector3d(0, 0, 0), Eigen::Vector2d(4, 4));
  vis::GameObject obj2("obj2", Eigen::Vector3d(2, 0, 0), Eigen::Vector2d(4, 4));
  EXPECT_TRUE(CheckCircularCollision(obj1, obj2));
}

TEST(ResolveCircularCollisionTest, SymmetricCollision_EqualMass) {
  // Two equal-sized objects, moving directly toward each other with equal speed
  vis::GameObject obj1("obj1", Eigen::Vector3d(0.0f, 0.0f, 0), Eigen::Vector2d(4.0f, 4.0f),
                       Eigen::Vector3d(2.0f, 0.0f, 0));
  vis::GameObject obj2("obj2", Eigen::Vector3d(3.0f, 0.0f, 0), Eigen::Vector2d(4.0f, 4.0f),
                       Eigen::Vector3d(-2.0f, 0.0f, 0));

  EXPECT_TRUE(vis::CheckCircularCollision(obj1, obj2));

  vis::ResolveCircularCollision(obj1, obj2);

  // Because masses are equal, they should swap velocities after collision
  EXPECT_NEAR(obj1.velocity.x, -2.0f, 0.01f);
  EXPECT_NEAR(obj2.velocity.x, 2.0f, 0.01f);

  // Y velocities should remain unchanged (no vertical component)
  EXPECT_NEAR(obj1.velocity.y, 0.0f, 0.01f);
  EXPECT_NEAR(obj2.velocity.y, 0.0f, 0.01f);
}

TEST(ResolveCircularCollisionTest, AsymmetricMasses) {
  vis::GameObject obj1("light", Eigen::Vector3d(0.0f, 0.0f, 0), Eigen::Vector2d(4.0f, 4.0f),
                       Eigen::Vector3d(5.0f, 0.0f, 0), Eigen::Vector3d(0, 0),
                       1);  // Light (mass 1)
  vis::GameObject obj2("heavy", Eigen::Vector3d(2.0f, 0.0f, 0), Eigen::Vector2d(4.0f, 4.0f),
                       Eigen::Vector3d(0.0f, 0.0f,0), Eigen::Vector3d(0, 0, 0),
                       100);  // Heavy (mass 100)

  ASSERT_TRUE(vis::CheckCircularCollision(obj1, obj2));

  vis::ResolveCircularCollision(obj1, obj2);

  // Lighter object should bounce back
  EXPECT_LT(obj1.velocity.x, 0.0f);
  // Heavier object should barely move
  EXPECT_NEAR(obj2.velocity.x, 0.0f, 0.5f);
}

TEST(ResolveCircularCollisionTest, OverlapResolved) {
  vis::GameObject obj1("obj1", Eigen::Vector3d(0.0f, 0.0f,0), Eigen::Vector2d(4.0f, 4.0f));
  vis::GameObject obj2("obj2", Eigen::Vector3d(1.0f, 0.0f,0), Eigen::Vector2d(4.0f, 4.0f));

  ASSERT_TRUE(vis::CheckCircularCollision(obj1, obj2));
  vis::ResolveCircularCollision(obj1, obj2);

  float radius = 2.0f;
  float centerDist = glm::distance(obj1.GetCenterPosition(), obj2.GetCenterPosition());

  // Ensure objects are no longer overlapping
  EXPECT_GE(centerDist, 2 * radius - 0.01f);
}