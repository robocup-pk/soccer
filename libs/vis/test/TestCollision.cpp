#include <gtest/gtest.h>

#include "GameObject.h"
#include "GLWindow.h"
#include "Collision.h"

TEST(CollisionTest, NoCollision) {
  vis::GameObject obj1("obj1", glm::vec2(1, 1), glm::vec2(5, 5));
  vis::GameObject obj2("obj2", glm::vec2(10, 10), glm::vec2(5, 5));
  EXPECT_FALSE(CheckCircularCollision(obj1, obj2));
}

TEST(CollisionTest, JustTouching) {
  // --->x
  // |
  // y
  vis::GameObject obj1("obj1", glm::vec2(0, 0), glm::vec2(4, 4));
  vis::GameObject obj2("obj2", glm::vec2(4, 0), glm::vec2(4, 4));
  // Center distance = 4, radii = 2 + 2
  EXPECT_TRUE(CheckCircularCollision(obj1, obj2));
}

TEST(CollisionTest, Overlapping) {
  vis::GameObject obj1("obj1", glm::vec2(0, 0), glm::vec2(4, 4));
  vis::GameObject obj2("obj2", glm::vec2(2, 0),
                       glm::vec2(4, 4));  // Center distance = 2, radii = 2 + 2
  EXPECT_TRUE(CheckCircularCollision(obj1, obj2));
}

TEST(ResolveCircularCollisionTest, SymmetricCollision_EqualMass) {
  // Two equal-sized objects, moving directly toward each other with equal speed
  vis::GameObject obj1("obj1", glm::vec2(0.0f, 0.0f), glm::vec2(4.0f, 4.0f),
                       glm::vec2(2.0f, 0.0f));
  vis::GameObject obj2("obj2", glm::vec2(3.0f, 0.0f), glm::vec2(4.0f, 4.0f),
                       glm::vec2(-2.0f, 0.0f));

  EXPECT_TRUE(vis::CheckCircularCollision(obj1, obj2));

  int mass1 = 10;
  int mass2 = 10;

  vis::ResolveCircularCollision(obj1, obj2, mass1, mass2);

  // Because masses are equal, they should swap velocities after collision
  EXPECT_NEAR(obj1.velocity.x, -2.0f, 0.01f);
  EXPECT_NEAR(obj2.velocity.x, 2.0f, 0.01f);

  // Y velocities should remain unchanged (no vertical component)
  EXPECT_NEAR(obj1.velocity.y, 0.0f, 0.01f);
  EXPECT_NEAR(obj2.velocity.y, 0.0f, 0.01f);
}

TEST(ResolveCircularCollisionTest, AsymmetricMasses) {
  vis::GameObject obj1("light", glm::vec2(0.0f, 0.0f), glm::vec2(4.0f, 4.0f), glm::vec2(5.0f, 0.0f));   // Light (mass 1)
  vis::GameObject obj2("heavy", glm::vec2(2.0f, 0.0f), glm::vec2(4.0f, 4.0f), glm::vec2(0.0f, 0.0f));   // Heavy (mass 100)

  ASSERT_TRUE(vis::CheckCircularCollision(obj1, obj2));

  vis::ResolveCircularCollision(obj1, obj2);

  // Lighter object should bounce back
  EXPECT_LT(obj1.velocity.x, 0.0f);
  // Heavier object should barely move
  EXPECT_NEAR(obj2.velocity.x, 0.0f, 0.5f);
}

TEST(ResolveCircularCollisionTest, OverlapResolved) {
  vis::GameObject obj1("obj1", glm::vec2(0.0f, 0.0f), glm::vec2(4.0f, 4.0f));
  vis::GameObject obj2("obj2", glm::vec2(1.0f, 0.0f), glm::vec2(4.0f, 4.0f));

  ASSERT_TRUE(vis::CheckCircularCollision(obj1, obj2));
  vis::ResolveCircularCollision(obj1, obj2);

  float radius = 2.0f;
  float centerDist = glm::distance(obj1.GetCenterPosition(), obj2.GetCenterPosition());

  // Ensure objects are no longer overlapping
  EXPECT_GE(centerDist, 2 * radius - 0.01f);
}