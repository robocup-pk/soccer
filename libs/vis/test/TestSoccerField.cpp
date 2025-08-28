#include <gtest/gtest.h>
#include "SoccerField.h"
#include "Dimensions.h"

// Test singleton behavior
TEST(SoccerFieldTest, TestSoccerFieldSingleton) {
  vis::SoccerField& instance1 = vis::SoccerField::GetInstance();
  vis::SoccerField& instance2 = vis::SoccerField::GetInstance();

  EXPECT_EQ(&instance1, &instance2) << "GetInstance should return the same instance";
}

// Test aspect ratio calculation
TEST(SoccerFieldAspectRatioTest, TestSoccerFieldAspectRatio) {
  vis::SoccerField& field = vis::SoccerField::GetInstance();

  float expected_ratio =
      cfg::Dimensions::actual_field_width_mm / cfg::Dimensions::actual_field_height_mm;
  EXPECT_NEAR(field.width_height_ratio, expected_ratio, 1e-6f)
      << "Width-height ratio should match actual field ratio";
}

// Test field dimensions calculation
TEST(SoccerFieldTest, TestSoccerFieldDimensions) {
  vis::SoccerField& field = vis::SoccerField::GetInstance();

  // Check that dimensions are positive
  EXPECT_GT(field.width_mm, 0.0f) << "Width should be positive";
  EXPECT_GT(field.height_mm, 0.0f) << "Height should be positive";
  EXPECT_EQ(field.height_mm, cfg::Dimensions::height_mm) << "Height should match config height";

  // Check width calculation based on aspect ratio
  float expected_width = cfg::Dimensions::height_mm * field.width_height_ratio;
  EXPECT_NEAR(field.width_mm, expected_width, 1e-6f)
      << "Width should be height times aspect ratio";
}

// Test scaled dimensions
TEST(SoccerFieldTest, TestSoccerFieldScaledDimensions) {
  vis::SoccerField& field = vis::SoccerField::GetInstance();

  // Check actual_to_our_ratio
  float expected_ratio = cfg::Dimensions::actual_field_width_mm / field.width_mm;
  EXPECT_NEAR(field.actual_to_our_ratio, expected_ratio, 1e-6f)
      << "Actual to our ratio should be correct";

  // Check central circle radius scaling
  float expected_radius =
      cfg::Dimensions::actual_field_central_circle_radius_mm / field.actual_to_our_ratio;
  EXPECT_NEAR(field.central_circle_radius_mm, expected_radius, 1e-6f)
      << "Central circle radius should be scaled correctly";

  // Check playing area dimensions
  float expected_playing_width =
      cfg::Dimensions::actual_field_playing_area_width_mm / field.actual_to_our_ratio;
  EXPECT_NEAR(field.playing_area_width_mm, expected_playing_width, 1e-6f)
      << "Playing area width should be scaled correctly";

  float expected_playing_height =
      cfg::Dimensions::actual_field_playing_area_height_mm / field.actual_to_our_ratio;
  EXPECT_NEAR(field.playing_area_height_mm, expected_playing_height, 1e-6f)
      << "Playing area height should be scaled correctly";

  // Check penalty area dimensions
  float expected_penalty_width =
      cfg::Dimensions::actual_field_penalty_area_width_mm / field.actual_to_our_ratio;
  EXPECT_NEAR(field.penalty_area_width_mm, expected_penalty_width, 1e-6f)
      << "Penalty area width should be scaled correctly";

  float expected_penalty_height =
      cfg::Dimensions::actual_field_penalty_area_height_mm / field.actual_to_our_ratio;
  EXPECT_NEAR(field.penalty_area_height_mm, expected_penalty_height, 1e-6f)
      << "Penalty area height should be scaled correctly";

  // Check goal dimensions
  float expected_goal_width =
      cfg::Dimensions::actual_field_goal_width_mm / field.actual_to_our_ratio;
  EXPECT_NEAR(field.goal_width_mm, expected_goal_width, 1e-6f)
      << "Goal width should be scaled correctly";

  float expected_goal_height =
      cfg::Dimensions::actual_field_goal_height_mm / field.actual_to_our_ratio;
  EXPECT_NEAR(field.goal_height_mm, expected_goal_height, 1e-6f)
      << "Goal height should be scaled correctly";

  // Check line width
  float expected_line_width =
      cfg::Dimensions::actual_field_line_width_mm / field.actual_to_our_ratio;
  EXPECT_NEAR(field.line_width_mm, expected_line_width, 1e-6f)
      << "Line width should be scaled correctly";

  // Check outer dimensions
  float expected_outer_width = cfg::Dimensions::outer_field_width_mm / field.actual_to_our_ratio;
  EXPECT_NEAR(field.outer_width_mm, expected_outer_width, 1e-6f)
      << "Outer width should be scaled correctly";

  float expected_outer_height = cfg::Dimensions::outer_field_height_mm / field.actual_to_our_ratio;
  EXPECT_NEAR(field.outer_height_mm, expected_outer_height, 1e-6f)
      << "Outer height should be scaled correctly";
}

// Test that all dimensions are positive after initialization
TEST(SoccerFieldTest, TestAllDimensionsArePositive) {
  vis::SoccerField& field = vis::SoccerField::GetInstance();

  EXPECT_GT(field.width_mm, 0.0f);
  EXPECT_GT(field.height_mm, 0.0f);
  EXPECT_GT(field.central_circle_radius_mm, 0.0f);
  EXPECT_GT(field.playing_area_width_mm, 0.0f);
  EXPECT_GT(field.playing_area_height_mm, 0.0f);
  EXPECT_GT(field.penalty_area_width_mm, 0.0f);
  EXPECT_GT(field.penalty_area_height_mm, 0.0f);
  EXPECT_GT(field.goal_width_mm, 0.0f);
  EXPECT_GT(field.goal_height_mm, 0.0f);
  EXPECT_GT(field.line_width_mm, 0.0f);
  EXPECT_GT(field.outer_width_mm, 0.0f);
  EXPECT_GT(field.outer_height_mm, 0.0f);
  EXPECT_GT(field.actual_to_our_ratio, 0.0f);
}