#ifndef DIMENSIONS_H
#define DIMENSIONS_H

namespace cfg {
struct Dimensions {
  static const float actual_field_width_mm;
  static const float actual_field_height_mm;
  static const float actual_field_central_circle_radius_mm;
  static const float actual_field_playing_area_width_mm;
  static const float actual_field_playing_area_height_mm;
  static const float actual_field_penalty_area_width_mm;
  static const float actual_field_penalty_area_height_mm;
  static const float actual_field_goal_width_mm;
  static const float actual_field_goal_height_mm;
  static const float actual_field_line_width_mm;
  static const float outer_field_width_mm;
  static const float outer_field_height_mm;

  static const float width_mm;
  static const float height_mm;
};
}  // namespace cfg

#endif