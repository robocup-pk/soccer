#include "Dimensions.h"

namespace cfg {

const float Dimensions::actual_field_width_mm = 10400.0f;
const float Dimensions::actual_field_height_mm = 7400.0f;
const float Dimensions::actual_field_central_circle_radius_mm = 500.0f;
const float Dimensions::actual_field_playing_area_width_mm = 9000.0f;
const float Dimensions::actual_field_playing_area_height_mm = 6000.0f;
const float Dimensions::actual_field_penalty_area_width_mm = 1000.0f;
const float Dimensions::actual_field_penalty_area_height_mm = 2000.0f;
const float Dimensions::actual_field_goal_width_mm = 200.0f;
const float Dimensions::actual_field_goal_height_mm = 1000.0f;
const float Dimensions::actual_field_line_width_mm = 10.0f;
const float Dimensions::outer_field_width_mm = 9600.0f;
const float Dimensions::outer_field_height_mm = 6600.0f;

// Note: Dont use this values, they are later scaled down in Soccer Field to maintain the aspect
// and actual/our ratio of the field. If you want soccer field Dimensions, use the
// Singleton Class `SoccerField` which will return the correct values.
const float Dimensions::width_mm = 5486.4f;
const float Dimensions::height_mm = 2900.0f;

}  // namespace cfg