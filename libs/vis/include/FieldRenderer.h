#ifndef FIELD_RENDERER_H
#define FIELD_RENDERER_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <string>

#include "SpriteRenderer.h"

namespace vis {

class FieldRenderer {
 public:
  float width_height_ratio;
  float our_field_width_mm;
  float our_field_height_mm;
  float our_field_central_circle_radius_mm;
  float our_field_playing_area_width_mm;
  float our_field_playing_area_height_mm;
  float our_field_penalty_area_width_mm;
  float our_field_penalty_area_height_mm;
  float our_field_goal_width_mm;
  float our_field_goal_height_mm;
  float our_field_line_width_mm;
  float our_field_outer_width_mm;
  float our_field_outer_height_mm;

  float actual_to_our_ratio;

  FieldRenderer();
  ~FieldRenderer();
  void RenderField(GLFWwindow* r_window);
  void FieldRendererInit();

  // Unit conversion functions
  float MmToPixels(float mm_value) const;
  float GetPixelsPerMm() const;

 private:
  // Initialization functions
  void MaintainAspectRatio();
  void CalculateFieldDimensions();

  void SetupProjection();
  void Render();

  // Draw functions
  void SetupBuffers();
  void UpdateProjectionMatrix();
  void DrawCenterCircle();
  void DrawOuterRectangle();
  void DrawPenaltyAreas();
  void DrawGoalAreas();

  GLFWwindow* window;
  SpriteRenderer renderer;

  unsigned int shaderProgram;
  unsigned int VAO, VBO;
  int projectionLoc;
  float projectionMatrix[16];
};

}  // namespace vis

#endif