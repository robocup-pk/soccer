#ifndef FIELD_RENDERER_H
#define FIELD_RENDERER_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <string>

#include "SpriteRenderer.h"

namespace vis {

class SoccerField {
 public:
  float width_mm;
  float height_mm;
  float width_height_ratio;
  float central_circle_radius_mm;
  float playing_area_width_mm;
  float playing_area_height_mm;
  float penalty_area_width_mm;
  float penalty_area_height_mm;
  float goal_width_mm;
  float goal_height_mm;
  float line_width_mm;
  float outer_width_mm;
  float outer_height_mm;

  float actual_to_our_ratio;

  void RenderField(GLFWwindow* r_window);
  void SoccerFieldInit();

  static SoccerField& GetInstance() {
    static SoccerField instance;
    return instance;
  }

 private:
  // Initialization functions
  void MaintainAspectRatio();
  void CalculateFieldDimensions();

  SoccerField();
  ~SoccerField();

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