#include <iostream>
#include <cmath>
#include <string>
#include <vector>

#include "SoccerField.h"
#include "Dimensions.h"
#include "Shader.h"
#include "ResourceManager.h"
#include "SpriteRenderer.h"
#include "Utils.h"

vis::SoccerField::SoccerField() {
  vis::SoccerField::MaintainAspectRatio();
  vis::SoccerField::CalculateFieldDimensions();
}

void vis::SoccerField::RenderField(GLFWwindow* r_window) {
  this->window = r_window;
  Render();
}

void vis::SoccerField::SoccerFieldInit() {
  Shader shader = ResourceManager::GetShader("field");
  shader.Use();

  // Store the shader program ID for later use
  this->shaderProgram = shader.ID;

  // Get uniform locations
  this->projectionLoc = glGetUniformLocation(this->shaderProgram, "projection");

  // Initialize renderer with field shader
  this->renderer.Init(shader);

  SetupBuffers();
  UpdateProjectionMatrix();
}

void vis::SoccerField::MaintainAspectRatio() {
  if (cfg::Dimensions::width_mm <= 0 || cfg::Dimensions::height_mm <= 0) {
    std::cerr << "Error: Width and height must be positive values." << std::endl;
    return;
  }

  // Calculate the width-height ratio
  vis::SoccerField::width_height_ratio =
      cfg::Dimensions::actual_field_width_mm / cfg::Dimensions::actual_field_height_mm;

  // Ensure the ratio is not zero to avoid division by zero
  if (vis::SoccerField::width_height_ratio == 0) {
    std::cerr << "Error: Width-height ratio cannot be zero." << std::endl;
    return;
  }

  // Calculate our field dimensions based on the actual field dimensions
  vis::SoccerField::height_mm = cfg::Dimensions::height_mm;
  vis::SoccerField::width_mm = vis::SoccerField::height_mm * vis::SoccerField::width_height_ratio;

  // Ensure the calculated dimensions are positive
  if (vis::SoccerField::width_mm <= 0 || vis::SoccerField::height_mm <= 0) {
    std::cerr << "Error: Calculated field dimensions must be positive." << std::endl;
    return;
  }
}

void vis::SoccerField::CalculateFieldDimensions() {
  this->actual_to_our_ratio = cfg::Dimensions::actual_field_width_mm / this->width_mm;

  this->central_circle_radius_mm =
      cfg::Dimensions::actual_field_central_circle_radius_mm / this->actual_to_our_ratio;
  this->playing_area_width_mm =
      cfg::Dimensions::actual_field_playing_area_width_mm / this->actual_to_our_ratio;
  this->playing_area_height_mm =
      cfg::Dimensions::actual_field_playing_area_height_mm / this->actual_to_our_ratio;
  this->penalty_area_width_mm =
      cfg::Dimensions::actual_field_penalty_area_width_mm / this->actual_to_our_ratio;
  this->penalty_area_height_mm =
      cfg::Dimensions::actual_field_penalty_area_height_mm / this->actual_to_our_ratio;
  this->goal_width_mm = cfg::Dimensions::actual_field_goal_width_mm / this->actual_to_our_ratio;
  this->goal_height_mm = cfg::Dimensions::actual_field_goal_height_mm / this->actual_to_our_ratio;
  this->line_width_mm = cfg::Dimensions::actual_field_line_width_mm / this->actual_to_our_ratio;
  this->outer_width_mm = cfg::Dimensions::outer_field_width_mm / this->actual_to_our_ratio;
  this->outer_height_mm = cfg::Dimensions::outer_field_height_mm / this->actual_to_our_ratio;
}

void vis::SoccerField::SetupBuffers() {
  // Rectangle vertices (field boundary) - convert from mm to pixels
  float half_width = util::MmToPixels(this->playing_area_width_mm / 2);
  float half_height = util::MmToPixels(this->playing_area_height_mm / 2);

  float vertices[] = {
      -half_width, -half_height,  // Bottom left
      half_width,  -half_height,  // Bottom right
      half_width,  half_height,   // Top right
      -half_width, half_height    // Top left
  };

  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);

  glBindVertexArray(VAO);

  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}

void vis::SoccerField::UpdateProjectionMatrix() {
  // Create orthographic projection matrix for 2D rendering
  float half_width = util::MmToPixels(this->width_mm / 2);
  float half_height = util::MmToPixels(this->height_mm / 2);

  // Initialize to zero
  for (int i = 0; i < 16; i++) {
    projectionMatrix[i] = 0.0f;
  }

  // 2D Orthographic projection matrix for centered coordinate system
  projectionMatrix[0] =
      1.0f / half_width;  // X scaling: maps [-half_width, +half_width] to [-1, +1]
  projectionMatrix[5] =
      1.0f / half_height;        // Y scaling: maps [-half_height, +half_height] to [-1, +1]
  projectionMatrix[10] = -1.0f;  // Z scaling: simple -1 for 2D (maps any Z to clip space)
  projectionMatrix[15] = 1.0f;   // Homogeneous coordinate
}

void vis::SoccerField::DrawCenterCircle() {
  float radius = util::MmToPixels(this->central_circle_radius_mm);

  // Calculate optimal number of segments based on circle size
  // Rule: approximately 1 segment per 3-4 pixels of circumference for smooth appearance
  float circumference = 2.0f * M_PI * radius;
  int num_segments = std::max(12, std::min(128, (int)(circumference / 3.0f)));

  // Ensure even number of segments for symmetry
  if (num_segments % 2 != 0) {
    num_segments++;
  }

  // Generate circle vertices
  std::vector<float> circle_vertices(num_segments * 2);  // 2 coordinates per vertex

  for (int i = 0; i < num_segments; i++) {
    float angle = 2.0f * M_PI * i / num_segments;
    circle_vertices[i * 2] = radius * cos(angle);      // x coordinate
    circle_vertices[i * 2 + 1] = radius * sin(angle);  // y coordinate
  }

  // Create VAO and VBO for the circle
  unsigned int circleVAO, circleVBO;
  glGenVertexArrays(1, &circleVAO);
  glGenBuffers(1, &circleVBO);

  glBindVertexArray(circleVAO);
  glBindBuffer(GL_ARRAY_BUFFER, circleVBO);
  glBufferData(GL_ARRAY_BUFFER, circle_vertices.size() * sizeof(float), circle_vertices.data(),
               GL_STATIC_DRAW);

  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);

  // Draw the circle as a line loop
  glDrawArrays(GL_LINE_LOOP, 0, num_segments);

  // Clean up
  glDeleteVertexArrays(1, &circleVAO);
  glDeleteBuffers(1, &circleVBO);
}

void vis::SoccerField::DrawOuterRectangle() {
  // Calculate outer field dimensions
  float half_outer_width = util::MmToPixels(this->outer_width_mm / 2);
  float half_outer_height = util::MmToPixels(this->outer_height_mm / 2);

  // Outer rectangle vertices
  float outer_vertices[] = {
      -half_outer_width, -half_outer_height,  // Bottom left
      half_outer_width,  -half_outer_height,  // Bottom right
      half_outer_width,  half_outer_height,   // Top right
      -half_outer_width, half_outer_height    // Top left
  };

  // Create VAO and VBO for the outer rectangle
  unsigned int outerVAO, outerVBO;
  glGenVertexArrays(1, &outerVAO);
  glGenBuffers(1, &outerVBO);

  glBindVertexArray(outerVAO);
  glBindBuffer(GL_ARRAY_BUFFER, outerVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(outer_vertices), outer_vertices, GL_STATIC_DRAW);

  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);

  // Set color to black for outer boundary
  int colorLoc = glGetUniformLocation(shaderProgram, "color");
  glUniform3f(colorLoc, 0.0f, 0.0f, 0.0f);  // Black color

  // Draw the outer rectangle
  glDrawArrays(GL_LINE_LOOP, 0, 4);

  // Clean up
  glDeleteVertexArrays(1, &outerVAO);
  glDeleteBuffers(1, &outerVBO);
}

void vis::SoccerField::DrawPenaltyAreas() {
  // Calculate penalty area dimensions
  float penalty_width = util::MmToPixels(this->penalty_area_width_mm);
  float penalty_height = util::MmToPixels(this->penalty_area_height_mm);
  float half_penalty_width = penalty_width / 2;
  float half_penalty_height = penalty_height / 2;

  // Calculate field boundaries for positioning
  float half_field_width = util::MmToPixels(this->playing_area_width_mm / 2);

  // Left penalty area (positioned at left end of field)
  float left_penalty_vertices[] = {
      -half_field_width,
      -half_penalty_height,  // Bottom left
      -half_field_width + penalty_width,
      -half_penalty_height,  // Bottom right
      -half_field_width + penalty_width,
      half_penalty_height,  // Top right
      -half_field_width,
      half_penalty_height  // Top left
  };

  // Right penalty area (positioned at right end of field)
  float right_penalty_vertices[] = {
      half_field_width - penalty_width,
      -half_penalty_height,  // Bottom left
      half_field_width,
      -half_penalty_height,  // Bottom right
      half_field_width,
      half_penalty_height,  // Top right
      half_field_width - penalty_width,
      half_penalty_height  // Top left
  };

  // Set color to white for penalty area lines
  int colorLoc = glGetUniformLocation(shaderProgram, "color");
  glUniform3f(colorLoc, 1.0f, 1.0f, 1.0f);  // White color

  // Draw left penalty area
  unsigned int leftPenaltyVAO, leftPenaltyVBO;
  glGenVertexArrays(1, &leftPenaltyVAO);
  glGenBuffers(1, &leftPenaltyVBO);

  glBindVertexArray(leftPenaltyVAO);
  glBindBuffer(GL_ARRAY_BUFFER, leftPenaltyVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(left_penalty_vertices), left_penalty_vertices,
               GL_STATIC_DRAW);

  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);

  glDrawArrays(GL_LINE_LOOP, 0, 4);

  glDeleteVertexArrays(1, &leftPenaltyVAO);
  glDeleteBuffers(1, &leftPenaltyVBO);

  // Draw right penalty area
  unsigned int rightPenaltyVAO, rightPenaltyVBO;
  glGenVertexArrays(1, &rightPenaltyVAO);
  glGenBuffers(1, &rightPenaltyVBO);

  glBindVertexArray(rightPenaltyVAO);
  glBindBuffer(GL_ARRAY_BUFFER, rightPenaltyVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(right_penalty_vertices), right_penalty_vertices,
               GL_STATIC_DRAW);

  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);

  glDrawArrays(GL_LINE_LOOP, 0, 4);

  glDeleteVertexArrays(1, &rightPenaltyVAO);
  glDeleteBuffers(1, &rightPenaltyVBO);
}

void vis::SoccerField::DrawGoalAreas() {
  // Calculate goal area dimensions in pixels
  float goal_width = util::MmToPixels(this->goal_width_mm);
  float half_goal_height = util::MmToPixels(this->goal_height_mm / 2);

  // Calculate field boundaries for positioning
  float half_field_width = util::MmToPixels(this->playing_area_width_mm / 2);

  // Left goal area (positioned at left end of field)
  float left_goal_vertices[] = {
      -half_field_width,
      -half_goal_height,  // Bottom left
      -half_field_width - goal_width,
      -half_goal_height,  // Bottom right
      -half_field_width - goal_width,
      half_goal_height,  // Top right
      -half_field_width,
      half_goal_height  // Top left
  };

  // Right goal area (positioned at right end of field)
  float right_goal_vertices[] = {
      half_field_width + goal_width,
      -half_goal_height,  // Bottom left
      half_field_width,
      -half_goal_height,  // Bottom right
      half_field_width,
      half_goal_height,  // Top right
      half_field_width + goal_width,
      half_goal_height  // Top left
  };

  // Set color to red for goal area lines
  int colorLoc = glGetUniformLocation(shaderProgram, "color");
  glUniform3f(colorLoc, 1.0f, 0.0f, 0.0f);  // Red color

  // Draw left goal area
  unsigned int leftGoalVAO, leftGoalVBO;
  glGenVertexArrays(1, &leftGoalVAO);
  glGenBuffers(1, &leftGoalVBO);

  glBindVertexArray(leftGoalVAO);
  glBindBuffer(GL_ARRAY_BUFFER, leftGoalVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(left_goal_vertices), left_goal_vertices, GL_STATIC_DRAW);

  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);

  glDrawArrays(GL_LINE_LOOP, 0, 4);

  glDeleteVertexArrays(1, &leftGoalVAO);
  glDeleteBuffers(1, &leftGoalVBO);

  // Draw right goal area
  unsigned int rightGoalVAO, rightGoalVBO;
  glGenVertexArrays(1, &rightGoalVAO);
  glGenBuffers(1, &rightGoalVBO);

  glBindVertexArray(rightGoalVAO);
  glBindBuffer(GL_ARRAY_BUFFER, rightGoalVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(right_goal_vertices), right_goal_vertices, GL_STATIC_DRAW);

  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);

  glDrawArrays(GL_LINE_LOOP, 0, 4);

  glDeleteVertexArrays(1, &rightGoalVAO);
  glDeleteBuffers(1, &rightGoalVBO);
}

void vis::SoccerField::SetupProjection() {
  glViewport(0, 0, util::MmToPixels(this->width_mm), util::MmToPixels(this->height_mm));
}

void vis::SoccerField::Render() {
  glUseProgram(shaderProgram);

  // Set projection matrix
  glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, projectionMatrix);

  // Set color to white for boundary lines
  int colorLoc = glGetUniformLocation(shaderProgram, "color");
  glUniform3f(colorLoc, 1.0f, 1.0f, 1.0f);

  // Set line width using consistent conversion
  float line_width_pixels = util::MmToPixels(this->line_width_mm);
  glLineWidth(line_width_pixels);

  // Draw outer rectangle first (black boundary)
  DrawOuterRectangle();

  // Reset color to white for field lines
  glUniform3f(colorLoc, 1.0f, 1.0f, 1.0f);

  // Calculate field dimensions
  float half_width = util::MmToPixels(this->playing_area_width_mm / 2);
  float half_height = util::MmToPixels(this->playing_area_height_mm / 2);

  // Draw the boundary rectangle
  glBindVertexArray(VAO);
  glDrawArrays(GL_LINE_LOOP, 0, 4);
  glBindVertexArray(0);

  // Draw center line (vertical line dividing field into left/right halves)
  float center_line_vertices[] = {
      0.0f, -half_height,  // Bottom center
      0.0f, half_height    // Top center
  };

  unsigned int centerLineVAO, centerLineVBO;
  glGenVertexArrays(1, &centerLineVAO);
  glGenBuffers(1, &centerLineVBO);

  glBindVertexArray(centerLineVAO);
  glBindBuffer(GL_ARRAY_BUFFER, centerLineVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(center_line_vertices), center_line_vertices,
               GL_STATIC_DRAW);

  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);

  glDrawArrays(GL_LINES, 0, 2);

  glDeleteVertexArrays(1, &centerLineVAO);
  glDeleteBuffers(1, &centerLineVBO);

  // Draw halfway line (horizontal line dividing field into top/bottom halves)
  float halfway_line_vertices[] = {
      -half_width, 0.0f,  // Left center
      half_width, 0.0f    // Right center
  };

  unsigned int halfwayLineVAO, halfwayLineVBO;
  glGenVertexArrays(1, &halfwayLineVAO);
  glGenBuffers(1, &halfwayLineVBO);

  glBindVertexArray(halfwayLineVAO);
  glBindBuffer(GL_ARRAY_BUFFER, halfwayLineVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(halfway_line_vertices), halfway_line_vertices,
               GL_STATIC_DRAW);

  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);
  glDrawArrays(GL_LINES, 0, 2);

  glDeleteVertexArrays(1, &halfwayLineVAO);
  glDeleteBuffers(1, &halfwayLineVBO);

  // Draw center circle
  DrawCenterCircle();

  // Draw penalty areas
  DrawPenaltyAreas();

  // Draw goal areas
  DrawGoalAreas();
}