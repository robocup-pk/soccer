#include "PathRenderer.h"
#include "Utils.h"
#include "Shader.h"
#include "ResourceManager.h"
#include "SoccerField.h"

vis::PathRenderer::PathRenderer() {
  pathVAO = 0;
  pathVBO = 0;
  pathVisible = false;
  initialized = false;
  pathColor = glm::vec3(1.0f, 0.0f, 0.0f);  // Default red
  pathVertexCount = 0;
}

void vis::PathRenderer::Init() {
  if (initialized) return;  // Already initialized

  // Initialize path rendering buffers - call this after OpenGL context is ready
  glGenVertexArrays(1, &pathVAO);
  glGenBuffers(1, &pathVBO);

  glBindVertexArray(pathVAO);
  glBindBuffer(GL_ARRAY_BUFFER, pathVBO);

  // Set up vertex attributes (same as field lines)
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);

  glBindVertexArray(0);

  initialized = true;
}

void vis::PathRenderer::SetPath(state::Path &new_path,
                                glm::vec3 color = glm::vec3(1.0f, 0.0f, 0.0f)) {
  if (!initialized) return;  // Don't set path if not initialized
  this->path = new_path;
  pathColor = color;
  pathVisible = true;
  UpdatePath();
}

void vis::PathRenderer::UpdatePath() {
  if (!initialized || path.size() < 2) {
    pathVertexCount = 0;
    return;
  }

  // Convert path to OpenGL vertices (similar to field line conversion)
  std::vector<float> vertices;
  vertices.reserve(path.size() * 4);  // 2 points per segment, 2 coords per point

  for (size_t i = 0; i < path.size() - 1; ++i) {
    // Convert from world coordinates (meters) to screen coordinates
    // Use the same coordinate system as SoccerField
    float x1 = util::MmToPixels(path[i].x * 1000.0f);  // m to mm to pixels
    float y1 = util::MmToPixels(path[i].y * 1000.0f);
    float x2 = util::MmToPixels(path[i + 1].x * 1000.0f);
    float y2 = util::MmToPixels(path[i + 1].y * 1000.0f);

    vertices.push_back(x1);
    vertices.push_back(y1);
    vertices.push_back(x2);
    vertices.push_back(y2);
  }

  pathVertexCount = vertices.size() / 2;

  // Update VBO with new data
  glBindBuffer(GL_ARRAY_BUFFER, pathVBO);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void vis::PathRenderer::Render() {
  if (!initialized || !pathVisible || pathVertexCount == 0) return;

  // Use the same shader as SoccerField for consistency
  Shader fieldShader = ResourceManager::GetShader("field");
  fieldShader.Use();

  // Set projection matrix (same as field rendering)
  glm::mat4 projection = glm::ortho(-util::MmToPixels(SoccerField::GetInstance().width_mm) / 2.0f,
                                    util::MmToPixels(SoccerField::GetInstance().width_mm) / 2.0f,
                                    -util::MmToPixels(SoccerField::GetInstance().height_mm) / 2.0f,
                                    util::MmToPixels(SoccerField::GetInstance().height_mm) / 2.0f);
  fieldShader.SetMatrix4("projection", projection);

  // Set path color
  fieldShader.SetVector3f("color", pathColor);

  // Set line width (thicker for visibility)
  glLineWidth(4.0f);

  // Render path
  glBindVertexArray(pathVAO);
  glDrawArrays(GL_LINES, 0, pathVertexCount);
  glBindVertexArray(0);

  // Reset line width
  glLineWidth(1.0f);
}

void vis::PathRenderer::ClearPath() {
  path.clear();
  pathVisible = false;
  pathVertexCount = 0;
}

void vis::PathRenderer::SetPathVisible(bool visible) { pathVisible = visible; }

vis::PathRenderer::~PathRenderer() {
  if (initialized) {
    glDeleteVertexArrays(1, &pathVAO);
    glDeleteBuffers(1, &pathVBO);
  }
}