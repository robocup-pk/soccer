#include <iostream>
#include <fstream>
#include <sstream>
#include <map>

#include "SpriteRenderer.h"
#include "GLSimulation.h"
#include "Shader.h"
#include "Texture.h"
#include "GLCallback.h"
#include "Utils.h"
#include "SoccerField.h"

void vis::SpriteRenderer::Init(Shader& shader) {
  this->shader = shader;
  this->InitRenderData();

  float half_width =
      static_cast<float>(util::MmToPixels(SoccerField::GetInstance().width_mm)) / 2.0f;
  float half_height =
      static_cast<float>(util::MmToPixels(SoccerField::GetInstance().height_mm)) / 2.0f;

  // CORRECTED: For Y+ to go up, bottom should be negative and top should be positive
  glm::mat4 projection = glm::ortho(
      -half_width, half_width,    // left: -400, right: +400 (for 800px window)
      -half_height, half_height,  // bottom: -300, top: +300 (for 600px window, Y+ goes up)
      -1.0f, 1.0f);
  this->shader.Use().SetMatrix4("projection", projection);
}

vis::SpriteRenderer::~SpriteRenderer() {
  if (glfwGetCurrentContext() != nullptr) {
    glDeleteVertexArrays(1, &this->quadVAO);
  }
}

void vis::SpriteRenderer::DrawSprite(Texture2D& texture, glm::vec2 position, glm::vec2 size,
                                     float rotate, glm::vec3 color) {
  position.y = -position.y;
  shader.Use();
  glm::mat4 model = glm::mat4(1.0f);

  // 1. Translate to world position (centered at (0,0))
  model = glm::translate(model, glm::vec3(position, 0.0f));

  // 2. Rotate around the center (since quad is [-0.5,0.5], rotation is centered)
  model = glm::rotate(model, glm::radians(rotate), glm::vec3(0.0f, 0.0f, 1.0f));

  // 3. Scale to size
  model = glm::scale(model, glm::vec3(size, 1.0f));

  shader.SetMatrix4("model", model);
  shader.SetVector3f("spriteColor", color);

  glActiveTexture(GL_TEXTURE0);
  texture.Bind();
  glBindVertexArray(quadVAO);
  glDrawArrays(GL_TRIANGLES, 0, 6);
  glBindVertexArray(0);
}

void vis::SpriteRenderer::InitRenderData() {
  // configure VAO/VBO
  unsigned int VBO;

  float vertices[] = {
      -0.5f, 0.5f,  0.0f, 0.0f,  // top-left
      0.5f,  -0.5f, 1.0f, 1.0f,  // bottom-right
      -0.5f, -0.5f, 0.0f, 1.0f,  // bottom-left

      -0.5f, 0.5f,  0.0f, 0.0f,  // top-left
      0.5f,  0.5f,  1.0f, 0.0f,  // top-right
      0.5f,  -0.5f, 1.0f, 1.0f   // bottom-right
  };

  glGenVertexArrays(1, &this->quadVAO);
  glGenBuffers(1, &VBO);

  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

  glBindVertexArray(this->quadVAO);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}