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

  // Set up projection matrix for 2D rendering
  // glm::mat4 projection =
  //     glm::ortho(0.0f,
  //     static_cast<float>(util::MmToPixels(SoccerField::GetInstance().width_mm)),
  //                static_cast<float>(util::MmToPixels(SoccerField::GetInstance().height_mm)),
  //                0.0f, -1.0f, 1.0f);
  // Set up projection matrix with (0,0) at center
  float half_width =
      static_cast<float>(util::MmToPixels(SoccerField::GetInstance().width_mm)) / 2.0f;
  float half_height =
      static_cast<float>(util::MmToPixels(SoccerField::GetInstance().height_mm)) / 2.0f;

  glm::mat4 projection =
      glm::ortho(-half_width, half_width,  // left: -400, right: +400 (for 800px window)
                 half_height,
                 -half_height,  // bottom: +300, top: -300 (for 600px window, keeps y+ going up)
                 -1.0f, 1.0f);
  this->shader.Use().SetMatrix4("projection", projection);
}

vis::SpriteRenderer::~SpriteRenderer() { glDeleteVertexArrays(1, &this->quadVAO); }

void vis::SpriteRenderer::DrawSprite(Texture2D& texture, glm::vec2 position, glm::vec2 size,
                                     float rotate, glm::vec3 color) {
  this->shader.Use();
  glm::mat4 model = glm::mat4(1.0f);
  model = glm::translate(model, glm::vec3(position, 0.0f));
  // first translate (transformations are: scale happens first, then
  // rotation, and then final translation happens; reversed order)

  model = glm::translate(model, glm::vec3(0.5f * size.x, 0.5f * size.y,
                                          0.0f));  // move origin of rotation to center of quad
  model = glm::rotate(model, glm::radians(rotate), glm::vec3(0.0f, 0.0f, 1.0f));  // then rotate
  model =
      glm::translate(model, glm::vec3(-0.5f * size.x, -0.5f * size.y, 0.0f));  // move origin back
  model = glm::scale(model, glm::vec3(size, 1.0f));                            // last scale

  this->shader.SetMatrix4("model", model);
  // render textured quad
  this->shader.SetVector3f("spriteColor", color);

  glActiveTexture(GL_TEXTURE0);
  texture.Bind();

  glBindVertexArray(this->quadVAO);
  glDrawArrays(GL_TRIANGLES, 0, 6);
  glBindVertexArray(0);
}

void vis::SpriteRenderer::InitRenderData() {
  // configure VAO/VBO
  unsigned int VBO;

  // 6 vertices for 2 triangles (quad)
  // Each vertex: x, y, u, v (position + texture coordinates)
  float vertices[] = {
      // First triangle
      0.0f, 1.0f, 0.0f, 1.0f,  // top-left
      1.0f, 0.0f, 1.0f, 0.0f,  // bottom-right
      0.0f, 0.0f, 0.0f, 0.0f,  // bottom-left

      // Second triangle
      0.0f, 1.0f, 0.0f, 1.0f,  // top-left
      1.0f, 1.0f, 1.0f, 1.0f,  // top-right
      1.0f, 0.0f, 1.0f, 0.0f   // bottom-right
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
