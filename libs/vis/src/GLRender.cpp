#include <cmath>
#include <iostream>

#include "GLCallback.h"
#include "GLWindow.h"

void vis::GLWindow::CreateVertexAttributeObject(const float* vertices, unsigned int* indices) {
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  // vbo
  unsigned int vbo;
  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, 32 * sizeof(float), vertices,
               GL_STATIC_DRAW);  // TODO: 9...make it general

  // ebo
  if (indices != nullptr) {
    unsigned int ebo;
    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * sizeof(unsigned int), indices,
                 GL_STATIC_DRAW);  // TODO: 6??...
  }

  // vertex attribute
  // glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
  // glEnableVertexAttribArray(0);

  // position (location = 0)
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(0));
  glEnableVertexAttribArray(0);

  // color (location = 1)
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  // tex coords (location = 2)
  glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
  glEnableVertexAttribArray(2);
}

void vis::GLWindow::Render() {  // Ensure blending is on (in case it got disabled somewhere)
  glEnable(GL_BLEND);
  glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
  // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glUseProgram(shader_program_id);

  // Position Offset
  int offset_loc = glGetUniformLocation(shader_program_id, "offset");
  float x = vis::GLCallback::x_offset;
  float y = vis::GLCallback::y_offset;
  glUniform2f(offset_loc, x, y);

  // Robot Color
  // float current_time = glfwGetTime();
  // float green_value = (std::sin(current_time) / 2.0f) + 0.5f;
  // int robot_color_loc = glGetUniformLocation(shader_program_id, "robot_color");
  // glUniform4f(robot_color_loc, 0.0f, green_value, 0.0f, 1.0f);

  //
  int texture_loc = glGetUniformLocation(shader_program_id, "ourTexture");
  glUniform1i(texture_loc, 0);  // GL_TEXTURE0
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, texture_id);

  // Render
  glBindVertexArray(vao);
  if (use_ebo) {
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
  } else {
    glDrawArrays(GL_TRIANGLES, 0, 3);
  }

  // Update Step: Polls IO, SwapBuffers
}