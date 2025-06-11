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
  glBufferData(GL_ARRAY_BUFFER, 12 * sizeof(float), vertices,
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
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);
}

void vis::GLWindow::Draw() {
  glUseProgram(shader_program_id);
  int offset_loc = glGetUniformLocation(shader_program_id, "offset");
  float x = vis::GLCallback::x_offset;
  float y = vis::GLCallback::y_offset;
  std::cout << "[vis::GLWindow::Draw] " << x << " " << y << std::endl;
  glUniform2f(offset_loc, x, y);
  glBindVertexArray(vao);

  if (use_ebo) {
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
  } else {
    glDrawArrays(GL_TRIANGLES, 0, 3);
  }
}