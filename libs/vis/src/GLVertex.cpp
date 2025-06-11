#include <iostream>

#include "GLWindow.h"

void vis::GLWindow::CreateVertexAttributeObject(const float* vertices) {
  unsigned int vbo;

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);

  // TODO: 9...make it general
  glBufferData(GL_ARRAY_BUFFER, 9 * sizeof(float), vertices, GL_STATIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);
}

void vis::GLWindow::Draw() {
  glUseProgram(shader_program_id);
  glBindVertexArray(vao);

  glDrawArrays(GL_TRIANGLES, 0, 3);
}