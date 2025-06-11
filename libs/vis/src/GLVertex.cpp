#include <iostream>

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

  // When using VAO instead of EBO
  glBindVertexArray(vao);
  // glDrawArrays(GL_TRIANGLES, 0, 3);

  // EBO
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}