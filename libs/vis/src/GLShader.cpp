#include <string>

#include "GLWindow.h"
#include "Utils.h"

void vis::GLWindow::CreateVertexBuffer(const void* vertices) {
  unsigned int vbo;
  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);  // the array buffer now maps to vbo

  // stream: position doesn't change, used fixed number of times
  // static: position doesn't change, used a lot
  // dynamic: position changes, used a lot
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
}

bool vis::GLWindow::CreateVertexShader() {
  // Get the source code from file
  std::string shader_code = util::ReadFile("../resources/shaders/VertexShader.vs");
  const char* vertex_shader_source = shader_code.c_str();

  vertex_shader_id = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex_shader_id, 1, &vertex_shader_source, NULL);
  glCompileShader(vertex_shader_id);

  // Verify if vertex shader compiled successfully
  int success;
  char info_log[512];
  glGetShaderiv(vertex_shader_id, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(vertex_shader_id, 512, NULL, info_log);
    std::cout << "[vis::GLWindow::CreateVertexShader] Failed!\n" << info_log << std::endl;
    return false;
  }

  return true;
}

bool vis::GLWindow::CreateFragmentShader() {
  std::string shader_code = util::ReadFile("../resources/shaders/FragmentShader.fs");
  const char* fragment_shader_source = shader_code.c_str();

  fragment_shader_id = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragment_shader_id, 1, &fragment_shader_source, NULL);
  glCompileShader(fragment_shader_id);

  // Verify if this shader linked successfully
  int success;
  char info_log[512];
  glGetProgramiv(fragment_shader_id, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(fragment_shader_id, 512, NULL, info_log);
    std::cout << "[vis::GLWindow::CreateFragmentShader] Failed!\n" << info_log << std::endl;
    return false;
  }

  return true;
}

bool vis::GLWindow::CreateShaderProgram() {
  unsigned int shader_program_id;
  shader_program_id = glCreateProgram();
  glAttachShader(shader_program_id, vertex_shader_id);
  glAttachShader(shader_program_id, fragment_shader_id);
  glLinkProgram(shader_program_id);

  // Verify if shader program linked successfully
  int success;
  char info_log[512];
  glGetProgramiv(shader_program_id, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(shader_program_id, 512, NULL, info_log);
    std::cout << "[vis::GLWindow::CreateShaderProgram] Failed!\n" << info_log << std::endl;
    return false;
  }

  glUseProgram(shader_program_id);

  glDeleteShader(vertex_shader_id);
  glDeleteShader(fragment_shader_id);

  return true;
}