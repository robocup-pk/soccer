#include <iostream>

#include "Shader.h"
#include "GLSimulation.h"

vis::Shader& vis::Shader::Use() {
  glUseProgram(this->ID);
  return *this;
}

void vis::Shader::Compile(const char* vertexSource, const char* fragmentSource) {
  unsigned int sVertex, sFragment;
  // vertex Shader
  sVertex = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(sVertex, 1, &vertexSource, NULL);
  glCompileShader(sVertex);
  checkCompileErrors(sVertex, "VERTEX");
  // fragment Shader
  sFragment = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(sFragment, 1, &fragmentSource, NULL);
  glCompileShader(sFragment);
  checkCompileErrors(sFragment, "FRAGMENT");
  // shader program
  this->ID = glCreateProgram();
  glAttachShader(this->ID, sVertex);
  glAttachShader(this->ID, sFragment);

  glLinkProgram(this->ID);
  checkCompileErrors(this->ID, "PROGRAM");
  // delete the shaders as they're linked into our program now and no longer necessary
  glDeleteShader(sVertex);
  glDeleteShader(sFragment);
}

void vis::Shader::SetFloat(const char* name, float value, bool useShader) {
  if (useShader) this->Use();
  glUniform1f(glGetUniformLocation(this->ID, name), value);
}
void vis::Shader::SetInteger(const char* name, int value, bool useShader) {
  if (useShader) this->Use();
  glUniform1i(glGetUniformLocation(this->ID, name), value);
}
void vis::Shader::SetVector2f(const char* name, float x, float y, bool useShader) {
  if (useShader) this->Use();
  glUniform2f(glGetUniformLocation(this->ID, name), x, y);
}
void vis::Shader::SetVector2f(const char* name, const glm::vec2& value, bool useShader) {
  if (useShader) this->Use();
  glUniform2f(glGetUniformLocation(this->ID, name), value.x, value.y);
}
void vis::Shader::SetVector3f(const char* name, float x, float y, float z, bool useShader) {
  if (useShader) this->Use();
  glUniform3f(glGetUniformLocation(this->ID, name), x, y, z);
}
void vis::Shader::SetVector3f(const char* name, const glm::vec3& value, bool useShader) {
  if (useShader) this->Use();
  glUniform3f(glGetUniformLocation(this->ID, name), value.x, value.y, value.z);
}
void vis::Shader::SetVector4f(const char* name, float x, float y, float z, float w,
                              bool useShader) {
  if (useShader) this->Use();
  glUniform4f(glGetUniformLocation(this->ID, name), x, y, z, w);
}
void vis::Shader::SetVector4f(const char* name, const glm::vec4& value, bool useShader) {
  if (useShader) this->Use();
  glUniform4f(glGetUniformLocation(this->ID, name), value.x, value.y, value.z, value.w);
}
void vis::Shader::SetMatrix4(const char* name, const glm::mat4& matrix, bool useShader) {
  if (useShader) this->Use();
  glUniformMatrix4fv(glGetUniformLocation(this->ID, name), 1, false, glm::value_ptr(matrix));
}

void vis::Shader::checkCompileErrors(unsigned int object, std::string type) {
  int success;
  char infoLog[1024];
  if (type != "PROGRAM") {
    glGetShaderiv(object, GL_COMPILE_STATUS, &success);
    if (!success) {
      glGetShaderInfoLog(object, 1024, NULL, infoLog);
      std::cout << "| ERROR::SHADER: Compile-time error: Type: " << type << "\n"
                << infoLog << "\n -- --------------------------------------------------- -- "
                << std::endl;
    }
  } else {
    glGetProgramiv(object, GL_LINK_STATUS, &success);
    if (!success) {
      glGetProgramInfoLog(object, 1024, NULL, infoLog);
      std::cout << "| ERROR::Shader: Link-time error: Type: " << type << "\n"
                << infoLog << "\n -- --------------------------------------------------- -- "
                << std::endl;
    }
  }
}