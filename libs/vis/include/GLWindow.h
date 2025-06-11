#ifndef GL_WINDOW_H
#define GL_WINDOW_H

// extern libs
#include <glad/glad.h>  // ORDER MATTERS (bw glad and glfw)
#include <GLFW/glfw3.h>

namespace vis {

class GLWindow {
 public:
  bool Init(int width_px, int height_px, const char* window_title, bool use_ebo = false);
  void RegisterCallbacks();
  GLFWwindow* GetRawGLFW() const;

  // Logic used in simulation
  void SetScreenColor();
  bool Update();

  // OpenGL specific stuff
  void Draw();
  void CreateVertexAttributeObject(const float* vertices, unsigned int* indices = nullptr);
  bool CreateVertexShader();
  bool CreateFragmentShader();
  bool CreateShaderProgram();

  ~GLWindow();

  unsigned int shader_program_id;
  unsigned int vao;  // vertex attribute object
  unsigned int ebo;

 private:
  GLFWwindow* window;
  int width_px, height_px;

  unsigned int fragment_shader_id;
  unsigned int vertex_shader_id;

  bool use_ebo;
};

}  // namespace vis

#endif  // GL_WINDOW_H