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
  bool RunSimulationStep();
  void SetScreenColor();
  bool Update();

  // OpenGL specific stuff
  void Render();
  void CreateVertexAttributeObject(const float* vertices, unsigned int* indices = nullptr);
  bool CreateVertexShader();
  bool CreateFragmentShader();
  bool CreateShaderProgram();
  void CreateTexture(int width_px, int height_px);

  ~GLWindow();

  unsigned int shader_program_id;
  unsigned int vao;  // vertex attribute object
  unsigned int ebo;

  // float x_offset = 0.0f;
  // float y_offset = 0.0f;

 private:
  GLFWwindow* window;
  int width_px, height_px;

  unsigned int fragment_shader_id;
  unsigned int vertex_shader_id;
  unsigned int texture_id;

  bool use_ebo;
};

}  // namespace vis

#endif  // GL_WINDOW_H