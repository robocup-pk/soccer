// cpp std libs
#include <iostream>

// self libs
#include "GLWindow.h"

bool vis::GLWindow::Init(int width_px_, int height_px_, const char* window_title) {
  width_px = width_px_;
  height_px = height_px_;

  // Initialize GLFW
  if (!glfwInit()) {
    std::cout << "[vis::GLWindow::Init] Couldn’t initialise GLFW\n";
    return false;
  }

  // Configure GLFW (option = GLFW_* and value = int)
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);  // required on macOS
#endif

  window = glfwCreateWindow(width_px, height_px, "RocoCup Simulator", nullptr, nullptr);

  if (!window) {
    std::cout << "[vis::GLWindow::Init] Couldn’t create window\n";
    glfwTerminate();
    return false;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);  // enable vsync (optional)

  // GLAD (manages function pointers for opengl)
  if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
    std::cout << "[vis::GLWindow::Init] Couldn’t load GL functions\n";
    return false;
  }

  // Area in GLFW window where OpenGL rendering is performed
  // glfwGetFramebufferSize(window, &width_px, &height_px);
  glViewport(0, 0, width_px, height_px);

  RegisterCallbacks();

  // Shaders
  if (!CreateVertexShader()) return false;
  if (!CreateFragmentShader()) return false;
  if (!CreateShaderProgram()) return false;

  // Vertex Attribute Object
  const float vertices[] = {0.5f,  0.5f,  0.0f, 0.5f,  -0.5f, 0.0f,
                            -0.5f, -0.5f, 0.0f, -0.5f, 0.5f,  0.0f};
  CreateVertexAttributeObject(vertices);
  return true;
}

bool vis::GLWindow::Update() {
  // Add the simulation logic here
  SetScreenColor();
  Draw();

  // This is the update logic
  if (glfwWindowShouldClose(window)) return false;
  glfwSwapBuffers(window);  // Large color buffer for each pixel in glfw window
  glfwPollEvents();         // Checks if any event is triggered. Updates state. Callbacks
  return true;
}

void vis::GLWindow::SetScreenColor() {
  glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);  // We also have depth and stencil buffers (?)
}

GLFWwindow* vis::GLWindow::GetRawGLFW() const { return window; }

vis::GLWindow::~GLWindow() {
  glfwTerminate();  // cleans all the resources taken by glfw
  // TODO: free the window ptr
}