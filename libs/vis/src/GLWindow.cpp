// cpp std libs
#include <iostream>

// self libs
#include "GLWindow.h"

bool vis::GLWindow::RunSimulationStep() {
  Render();
  return Update();
}

bool vis::GLWindow::Init(int width_px_, int height_px_, const char* window_title, bool use_ebo_) {
  width_px = width_px_;
  height_px = height_px_;
  use_ebo = use_ebo_;

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

  // Enable alpha blending for transparency
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Area in GLFW window where OpenGL rendering is performed
  // glfwGetFramebufferSize(window, &width_px, &height_px);
  // Create a square viewport centered in the window
  int square_size = std::min(width_px, height_px);  // Use the smaller dimension
  std::cout << "[vis::GLWindow::Init] Square " << square_size << std::endl;
  int x_offset = (width_px - square_size) / 2;   // Center horizontally
  int y_offset = (height_px - square_size) / 2;  // Center vertically

  glViewport(x_offset, y_offset, square_size, square_size);
  // glViewport(0, 0, width_px, height_px);

  RegisterCallbacks();

  // Shaders
  if (!CreateVertexShader()) return false;
  if (!CreateFragmentShader()) return false;
  if (!CreateShaderProgram()) return false;

  // float vertices[] = {0.5f, 0.5f, 0.0f, 0.5f, -0.5f, 0.0f, -0.5f, -0.5f, 0.0f, -0.5f, 0.5f,
  // 0.0f};
  // float vertices[] = {
  //     // positions        // colors           // tex coords
  //     0.5f,  0.5f,  0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f,  // 0: top right
  //     0.5f,  -0.5f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f,  // 1: bottom right
  //     -0.5f, -0.5f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,  // 2: bottom left
  //     -0.5f, 0.5f,  0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f   // 3: top left
  // };

  // And adjust your vertices:
  float vertices[] = {
      // positions        // colors           // tex coords (flipped Y)
      0.5f,  0.5f,  0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f,  // 0: top right
      0.5f,  -0.5f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f,  // 1: bottom right
      -0.5f, -0.5f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f,  // 2: bottom left
      -0.5f, 0.5f,  0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f   // 3: top left
  };

  if (use_ebo) {
    // unsigned int indices[] = {0, 1, 3, 1, 2, 3};
    // To:
    unsigned int indices[] = {
        0, 1, 2,  // First triangle: top-right, bottom-right, bottom-left
        0, 2, 3   // Second triangle: top-right, bottom-left, top-left
    };
    // unsigned int indices[] = {
    //     0, 1, 2,  // Triangle 1: top-right, bottom-right, bottom-left
    //     0, 2, 3   // Triangle 2: top-right, bottom-left, top-left
    // };
    std::cout << "=== INDICES ===" << std::endl;
    std::cout << "Triangle 1: " << indices[0] << ", " << indices[1] << ", " << indices[2]
              << std::endl;
    std::cout << "Triangle 2: " << indices[3] << ", " << indices[4] << ", " << indices[5]
              << std::endl;
    std::cout << "===============" << std::endl;
    CreateVertexAttributeObject(vertices, indices);
  } else {
    CreateVertexAttributeObject(vertices, nullptr);
  }

  CreateTexture(50, 50);

  return true;
}

bool vis::GLWindow::Update() {
  // Add the simulation logic here
  SetScreenColor();
  Render();

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