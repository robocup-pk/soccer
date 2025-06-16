// cpp std libs
#include <iostream>
#include <filesystem>
#include <algorithm>

// self libs
#include "Collision.h"
#include "GLConfig.h"
#include "GLWindow.h"
#include "ResourceManager.h"
#include "GLCallback.h"

bool vis::GLWindow::RunSimulationStep(float dt) {
  ProcessInput(dt);
  for (auto& [name, game_object] : game_objects) {
    if (name == "background") continue;
    game_object.Move(dt);
  };
  CheckAndResolveCollisions(game_objects);
  Render(dt);
  return Update();
}

void vis::GLWindow::Render(float dt) {
  glClearColor(1.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);
  for (auto& [name, game_object] : game_objects) {
    if (name == "robot0") {
      game_object.Draw(renderer, glm::vec2(vis::GLCallback::x_offset_robot0_worldf,
                                           vis::GLCallback::y_offset_robot0_worldf));
    } else if (name == "robot1") {
      game_object.Draw(renderer, glm::vec2(vis::GLCallback::x_offset_robot1_worldf,
                                           vis::GLCallback::y_offset_robot1_worldf));
    } else {
      game_object.Draw(renderer);
    }
  }
}

bool vis::GLWindow::Update() {
  if (glfwWindowShouldClose(window)) return false;
  glfwSwapBuffers(window);  // Large color buffer for each pixel in glfw window
  glfwPollEvents();         // Checks if any event is triggered. Updates state. Callbacks
  return true;
}

vis::GLWindow::GLWindow(int width_px, int height_px, const char* window_title) {
  // Initialize GLFW
  if (!glfwInit()) {
    std::cout << "[vis::GLWindow::Init] Couldn’t initialise GLFW\n";
    return;
  }

  // Configure GLFW options
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);  // required on macOS
#endif

  // Create a window
  window = glfwCreateWindow(width_px, height_px, window_title, nullptr, nullptr);
  if (!window) {
    std::cout << "[vis::GLWindow::Init] Couldn’t create window\n";
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);  // enable vsync (optional)

  // GLAD (manages function pointers for opengl)
  if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
    std::cout << "[vis::GLWindow::Init] Couldn’t load GL functions\n";
    return;
  }

  // Enable alpha blending for transparency
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  // Area in GLFW window where OpenGL rendering is performed
  glViewport(0, 0, width_px, height_px);

  RegisterCallbacks();
  InitGameObjects();
}

void vis::GLWindow::InitGameObjects() {
  // Load Shaders
  ResourceManager::LoadShader("libs/vis/resources/shaders/sprite.vs",
                              "libs/vis/resources/shaders/sprite.fs", "sprite");
  ResourceManager::GetShader("sprite").Use().SetInteger("sprite", 0);

  // sprite Renderer
  Shader shader = ResourceManager::GetShader("sprite");
  shader.Use();

  renderer.Init(shader);

  // Load Textures
  ResourceManager::LoadTexture("libs/vis/resources/textures/happyface.png", false, "face");
  ResourceManager::LoadTexture("libs/vis/resources/textures/background.jpg", false, "background");
  ResourceManager::LoadTexture("libs/vis/resources/textures/ball.png", false, "ball");

  // Window
  game_objects["background"] = GameObject(
      "background",
      glm::vec2(-vis::GLConfig::window_width_px / 2, -vis::GLConfig::window_height_px / 2),
      glm::vec2(vis::GLConfig::window_width_px, vis::GLConfig::window_height_px), glm::vec2(0, 0),
      glm::vec2(0,0), ResourceManager::GetTexture("background"));

  // Robots
  for (int i = 0; i < vis::GLConfig::num_robots; ++i) {
    std::string name = "robot" + std::to_string(i);
    game_objects[name] =
        GameObject(name, glm::vec2(0, 130 + -350 * i), vis::GLConfig::robot_size_cm,
                   glm::vec2(0, 0), vis::GLConfig::init_robot_acceleration, ResourceManager::GetTexture("face"));
  }

  // Ball
  game_objects["ball"] =
      GameObject("ball", vis::GLConfig::init_ball_pos, vis::GLConfig::ball_radius_cm,
                 vis::GLConfig::init_ball_velocity, vis::GLConfig::init_ball_acceleration, ResourceManager::GetTexture("ball"));
}

GLFWwindow* vis::GLWindow::GetRawGLFW() const { return window; }

vis::GLWindow::~GLWindow() {
  glfwTerminate();  // cleans all the resources taken by glfw
  // TODO: free the window ptr
}