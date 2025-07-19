// cpp std libs
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <cassert>

// self libs
#include "Kinematics.h"
#include "GLConfig.h"
#include "GLSimulation.h"
#include "ResourceManager.h"
#include "GLCallback.h"
#include "Utils.h"
#include "SoccerField.h"

bool vis::GLSimulation::RunSimulationStep(std::vector<state::SoccerObject>& soccer_objects,
                                          float dt) {
  // Convert SoccerObjects to GameObjects
  for (auto& soccer_object : soccer_objects) {
    if (game_objects.find(soccer_object.name) == game_objects.end()) {
      std::cout << "[vs::GLSimulation::RunSimulationStep] Game Object " << soccer_object.name
                << " do not exist" << std::endl;
      return false;
    }
    UpdateGameObject(soccer_object);
  }

  // ProcessInput(dt);

  Render(dt);
  return Update();
}

void vis::GLSimulation::UpdateGameObject(const state::SoccerObject& soccer_object) {
  game_objects[soccer_object.name] = soccer_object;
}

void vis::GLSimulation::Render(float dt) {
  // Render Green Background
  glClearColor(0.0f, 0.5f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  // Render SoccerField::GetInstance() first (background)
  SoccerField::GetInstance().RenderField(this->window);

  for (auto& [name, game_object] : game_objects) {
    game_object.Draw(renderer);
  }
}

bool vis::GLSimulation::Update() {
  if (glfwWindowShouldClose(window)) return false;
  glfwSwapBuffers(window);  // Large color buffer for each pixel in glfw window
  glfwPollEvents();         // Checks if any event is triggered. Updates state. Callbacks
  return true;
}

std::map<std::string, vis::GameObject>& vis::GLSimulation::GetGameObjects() {
  return game_objects;
}

vis::GLSimulation::GLSimulation() {
  // Initialize GLFW
  if (!glfwInit()) {
    std::cout << "[vis::GLSimulation::Init] Couldn’t initialise GLFW\n";
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
  window = glfwCreateWindow(util::MmToPixels(SoccerField::GetInstance().width_mm),
                            util::MmToPixels(SoccerField::GetInstance().height_mm), "Soccer Field",
                            nullptr, nullptr);
  if (!window) {
    std::cout << "[vis::GLSimulation::Init] Couldn’t create window\n";
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);  // enable vsync (optional)

  // GLAD (manages function pointers for opengl)
  if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
    std::cout << "[vis::GLSimulation::Init] Couldn’t load GL functions\n";
    return;
  }

  // Enable alpha blending for transparency
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Area in GLFW window where OpenGL rendering is performed
#ifdef __APPLE__
  glViewport(0, 0, 2 * util::MmToPixels(SoccerField::GetInstance().width_mm),
             2 * util::MmToPixels(SoccerField::GetInstance().height_mm));
#else
  glViewport(0, 0, util::MmToPixels(SoccerField::GetInstance().width_mm),
             util::MmToPixels(SoccerField::GetInstance().height_mm));
#endif
  RegisterCallbacks();
}

void vis::GLSimulation::InitGameObjects(std::vector<state::SoccerObject>& soccer_objects) {
  // Load Shaders
  std::string vertex_shader_path =
      util::GetExecutableDir() + "/libs/vis/resources/shaders/sprite.vs";
  std::string fragment_shader_path =
      util::GetExecutableDir() + "/libs/vis/resources/shaders/sprite.fs";
  ResourceManager::LoadShader(vertex_shader_path.c_str(), fragment_shader_path.c_str(), "sprite");
  ResourceManager::GetShader("sprite").Use().SetInteger("sprite", 0);

  std::string field_vector_shader_path =
      util::GetExecutableDir() + "/libs/vis/resources/shaders/field.vs";

  std::string field_fragment_shader_path =
      util::GetExecutableDir() + "/libs/vis/resources/shaders/field.fs";
  ResourceManager::LoadShader(field_vector_shader_path.c_str(), field_fragment_shader_path.c_str(),
                              "field");
  ResourceManager::GetShader("field").Use().SetInteger("field", 0);

  SoccerField::GetInstance().SoccerFieldInit();

  // sprite Renderer
  Shader shader = ResourceManager::GetShader("sprite");
  shader.Use();

  renderer.Init(shader);

  // Load Textures
  std::string robot_texture_path =
      util::GetExecutableDir() + "/libs/vis/resources/textures/robot.png";
  std::string robot_texture_path_2 =
      util::GetExecutableDir() + "/libs/vis/resources/textures/ball.png";
  std::string ball_texture_path =
      util::GetExecutableDir() + "/libs/vis/resources/textures/ball.png";
  std::string arrow_texture_path =
      util::GetExecutableDir() + "/libs/vis/resources/textures/arrow.png";

  ResourceManager::LoadTexture(robot_texture_path.c_str(), false, "face");
  ResourceManager::LoadTexture(ball_texture_path.c_str(), false, "ball");
  ResourceManager::LoadTexture(arrow_texture_path.c_str(), false, "arrow");

  // Window
  Eigen::Vector3d window_position(-util::MmToPixels(SoccerField::GetInstance().width_mm) / 2,
                                  -util::MmToPixels(SoccerField::GetInstance().height_mm) / 2, 0);
  Eigen::Vector2d window_size(util::MmToPixels(SoccerField::GetInstance().width_mm),
                              util::MmToPixels(SoccerField::GetInstance().height_mm));

  // Robots and Ball
  for (const auto& soccer_object : soccer_objects) {
    if (soccer_object.name == "ball") {
      game_objects["ball"] =
          GameObject(soccer_object.name, soccer_object.position, soccer_object.size,
                     soccer_object.velocity, soccer_object.acceleration, soccer_object.mass_kg,
                     ResourceManager::GetTexture("ball"), Eigen::Vector3d(1.0, 1.0, 1.0));
    } else {
      game_objects[soccer_object.name] =
          GameObject(soccer_object.name, soccer_object.position, soccer_object.size,
                     soccer_object.velocity, soccer_object.acceleration, soccer_object.mass_kg,
                     ResourceManager::GetTexture("face"), Eigen::Vector3d(1.0, 1.0, 1.0));
    }
  }
}

GLFWwindow* vis::GLSimulation::GetRawGLFW() const { return window; }

vis::GLSimulation::~GLSimulation() {
  game_objects.clear();
  ResourceManager::Clear();

  if (window) {
    glfwDestroyWindow(window);
    window = nullptr;
  }

  glfwTerminate();
}

void vis::ProcessInput(GLFWwindow* gl_window, std::vector<rob::RobotManager>& robot_managers) {
  Eigen::Vector3d velocity_fBody_rob1(0, 0, 0);
  Eigen::Vector3d velocity_fBody_rob2(0, 0, 0);

  // Robot 1 (WSAD)
  if (glfwGetKey(gl_window, GLFW_KEY_W) == GLFW_PRESS) velocity_fBody_rob1.y() += 1;
  if (glfwGetKey(gl_window, GLFW_KEY_S) == GLFW_PRESS) velocity_fBody_rob1.y() -= 1;
  if (glfwGetKey(gl_window, GLFW_KEY_A) == GLFW_PRESS) velocity_fBody_rob1.x() -= 1;
  if (glfwGetKey(gl_window, GLFW_KEY_D) == GLFW_PRESS) velocity_fBody_rob1.x() += 1;
  if (glfwGetKey(gl_window, GLFW_KEY_C) == GLFW_PRESS) velocity_fBody_rob1.z() += 1;
  if (glfwGetKey(gl_window, GLFW_KEY_X) == GLFW_PRESS) velocity_fBody_rob1.z() -= 1;

  // Robot 2 (Arrow keys)
  if (robot_managers.size() > 1) {
    if (glfwGetKey(gl_window, GLFW_KEY_UP) == GLFW_PRESS) velocity_fBody_rob2.y() += 1;
    if (glfwGetKey(gl_window, GLFW_KEY_DOWN) == GLFW_PRESS) velocity_fBody_rob2.y() -= 1;
    if (glfwGetKey(gl_window, GLFW_KEY_LEFT) == GLFW_PRESS) velocity_fBody_rob2.x() -= 1;
    if (glfwGetKey(gl_window, GLFW_KEY_RIGHT) == GLFW_PRESS) velocity_fBody_rob2.x() += 1;
    if (glfwGetKey(gl_window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS) velocity_fBody_rob2.z() += 1;
    if (glfwGetKey(gl_window, GLFW_KEY_SLASH) == GLFW_PRESS) velocity_fBody_rob2.z() -= 1;

    velocity_fBody_rob2.normalize();
    robot_managers[1].SetBodyVelocity(velocity_fBody_rob2);
  }

  velocity_fBody_rob1.normalize();
  robot_managers[0].SetBodyVelocity(velocity_fBody_rob1);
}

void vis::ProcessInput(GLFWwindow* gl_window, std::vector<state::SoccerObject>& soccer_objects) {
  Eigen::Vector3d velocity_fBody_rob1(0, 0, 0);
  Eigen::Vector3d velocity_fBody_rob2(0, 0, 0);

  // Robot 1 (WSAD)
  if (glfwGetKey(gl_window, GLFW_KEY_W) == GLFW_PRESS) velocity_fBody_rob1.y() += 1.5;
  if (glfwGetKey(gl_window, GLFW_KEY_S) == GLFW_PRESS) velocity_fBody_rob1.y() -= 1.5;
  if (glfwGetKey(gl_window, GLFW_KEY_A) == GLFW_PRESS) velocity_fBody_rob1.x() -= 1.5;
  if (glfwGetKey(gl_window, GLFW_KEY_D) == GLFW_PRESS) velocity_fBody_rob1.x() += 1.5;
  if (glfwGetKey(gl_window, GLFW_KEY_C) == GLFW_PRESS) velocity_fBody_rob1.z() += 1.5;
  if (glfwGetKey(gl_window, GLFW_KEY_X) == GLFW_PRESS) velocity_fBody_rob1.z() -= 1.5;
  if (glfwGetKey(gl_window, GLFW_KEY_K) == GLFW_PRESS) {
    if (soccer_objects[soccer_objects.size() - 1].is_attached &&
        soccer_objects[soccer_objects.size() - 1].attached_to == &soccer_objects[0]) {
      kin::DetachBall(soccer_objects[soccer_objects.size() - 1], 6.5);
    }
  }

  // Robot 2 (Arrow keys)
  if (soccer_objects.size() > 2) {
    if (glfwGetKey(gl_window, GLFW_KEY_UP) == GLFW_PRESS) velocity_fBody_rob2.y() += 1.5;
    if (glfwGetKey(gl_window, GLFW_KEY_DOWN) == GLFW_PRESS) velocity_fBody_rob2.y() -= 1.5;
    if (glfwGetKey(gl_window, GLFW_KEY_LEFT) == GLFW_PRESS) velocity_fBody_rob2.x() -= 1.5;
    if (glfwGetKey(gl_window, GLFW_KEY_RIGHT) == GLFW_PRESS) velocity_fBody_rob2.x() += 1.5;
    if (glfwGetKey(gl_window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS) velocity_fBody_rob2.z() += 1.5;
    if (glfwGetKey(gl_window, GLFW_KEY_SLASH) == GLFW_PRESS) velocity_fBody_rob2.z() -= 1.5;
    if (glfwGetKey(gl_window, GLFW_KEY_L) == GLFW_PRESS) {
      if (soccer_objects[soccer_objects.size() - 1].is_attached &&
          soccer_objects[soccer_objects.size() - 1].attached_to == &soccer_objects[1]) {
        kin::DetachBall(soccer_objects[soccer_objects.size() - 1], 6.5);
      }
    }
    // if (glfwGetKey(gl_window, GLFW_KEY_SEMICOLON) == GLFW_PRESS) soccer_objects[1].PassBall();
    // if (glfwGetKey(gl_window, GLFW_KEY_M) == GLFW_PRESS) soccer_objects[1].GoHome();
    // velocity_fBody_rob2.normalize();
    soccer_objects[1].velocity = velocity_fBody_rob2;
  }

  // velocity_fBody_rob1.normalize();
  soccer_objects[0].velocity = velocity_fBody_rob1;
}

void vis::MouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    // Get window dimensions
    int window_width, window_height;
    glfwGetWindowSize(window, &window_width, &window_height);

    // Convert from pixel coordinates to world coordinates (meters)
    // 1. Convert pixels to millimeters
    float pixels_per_mm = util::PixelsPerMm();
    double x_mm = xpos / pixels_per_mm;
    double y_mm = ypos / pixels_per_mm;

    // 2. Convert from millimeters to meters
    double x_m = x_mm / 1000.0;
    double y_m = y_mm / 1000.0;

    // 3. Adjust coordinate system: center at field center, flip Y axis
    double field_width_m = vis::SoccerField::GetInstance().width_mm / 1000.0;
    double field_height_m = vis::SoccerField::GetInstance().height_mm / 1000.0;

    double world_x = x_m - (field_width_m / 2.0);   // Center X coordinate
    double world_y = (field_height_m / 2.0) - y_m;  // Center Y coordinate and flip Y axis
  }
}