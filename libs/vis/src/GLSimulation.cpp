// cpp std libs
#include <iostream>
#include <filesystem>
#include <algorithm>

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
  glViewport(0, 0, util::MmToPixels(SoccerField::GetInstance().width_mm),
             util::MmToPixels(SoccerField::GetInstance().height_mm));
  std::cout << "Window Size: " << util::MmToPixels(SoccerField::GetInstance().width_mm) << " "
            << util::MmToPixels(SoccerField::GetInstance().height_mm) << std::endl;
  // glViewport(0, 0, cfg::Coordinates::window_width_px,
  // cfg::Coordinates::window_height_px);
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
  Eigen::Vector3d window_position(-cfg::Coordinates::window_width_px / 2,
                                  -cfg::Coordinates::window_height_px / 2, 0);
  Eigen::Vector2d window_size(cfg::Coordinates::window_width_px,
                              cfg::Coordinates::window_height_px);

  // Robots and Ball
  for (const auto& soccer_object : soccer_objects) {
    if (soccer_object.name == "ball") {
      game_objects["ball"] =
          GameObject(soccer_object.name, soccer_object.position, soccer_object.size,
                     soccer_object.velocity, soccer_object.acceleration, soccer_object.mass_kg,
                     ResourceManager::GetTexture("ball"), Eigen::Vector3d(1.0, 1.0, 1.0));
    } else if (soccer_object.name.find("robot") != std::string::npos) {
      if (soccer_object.name == "robot1") {
        game_objects[soccer_object.name] =
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