#include <glad/glad.h>  // generated loader
#include <GLFW/glfw3.h>

#include <iostream>

int main() {
  /* ---------- Initialise GLFW ---------- */
  if (!glfwInit()) {
    std::cerr << "Couldn’t initialise GLFW\n";
    return -1;
  }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);  // required on macOS
#endif

  GLFWwindow* win = glfwCreateWindow(800, 600, "Simple OpenGL Window", nullptr, nullptr);
  if (!win) {
    std::cerr << "Couldn’t create window\n";
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(win);
  glfwSwapInterval(1);  // enable vsync (optional)

  /* ---------- Load OpenGL symbols ---------- */
  if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
    std::cerr << "Couldn’t load GL functions\n";
    return -1;
  }

  /* ---------- Main loop ---------- */
  while (!glfwWindowShouldClose(win)) {
    glfwPollEvents();

    glClearColor(0.10f, 0.20f, 0.30f, 1.0f);  // dark blue/grey
    glClear(GL_COLOR_BUFFER_BIT);

    glfwSwapBuffers(win);
  }

  /* ---------- Clean up ---------- */
  glfwDestroyWindow(win);
  glfwTerminate();
  return 0;
}
