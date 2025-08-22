#ifndef PATH_RENDERER_H
#define PATH_RENDERER_H

#include <glad/glad.h>  // ORDER MATTERS (bw glad and glfw)
#include <GLFW/glfw3.h>

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"

#include "Waypoint.h"

namespace vis {
class PathRenderer {
 public:
  PathRenderer();
  void Init();  // Initialize OpenGL resources after context is ready
  void Render();
  void UpdatePath();
  void SetPath(state::Path &path, glm::vec3 color);
  void SetPathVisible(bool visible);
  void ClearPath();

  ~PathRenderer();

 private:
  unsigned int pathVAO;
  unsigned int pathVBO;
  bool pathVisible;
  bool initialized;  // Track if OpenGL resources are initialized
  state::Path path;
  glm::vec3 pathColor;
  int pathVertexCount;
};
}  // namespace vis

#endif