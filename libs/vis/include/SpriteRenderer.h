#ifndef GL_SPRITE_RENDERER_H
#define GL_SPRITE_RENDERER_H

#include <string>

#include "Shader.h"
#include "Texture.h"

/*
 This can draw any object to the screen
*/
namespace vis {

class SpriteRenderer {
 public:
  void Init(Shader& shader);
  ~SpriteRenderer();

  void DrawSprite(const Texture2D& texture, glm::vec2 position, glm::vec2 size,
                  float rotate = 0.0f, glm::vec3 color = glm::vec3(1.0f));

 private:
  Shader shader;
  unsigned int quadVAO;

  void InitRenderData();
};

}  // namespace vis

#endif  // GL_SPRITE_RENDERER_H