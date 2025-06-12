#include <filesystem>
#include <iostream>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "GLWindow.h"

void vis::GLWindow::CreateTexture(int width_px, int height_px) {
  glGenTextures(1, &texture_id);
  glBindTexture(GL_TEXTURE_2D, texture_id);
  // set the texture wrapping/filtering options (on currently bound texture)
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  // load and generate the texture
  int width, height, nrChannels;

  // stbi_set_flip_vertically_on_load(true);
  unsigned char* data =
      stbi_load("libs/vis/resources/pngegg.png", &width, &height, &nrChannels, 4);

  if (data) {
    std::cout << "channels: " << nrChannels << std::endl;
    if (nrChannels == 3) {
      glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
    } else if (nrChannels == 4) {
      std::cout << "Loaded image: " << width << "x" << height << " with " << nrChannels
                << " channels" << std::endl;

      // Count transparent pixels for debugging
      int transparentCount = 0;
      int whiteCount = 0;

      for (int i = 0; i < width * height; i++) {
        int idx = i * 4;
        unsigned char r = data[idx + 0];
        unsigned char g = data[idx + 1];
        unsigned char b = data[idx + 2];
        unsigned char a = data[idx + 3];

        // Check if pixel is white
        if (r > 240 && g > 240 && b > 240) {
          whiteCount++;
          data[idx + 3] = 0;  // Make transparent
          transparentCount++;
        }

        // Debug first few pixels
        if (i < 5) {
          std::cout << "Pixel " << i << ": RGBA(" << (int)r << "," << (int)g << "," << (int)b
                    << "," << (int)a << ")";
          std::cout << " -> Alpha: " << (int)data[idx + 3] << std::endl;
        }
      }

      std::cout << "Found " << whiteCount << " white pixels, made " << transparentCount
                << " transparent" << std::endl;

      glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
    }
    glGenerateMipmap(GL_TEXTURE_2D);
  } else {
    std::cout << "[vis::GLWindow::CreateTexture] Failed to load texture. Reason: "
              << stbi_failure_reason() << std::endl;
  }
  stbi_image_free(data);

  glBindTexture(GL_TEXTURE_2D, texture_id);
}