#include <filesystem>
#include <iostream>

// #define STB_IMAGE_IMPLEMENTATION
// #include "stb_image.h"
// #include "GLWindow.h"

// void vis::GLWindow::CreateTexture(int width_px, int height_px) {

//   // load and generate the texture
//   int width, height, nrChannels;

//   // stbi_set_flip_vertically_on_load(true);
//   unsigned char* data =
//       stbi_load("libs/vis/resources/pngegg.png", &width, &height, &nrChannels, 4);

//   if (data) {
//     std::cout << "channels: " << nrChannels << std::endl;
//     if (nrChannels == 3) {
//       glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
//       glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
//     } else if (nrChannels == 4) {
//       glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
//       glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
//     }
//     glGenerateMipmap(GL_TEXTURE_2D);
//   } else {
//     std::cout << "[vis::GLWindow::CreateTexture] Failed to load texture. Reason: "
//               << stbi_failure_reason() << std::endl;
//   }
// }