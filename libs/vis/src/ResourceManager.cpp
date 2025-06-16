// std
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include <map>

// extern libs
#include <glad/glad.h>  // ORDER MATTERS (bw glad and glfw)
#include <GLFW/glfw3.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// self
#include "SpriteRenderer.h"
#include "ResourceManager.h"
#include "Shader.h"
#include "Texture.h"

std::map<std::string, vis::Texture2D> vis::ResourceManager::Textures;
std::map<std::string, vis::Shader> vis::ResourceManager::Shaders;

vis::Shader vis::ResourceManager::LoadShader(const char* vShaderFile, const char* fShaderFile,
                                             std::string name) {
  Shaders[name] = loadShaderFromFile(vShaderFile, fShaderFile);
  return Shaders[name];
}

vis::Shader vis::ResourceManager::GetShader(std::string name) { return Shaders[name]; }

vis::Texture2D vis::ResourceManager::LoadTexture(const char* file, bool alpha, std::string name) {
  Textures[name] = loadTextureFromFile(file, alpha);
  return Textures[name];
}

vis::Texture2D vis::ResourceManager::GetTexture(std::string name) { return Textures[name]; }

void vis::ResourceManager::Clear() {
  // (properly) delete all shaders
  for (auto iter : Shaders) glDeleteProgram(iter.second.ID);
  // (properly) delete all textures
  for (auto iter : Textures) glDeleteTextures(1, &iter.second.ID);
}

vis::Shader vis::ResourceManager::loadShaderFromFile(const char* vShaderFile,
                                                     const char* fShaderFile) {
  // 1. retrieve the vertex/fragment source code from filePath
  std::string vertexCode;
  std::string fragmentCode;
  std::string geometryCode;
  try {
    // open files
    std::ifstream vertexShaderFile(vShaderFile);
    std::ifstream fragmentShaderFile(fShaderFile);
    std::stringstream vShaderStream, fShaderStream;
    // read file's buffer contents into streams
    vShaderStream << vertexShaderFile.rdbuf();
    fShaderStream << fragmentShaderFile.rdbuf();
    // close file handlers
    vertexShaderFile.close();
    fragmentShaderFile.close();
    // convert stream into string
    vertexCode = vShaderStream.str();
    fragmentCode = fShaderStream.str();
  } catch (std::exception e) {
    std::cout << "ERROR::SHADER: Failed to read shader files" << std::endl;
  }
  const char* vShaderCode = vertexCode.c_str();
  const char* fShaderCode = fragmentCode.c_str();
  // 2. now create shader object from source code
  Shader shader;
  shader.Compile(vShaderCode, fShaderCode);
  return shader;
}

vis::Texture2D vis::ResourceManager::loadTextureFromFile(const char* file, bool alpha) {
  // create texture object
  Texture2D texture;
  if (alpha) {
    texture.Internal_Format = GL_RGBA;
    texture.Image_Format = GL_RGBA;
  }
  // load image
  int width, height, nrChannels;
  unsigned char* data = stbi_load(file, &width, &height, &nrChannels, 0);
  if (nrChannels == 3) {
    texture.Internal_Format = GL_RGB;
    texture.Image_Format = GL_RGB;
  } else if (nrChannels == 4) {
    texture.Internal_Format = GL_RGBA;
    texture.Image_Format = GL_RGBA;
  }
  
  // now generate texture
  texture.Generate(width, height, data);
  // and finally free image data
  stbi_image_free(data);
  return texture;
}