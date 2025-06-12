#version 330 core
layout (location = 0) in vec3 aPos;

uniform vec2 offset;
out vec4 vertexColor;

void main() {
  gl_Position = vec4(aPos.x + offset.x, aPos.y + offset.y, aPos.z, 1.0);
  vertexColor = vec4(0.5, 0.0, 0.0, 1.0);
}