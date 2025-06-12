#version 330 core
out vec4 frag_color;
uniform vec4 robot_color;

void main() {
  frag_color = robot_color;
}