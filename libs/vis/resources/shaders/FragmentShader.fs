#version 330 core
out vec4 FragColor;

in vec2 TexCoord;
uniform sampler2D ourTexture;

void main() {
  vec4 texColor = texture(ourTexture, TexCoord);

  // // Discard white/nearly white pixels
  // if (texColor.r > 0.98 && texColor.g > 0.98 && texColor.b > 0.98) {
  //     discard;
  // }

  FragColor = vec4(texColor.r, texColor.g, texColor.b, 1.0);
}