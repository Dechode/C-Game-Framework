#version 330 core
out vec4 fragColor;

in vec2 uvs;

uniform vec4 color;
uniform sampler2D textureID;

void main() {
    fragColor = texture(textureID, uvs) * color;
}
