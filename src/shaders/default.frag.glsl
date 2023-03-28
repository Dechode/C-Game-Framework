#version 330 core

out vec4 fragColor;

in vec2 uv;

uniform vec4 color;
uniform sampler2D textureID;

void main() {
    fragColor = texture(textureID, uv) * color;
}
