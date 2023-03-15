#version 330
layout(location = 0) in vec3 worldPosition;
layout(location = 1) in vec3 localPosition;
layout(location = 2) in vec4 color;
layout(location = 3) in float thickness;
layout(location = 4) in float fade;

layout(std140, binding = 0) uniform camera
{
    mat4 viewProjection;
};

struct VertexOutput
{
    vec3 localPosition;
    vec4 color;
    vec2 thickness;
    float fade;
};

layout(location = 0) out VertexOutput output;
layout(location = 3) out flat float texIndex;

void main()
{
    output.color = color;
    output.texCoords = 
}

