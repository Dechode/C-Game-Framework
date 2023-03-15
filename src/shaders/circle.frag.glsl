#version 330 core
layout(location = 0) out vec4 color;
layout(location = 1) out int color2;

struct VertexOutput
{
		vec3 localPosition;
		vec4 color;
		float thickness;
		float fade;
};

layout(location = 0) in VertexOutput input;

void main()
{
	vec3 circleColor = vec3(0.85, 0.35, 0.2);
	
}
