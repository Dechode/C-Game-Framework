#version 330 core

out vec4 fragColor;
in vec2 uv;
// in vec2 pos;

uniform vec4 color;
uniform sampler2D textureID;

void main()
{
	fragColor.rg = uv;
	fragColor.b = 0.0;
	fragColor.a = 0.0;
	float distance = 1.0 - length(uv * 2.0 - 1.0);
	distance = smoothstep(0.0, 0.03, distance);
	if (distance > 0.0)
		fragColor.a = 1.0;
//	fragColor = texture(textureID, uv) * color;
	fragColor.rgb = vec3(distance);
}
