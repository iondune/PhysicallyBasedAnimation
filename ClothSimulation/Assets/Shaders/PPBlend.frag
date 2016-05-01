
#version 330

in vec2 fTexCoords;

uniform sampler2D uScene;
uniform sampler2D uHighPass;

out vec4 outColor;


void main()
{
	vec3 Scene = texture(uScene, fTexCoords).rgb;
	vec3 HighPass = texture(uHighPass, fTexCoords).rgb;
	outColor = vec4(clamp(Scene + HighPass * 3.0, vec3(0.0), vec3(1.0)), 1.0);
}
