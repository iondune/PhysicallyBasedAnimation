
#version 330

in vec2 fTexCoords;

uniform sampler2D uTexture;

out vec4 outColor;


void main()
{
	vec3 SceneColor = texture(uTexture, fTexCoords).rgb;
	const float FilterValue = 0.4;
	SceneColor = SceneColor - vec3(1.0 - FilterValue);
	SceneColor = clamp(SceneColor * (1.0 / FilterValue), vec3(0.0), vec3(1.0));
	outColor = vec4(SceneColor, 1.0);
	// outColor = vec4(vec3(1.0, 0.5, 0.2), 1.0);
}
