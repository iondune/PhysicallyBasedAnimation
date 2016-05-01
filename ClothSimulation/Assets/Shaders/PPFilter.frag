
#version 330

in vec2 fTexCoords;

uniform sampler2D uTexture;

out vec4 outColor;


void main()
{
	vec2 tex_offset = 1.0 / textureSize(uTexture, 0); // gets size of single texel
	vec3 SceneColor;

	for (int i = 0; i < 2; ++ i)
	{
		for (int j = 0; j < 2; ++ j)
		{
			vec3 Sample = texture(uTexture, fTexCoords - tex_offset * vec2(i, j)).rgb;
			SceneColor.r = max(SceneColor.r, Sample.r);
			SceneColor.g = max(SceneColor.g, Sample.g);
			SceneColor.b = max(SceneColor.b, Sample.b);
		}
	}

	const float FilterValue = 0.4;
	SceneColor = SceneColor - vec3(1.0 - FilterValue);
	SceneColor = clamp(SceneColor * (1.0 / FilterValue), vec3(0.0), vec3(1.0));

	outColor = vec4(SceneColor, 1.0);
}
