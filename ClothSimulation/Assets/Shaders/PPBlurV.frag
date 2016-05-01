
#version 330

in vec2 fTexCoords;

uniform sampler2D uTexture;
const float weight[5] = float[] (0.227027, 0.1945946, 0.1216216, 0.054054, 0.016216);

out vec4 outColor;


void main()
{
	vec2 tex_offset = 1.0 / textureSize(uTexture, 0); // gets size of single texel
	vec3 result = texture(uTexture, fTexCoords).rgb * weight[0]; // current fragment's contribution

	for(int i = 1; i < 5; ++i)
	{
		result += texture(uTexture, fTexCoords + vec2(0.0, tex_offset.y * i)).rgb * weight[i];
		result += texture(uTexture, fTexCoords - vec2(0.0, tex_offset.y * i)).rgb * weight[i];
	}

	outColor = vec4(result, 1.0);
	// outColor = vec4(vec3(1.0, 0.5, 0.2), 1.0);
}
