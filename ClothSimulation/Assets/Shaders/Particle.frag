#version 150

in vec2 fTexCoords;
in float fAlpha;

uniform sampler2D uTexture;

out vec4 outColor;

void main()
{
	outColor = texture(uTexture, fTexCoords) * fAlpha;
}
