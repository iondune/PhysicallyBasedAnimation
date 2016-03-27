#version 150

in vec3 fNormal;
in vec3 fEye;

uniform vec3 uColor;
uniform int uFlipNormals;

out vec4 outColor;


void main()
{
	outColor = vec4(uColor * clamp(dot(normalize(fEye), normalize(uFlipNormals != 0 ? -fNormal : fNormal)), 0.0, 1.0), 1.0);
}
