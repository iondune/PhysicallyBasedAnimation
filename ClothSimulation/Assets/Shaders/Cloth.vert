#version 150

in vec3 vPosition;
in vec3 vNormal;

uniform mat4 uModelMatrix;
uniform mat4 uViewMatrix;
uniform mat4 uProjectionMatrix;
uniform vec3 uCameraPosition;

out vec3 fNormal;
out vec3 fEye;


void main()
{
	vec4 WorldPosition = uModelMatrix * vec4(vPosition, 1.0);

	gl_Position = uProjectionMatrix * uViewMatrix * WorldPosition;
	fEye = normalize(uCameraPosition - WorldPosition.xyz);
	fNormal = vNormal;
}
