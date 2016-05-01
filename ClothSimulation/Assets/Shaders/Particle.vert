#version 150

in vec2 vPosition;
in vec2 vTexCoords;
in vec3 vInstanceLocation;
in float vInstanceSize;
in float vInstanceRotation;
in float vInstanceAlpha;

uniform mat4 uModelMatrix;
uniform mat4 uViewMatrix;
uniform mat4 uProjectionMatrix;

out vec2 fTexCoords;
out float fAlpha;

void main()
{
	vec3 CameraRight = vec3(uViewMatrix[0][0], uViewMatrix[1][0], uViewMatrix[2][0]);
	vec3 CameraUp = vec3(uViewMatrix[0][1], uViewMatrix[1][1], uViewMatrix[2][1]);

	vec3 Position = (uModelMatrix * vec4(vInstanceLocation, 1.0)).xyz;
	vec2 Size = vec2(vInstanceSize);
	float Rotation = vInstanceRotation;

	vec2 VertexOffset = vec2(
		vPosition.x * cos(vInstanceRotation) - vPosition.y * sin(Rotation),
		vPosition.x * sin(vInstanceRotation) + vPosition.y * cos(Rotation));
	VertexOffset *= Size;

	vec3 Vertex =
		CameraRight * VertexOffset.x +
		CameraUp * VertexOffset.y;

	Vertex += Position;

	gl_Position = uProjectionMatrix * uViewMatrix * vec4(Vertex, 1.0);
	fTexCoords = vTexCoords;
	fAlpha = vInstanceAlpha;
}
