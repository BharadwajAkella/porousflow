#version 430

layout (location=0) in vec3 VertexPosition;
layout (location=1) in vec2 VertexTexCoord;
layout (location=2) in vec3 VertexNormal;
layout (location=3) in vec4 VertexColor;

out vec4 color;
out vec2 texCoord;
out vec3 normal;
out vec3 position;

uniform mat4 mvpMatrix;

void main()
{
	color = VertexColor;
	texCoord = VertexTexCoord;
	normal = VertexNormal;
	position = VertexPosition;
    gl_Position = mvpMatrix*vec4(VertexPosition, 1.0);
}
