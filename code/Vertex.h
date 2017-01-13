#ifndef VERTEX_H
#define VERTEX_H
#include "glm/glm.hpp"

using glm::vec2;
using glm::vec3;
using glm::vec4;

struct Vertex
{
	vec3 pos;
	vec2 texCoord;
	vec3 normal;
	vec4 color;
	Vertex() {
		pos = vec3(0.f);
		color = vec4(1.f);
	}

	Vertex(vec3 p_pos, vec2 p_tex, vec3 p_normal = vec3(1.f))
	{
		pos = p_pos;
		texCoord = p_tex;
		normal = p_normal;
		color = vec4(1.f);
	}

	Vertex(vec3 p_pos, vec4 p_color,vec2 p_tex, vec3 p_normal = vec3(1.f))
	{
		pos = p_pos;
		texCoord = p_tex;
		normal = p_normal;
		color = p_color;
	}
};

#endif