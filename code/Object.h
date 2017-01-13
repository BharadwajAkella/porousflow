#ifndef OBJECT_H
#define OBJECT_H

#include "glm/glm.hpp"
#include "Vertex.h"
#include <cmath>
#include <vector>
#include <GL/glew.h>
#include <GL/freeglut.h>
using std::vector;

using glm::vec3;

class Object{
private:	
	void createHandles();
	GLuint vaoHandle;
	GLuint vboHandle, indexVboHandle;
	int numFaces;
	GLenum mode;

	void CreateVertexBuffer(GLuint& vao, GLuint& vbo);
public:
	Object(Vertex* p_vertices, int numVertices, GLubyte* p_indices, int numIndices, GLenum p_mode);
	Object(GLenum p_mode){ mode = p_mode; }
	~Object();
	void fillBuffers(GLuint& vao, GLuint& vbo, Vertex* Vertices, unsigned int size);
	void fillBuffers();
	void render();
	void fillVertices(Vertex* p_vertices, int numVertices);
	GLuint& getVertexArrayHandle();
	GLuint& getVertexBufferHandle();
	vector<Vertex> vertices;
	vector<GLubyte> indices;
	bool useIndices = false;
};

#endif
