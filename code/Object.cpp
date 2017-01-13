#include "Object.h"
#include <iostream>
using namespace std;

GLuint& Object::getVertexArrayHandle(){
	return this->vaoHandle;
}

GLuint& Object::getVertexBufferHandle(){
	return this->vboHandle;
}

Object::Object(Vertex* p_vertices, int numVertices, GLubyte* p_indices, int numIndices, GLenum p_mode){
	mode = p_mode;
	fillVertices(p_vertices, numVertices);

	if (p_indices != NULL){
		for (int i = 0; i < numIndices; i++){
			indices.push_back(p_indices[i]);
		}
		useIndices = true;
	}

}

Object::~Object(){

}

void Object::fillVertices(Vertex* p_vertices, int numVertices){
	vertices.clear();
	for (int i = 0; i < numVertices; i++){
		vertices.push_back(p_vertices[i]);
	}
}

void Object::render(){
	glBindVertexArray(vaoHandle);
	int count = 0;
	glPointSize(10);
	if (useIndices){
		glDrawElements(mode, indices.size(), GL_UNSIGNED_BYTE, NULL);
	}
	else{
		glDrawArrays(mode, 0, vertices.size());
	}
}

void Object::createHandles(){
	glGenVertexArrays(1, &vaoHandle);
	glBindVertexArray(vaoHandle);
	glGenBuffers(1, &vboHandle);
}

void Object::fillBuffers(){
	fillBuffers(this->vaoHandle, this->vboHandle, &(this->vertices[0]), this->vertices.size());
}

void Object::fillBuffers(GLuint& vao, GLuint& vbo, Vertex* Vertices, unsigned int size){
	if (size != vertices.size()){
		fillVertices(Vertices, size);
	}
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);
	glEnableVertexAttribArray(3);

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, size*sizeof(Vertex), Vertices, GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), 0);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid*)sizeof(vec3));
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid*)(sizeof(vec3)+sizeof(vec2)));
	glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid*)(sizeof(vec3)+sizeof(vec2)+sizeof(vec3)));

	if (useIndices){
		glGenBuffers(1, &indexVboHandle);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexVboHandle);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size(), &indices[0], GL_STATIC_DRAW);
	}
}
