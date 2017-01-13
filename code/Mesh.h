/*
 * Mesh.h
 *
 *  Created on: Sep 24, 2014
 *      Author: bharadwaj
 */

#ifndef MESH_H_
#define MESH_H_

#include  <iostream>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <vector>
#include <GL/glew.h>
#include <GL/freeglut.h>
#include "Vertex.h"

class Mesh {
public:
	Mesh();
	bool loadModel(char* filepath);
	std::vector<Vertex> meshVertices;
	std::vector<GLubyte> meshIndices;
};

#endif /* MESH_H_ */
