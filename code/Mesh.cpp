/*
 * Mesh.cpp
 *
 *  Created on: Sep 24, 2014
 *      Author: bharadwaj
 */

#include "Mesh.h"


#include <glm/gtc/matrix_transform.hpp>
using glm::vec3;
using glm::vec2;
using namespace std;

Mesh::Mesh() {

}

bool Mesh::loadModel(char* filename) {
	Assimp::Importer Importer;
	const aiScene* aiscene = Importer.ReadFile(filename, aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_FlipUVs);

	if (!aiscene) {
		std::cout << "couldnt load model " << std::endl;
		return false;
	} else {

		for (int j = 0; j < 1/*aiscene->mNumMeshes*/; j++) {
			const aiVector3D zeroVector(0.0f, 0.0f, 0.0f);
			const aiMesh* aimesh = aiscene->mMeshes[j];
			for (unsigned int i = 0; i < aimesh->mNumVertices; i++) {
				const aiVector3D* pos = &(aimesh->mVertices[i]);
				const aiVector3D* normal = &(aimesh->mNormals[i]);
				const aiVector3D* texCoord = aimesh->HasTextureCoords(0) ? &(aimesh->mTextureCoords[0][i]) : &zeroVector;

				Vertex v(vec3(pos->x, pos->y, pos->z),
					vec2(texCoord->x, texCoord->y),
					vec3(normal->x, normal->y, normal->z));

				meshVertices.push_back(v);
			}
			for (unsigned int i = 0; i < aimesh->mNumFaces; i++) {
				const aiFace& face = aimesh->mFaces[i];
				assert(face.mNumIndices == 3);
				meshIndices.push_back(face.mIndices[0]);
				meshIndices.push_back(face.mIndices[1]);
				meshIndices.push_back(face.mIndices[2]);
			}
		}
		return true;
	}

}

