#ifndef FRAMEWORK_SPH_PARTICLE_H
#define FRAMEWORK_SPH_PARTICLE_H

#include <iostream>
#include <glm/glm.hpp>
#include <vector>
#include <string>
#include "Vertex.h"
#include "neighbordata.h"
#include <libconfig.h++>

class SPHParticle : public Vertex{
public:
	SPHParticle(){}


	//private:
	int pIndex;
	float	density,
		restDensity;
	float	viscocity;
	float	volume;

	float pressure;
	float mass;
	float massChange;
	float radius, h, deltaTime;

	glm::vec3	g;
	glm::vec3	velocity;

	//std::vector<SPHParticle>* particles;
	std::vector<std::vector<NeighborData> >* neighbors;
	
};


#endif