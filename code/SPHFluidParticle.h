#ifndef FRAMEWORK_SPH_FLUID_PARTICLE_H
#define FRAMEWORK_SPH_FLUID_PARTICLE_H

#include <iostream>
#include <glm/glm.hpp>
#include <vector>
#include <string>
#include "Vertex.h"
#include "SPHParticle.h"
#include "SPHPorousParticle.h"
#include "neighbordata.h"
#include <libconfig.h++>

class SPHPorousParticle;

class SPHFluidParticle : public SPHParticle{
public:
	SPHFluidParticle();

	void update();
	void readConfig();	
	void calculateViscousForce();
	void calculateBodyForce();
	void updateIntermediateVelocity();
	
	void updateDensity();
	void updatePressure();
	void calculatePressureForce();
	void updateVelocity();
	void initDensity();
	void updatePosition();


	float kernel(glm::vec3 pos, float h);
	float kernelGradient(glm::vec3 pos, float h);
	float kernelLaplacian(glm::vec3 pos, float h);
	float spikyKernelGradient(vec3 position, float h);
	float viscosityLaplacianKernel(vec3 position, float h);
	
	std::vector<SPHFluidParticle>* particles;
	SPHPorousParticle* boundaryFriend;
	glm::vec3 viscousForce, bodyForce, pressureForce, acceleration;
	float k , gamma;
	bool onboundary = false, active = true;
	int simplePressureUpdate;
	int minNeighbors;
};


#endif