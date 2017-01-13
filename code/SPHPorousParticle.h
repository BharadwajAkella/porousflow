#ifndef FRAMEWORK_SPH_POROUS_PARTICLE_H
#define FRAMEWORK_SPH_POROUS_PARTICLE_H

#include <iostream>
#include <glm/glm.hpp>
#include <vector>
#include <string>
#include "Vertex.h"
#include "SPHParticle.h"
#include "SPHFluidParticle.h"
#include "neighbordata.h"
#include <libconfig.h++>

class SPHFluidParticle;

class SPHPorousParticle : public SPHParticle{
public:
	//SPHPorousParticle(std::vector<std::vector<NeighborData> >* nbrs, std::vector<SPHPorousParticle*>* pcls);
	SPHPorousParticle::SPHPorousParticle();
	void updateFluidMass();
	void updateSaturation();
	void updateCapillaryPotential();
	void updatePorosity();
	void updateDensity();
	void updatePressure();
	void updateVelocity();
	glm::vec3 calculateCapillaryPotentialGradient();
	glm::vec3 calculatePressureGradient();
	glm::vec3 calculateVelocity();
	float calculateRateOfChangeOfMass();
	void diffuse();
	float kernel(glm::vec3 pos, float h);
	float kernelGradient(glm::vec3 pos, float h);
	float kernelLaplacian(glm::vec3 pos, float h);
	void updateProperties();
	void update();
	void readConfig();
	void conserveMass();

//private:
	int pIndex;
	float	porosity,
			restPorosity;
	
	float	permeability;
	float	fluidMass,
			fluidDensity;
	float	saturation;
	float	capillaryPotential;
	float massChange;
	short massChangeSign = 0;
	bool onboundary = false, trueBoundary = false, active = false;
	glm::vec3	g, newVelocity;
	float dmBydt = 0;
	int numActiveNeighbors = 0;
	std::vector<SPHPorousParticle>* particles;
	SPHFluidParticle* newFluidParticle = NULL;

	float	kc, 
			kp, 
			alpha, 
			beta, 
			gamma;
};


#endif