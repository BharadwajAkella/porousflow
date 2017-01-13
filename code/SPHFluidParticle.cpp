#include "SPHFluidParticle.h"
#include <libconfig.h++>


using glm::vec3;
using namespace std;
const float PI = 3.14592f;
const float accLim = 150.f;
const float velLim = 10.f;

using namespace libconfig;



void SPHFluidParticle::readConfig(){
	Config cfg;	
	cfg.readFile("fluidConfig.cfg");
	

	const Setting& root = cfg.getRoot();
	deltaTime = root["deltaTime"];
	k = root["k"];
	gamma = root["gamma"];
		
	viscocity = root["viscocity"];
	velocity = vec3(0, 0, 0);
	g = vec3(root["gx"], root["gy"], root["gz"]);

	pressure = 0.f;
	simplePressureUpdate = root["fluidPressureAlgo"];
	mass = root["fluidMass"];
	restDensity = root["restFluidDensity"];
	minNeighbors = root["minNeighbors"];
	density = restDensity;
	volume = mass / restDensity;//(4.0 / 3) * PI * radius * radius * radius;
	radius = pow(volume * (3.f / (4*PI)), (1.0/3)  );
	h = radius * 2;
		
		
	neighbors = 0;
	particles = 0;
	boundaryFriend = 0;

}

SPHFluidParticle::SPHFluidParticle(){
	readConfig();
}

void SPHFluidParticle::calculateViscousForce(){
	if (!onboundary){
		viscousForce = vec3(0, 0, 0);
		vec3 d2v = vec3(0, 0, 0);
		for (size_t i = 0; i < (*neighbors)[pIndex].size(); i++){
			int ipIndex = (*neighbors)[pIndex][i].idx;
			SPHFluidParticle* neighbor = &((*particles)[ipIndex]);
			if (neighbor != this && !(neighbor->onboundary)){
				float massN = neighbor->mass;
				float densityN = neighbor->density;
				vec3 velocityN = neighbor->velocity;
				vec3 posN = neighbor->pos;
				
				float d2W = viscosityLaplacianKernel(posN, h);

				if (densityN != 0)
					d2v += (massN / densityN)*(velocityN - velocity)*d2W;

			}
		}
		viscousForce += viscocity * d2v;
		acceleration += viscousForce / mass;
	}
}

void SPHFluidParticle::calculateBodyForce(){
	if (!onboundary){
		acceleration += g;
	}
}

void SPHFluidParticle::updateIntermediateVelocity(){
	if (!onboundary){
		velocity += deltaTime * (viscousForce + bodyForce) / mass;
	}
}

void SPHFluidParticle::updateVelocity(){
	if (!onboundary){
		velocity += deltaTime * acceleration;
		acceleration = vec3(0.f);
	}
}

void SPHFluidParticle::initDensity(){
	if (!onboundary){
		density = 0.f;
		float denom = 0.f;
		for (size_t i = 0; i < (*neighbors)[pIndex].size(); i++){
			int ipIndex = (*neighbors)[pIndex][i].idx;
			SPHFluidParticle* neighbor = &((*particles)[ipIndex]);
			if (neighbor != this){
				density += neighbor->mass * kernel(neighbor->pos, h);
				
			}
		}
		
	}
}

void SPHFluidParticle::updateDensity(){
	if (!onboundary && (*neighbors)[pIndex].size() > minNeighbors){
		density = 0.f;
		for (size_t i = 0; i < (*neighbors)[pIndex].size(); i++){
			int ipIndex = (*neighbors)[pIndex][i].idx;
			SPHFluidParticle* neighbor = &((*particles)[ipIndex]);
			if (neighbor != this){
				float densityInc = neighbor->mass * kernel(neighbor->pos, h);
				vec3 direction = glm::normalize(pos - neighbor->pos);
				/*vec3 dW = kernelGradient(neighbor->pos, h) * direction;
				densityInc += deltaTime * glm::dot((velocity - neighbor->velocity), dW);*/

				density += densityInc;				
			}
		}
		if (density <= 0.0001){
			cout << "here" << endl;
		}
	}
}

void SPHFluidParticle::updatePressure(){
	if (!onboundary){
		switch (simplePressureUpdate){
		default:
		case 1:
			pressure = k * (density - restDensity);
			break;
		case 2:
			pressure = k * (pow((density / restDensity), gamma) - 1);
			break;
		}		
	}
}

void SPHFluidParticle::calculatePressureForce(){
	if (!onboundary){
		vec3 dP = vec3(0, 0, 0);
		for (size_t i = 0; i < (*neighbors)[pIndex].size(); i++){
			int ipIndex = (*neighbors)[pIndex][i].idx;
			SPHFluidParticle* neighbor = &((*particles)[ipIndex]);

			if (neighbor != this){
				vec3 direction = glm::normalize(pos - neighbor->pos);
				float massN = neighbor->mass;
				float pressureN = neighbor->pressure;
				float densityN = neighbor->density;
				float dW = spikyKernelGradient(neighbor->pos, h);
				
				//dP = neighbor->mass * ((pressure / (density * density)) + (pressureN / (densityN * densityN))) * dW * direction;
				dP = -massN * ( (pressure + pressureN) / (densityN * 2) ) * dW * direction;
			}
		}
		/*dP = density * dP;
		pressureForce = -(mass / density) * dP;*/
		pressureForce = dP;
		acceleration += dP / mass;
	}
}

void SPHFluidParticle::updatePosition(){
	if (!onboundary){
		pos += deltaTime * velocity;
	}
}

void SPHFluidParticle::update(){
	
}

float SPHFluidParticle::kernel(vec3 position, float h){
	vec3 relPos = pos - position;
	float r = glm::dot(relPos, glm::normalize(relPos));
	float q = r / h;
	float retVal = 0.f;
	float sigma = 315.f / (PI * 64.f);
	if (q >= 0 && q < 1){
		retVal = (sigma / pow(h, 9)) * pow(h*h - r*r, 3);
	}
	else
		retVal = 0;
	return retVal;
}

float SPHFluidParticle::kernelGradient(vec3 position, float h){
	vec3 relPos = pos - position;
	float r = glm::dot(relPos, glm::normalize(relPos));
	float q = r / h;
	float sigma = 945.f / (PI * 32.f);
	return -((sigma * r) / pow(h, 9)) * pow(h*h - r*r, 2);
}

float SPHFluidParticle::kernelLaplacian(vec3 position, float h){
	vec3 relPos = pos - position;
	float r = glm::dot(relPos, glm::normalize(relPos));
	float q = r / h;
	float sigma = 945.f / (PI * 8.f);
	if (q >= 0 && q < 1){
		return (sigma / pow(h, 9)) * (h*h - r*r)* (r*r -0.75f*(h*h - r*r));
	}
	else
		return 0;
}

float SPHFluidParticle::spikyKernelGradient(vec3 position, float h){
	vec3 relPos = pos - position;
	float r = glm::dot(relPos, glm::normalize(relPos));
	float q = r / h;
	float sigma = -45 / PI;
	float retVal = (sigma / pow(h, 6)) * pow(h - r, 2);
	return retVal;
}

float SPHFluidParticle::viscosityLaplacianKernel(vec3 position, float h){
	vec3 relPos = pos - position;
	float r = glm::dot(relPos, glm::normalize(relPos));
	float q = r / h;
	float retVal = 0.f;
	float sigma = 45.f / (PI);
	retVal = (sigma / pow(h, 6)) * ( h - r);
	return retVal;
}