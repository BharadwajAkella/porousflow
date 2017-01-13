#include "SPHFluidParticle.h"
#include <libconfig.h++>

using glm::vec3;
using namespace std;
const float PI = 3.14592f;

using namespace libconfig;



void SPHFluidParticle::readConfig(){
	Config cfg;	
	cfg.readFile("fluidConfig.cfg");
	

	const Setting& root = cfg.getRoot();
	deltaTime = root["deltaTime"];
	k = root["k"];
	gamma = root["gamma"];
		
	viscocity = root["viscocity"];
	surfaceTension = root["surfaceTension"];
	velocity = vec3(0, 0, 0);
	g = vec3(root["gx"], root["gy"], root["gz"]);

	pressure = 0.f;
	simplePressureUpdate = root["fluidPressureAlgo"];
	mass = root["fluidMass"];
	restDensity = root["restFluidDensity"];
	density = restDensity;
	volume = mass / restDensity;//(4.0 / 3) * PI * radius * radius * radius;
	radius = pow(volume * (3.f / (4*PI)), (1.0/3)  );
	h = radius * 4;
		
		
	neighbors = 0;
	particles = 0;
	boundaryFriend = 0;
	calculateBodyForce();

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
			SPHFluidParticle* ithParticle = &((*particles)[ipIndex]);
			if (ithParticle != this && !(ithParticle->onboundary)){
				float massJ = ithParticle->mass;
				float densityJ = ithParticle->density;
				vec3 velocityJ = ithParticle->velocity;
				float d2W = viscosityLaplacianKernel(ithParticle->pos, h);
				d2v += (massJ / densityJ)*(velocityJ - velocity)*d2W;
			}
		}
		viscousForce += /*mass * */ viscocity * d2v;
		cout << pIndex << " 's v force = " << viscousForce.x << ", " << viscousForce.y << ", " << viscousForce.z << endl;

	}
}

void SPHFluidParticle::calculateBodyForce(){
	if (!onboundary){
		bodyForce = mass*g;
	}
}

void SPHFluidParticle::updateIntermediateVelocity(){
	if (!onboundary){
		velocity += deltaTime * (viscousForce + bodyForce) / mass;
	}
}

void SPHFluidParticle::updateVelocity(){
	if (!onboundary){
		velocity += deltaTime * pressureForce / mass;
	}
}

void SPHFluidParticle::initDensity(){
	if (!onboundary){
		density = 0.f;
		float denom = 0.f;
		for (size_t i = 0; i < (*neighbors)[pIndex].size(); i++){
			int ipIndex = (*neighbors)[pIndex][i].idx;
			SPHFluidParticle* ithParticle = &((*particles)[ipIndex]);
			if (ithParticle != this && !(ithParticle->onboundary)){
				density += ithParticle->mass * kernel(ithParticle->pos, h);
				//vec3 dW = kernelGradient(ithParticle->pos, h) ;
				//density += deltaTime * glm::dot((velocity - ithParticle->velocity), dW);
			}
		}
		
	}
}

void SPHFluidParticle::updateDensity(){
	if (!onboundary && (*neighbors)[pIndex].size() != 0){
		density = 0.f;
		for (size_t i = 0; i < (*neighbors)[pIndex].size(); i++){
			int ipIndex = (*neighbors)[pIndex][i].idx;
			SPHFluidParticle* ithParticle = &((*particles)[ipIndex]);
			if (ithParticle != this && !(ithParticle->onboundary)){
				float densityInc = ithParticle->mass * kernel(ithParticle->pos, h);
				density += densityInc;				
			}
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
			SPHFluidParticle* ithParticle = &((*particles)[ipIndex]);
			if (ithParticle != this && !(ithParticle->onboundary)){
				vec3 direction = glm::normalize(pos - ithParticle->pos);
				//dP = ithParticle->mass * ((pressure / (density * density)) + (ithParticle->pressure / (ithParticle->density * ithParticle->density))) * spikyKernelGradient(ithParticle->pos, h) * direction;
				dP = -ithParticle->mass * ( (pressure + ithParticle->pressure) / (ithParticle->density * 2) ) * spikyKernelGradient(ithParticle->pos, h) ;
			}
		}
		/*dP = density * dP;
		pressureForce = -(mass / density) * dP;*/
		pressureForce = dP;
		cout << pIndex << " 's p force = " << pressureForce.x << ", " << pressureForce.y << ", " << pressureForce.z << endl;
	}
}

void SPHFluidParticle::step1(){
	if (!onboundary){
		pressureForce = vec3(0.f);
		viscousForce = vec3(0.f);
		vec3 cfGrad = vec3(0.f);
		cfGradDirection = vec3(0.f);
		cfLaplacian = 0.f;

		for (size_t i = 0; i < (*neighbors)[pIndex].size(); i++){
			int ipIndex = (*neighbors)[pIndex][i].idx;
			SPHFluidParticle* neighbor = &((*particles)[ipIndex]);
			if (neighbor != this && !(neighbor->onboundary)){
				
				float densityN = neighbor->density;
				float pressureN = neighbor->pressure;
				float massN = neighbor->mass;
				vec3 velocityN = neighbor->velocity;

				updatePressure();
				neighbor->updatePressure();

				pressureForce += massN * (pressure + pressureN) / (2 * densityN) * spikyKernelGradient(neighbor->pos, h);

				viscousForce += massN * (velocityN - velocity) / densityN * viscosityLaplacianKernel(neighbor->pos, h);

				cfGrad += massN / densityN * kernelGradient(neighbor->pos, h);

				cfLaplacian += massN / densityN * kernelLaplacian(neighbor->pos, h);

			}
		}
		viscousForce = viscocity * viscousForce;
		
		cfGradDirection = glm::normalize(cfGrad);
		cfGradLength = glm::dot(cfGrad, cfGradDirection);
	}
}

void SPHFluidParticle::step2(){
	
}

void SPHFluidParticle::updatePosition(){
	if (!onboundary){
		velocity += deltaTime * pressureForce / mass;
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

vec3 SPHFluidParticle::kernelGradient(vec3 position, float h){
	vec3 relPos = pos - position;
	float r = glm::dot(relPos, glm::normalize(relPos));
	float q = r / h;
	float sigma = 945.f / (PI * 32.f);
	return -((sigma * relPos) / pow(h, 9)) * pow(h*h - r*r, 2);

}

float SPHFluidParticle::kernelLaplacian(vec3 position, float h){
	vec3 relPos = pos - position;
	float r = glm::dot(relPos, glm::normalize(relPos));
	float q = r / h;
	float sigma = 945.f / (PI * 8.f);
	return (sigma / pow(h, 9)) * (h*h - r*r)* (r*r -0.75f*(h*h - r*r));
}

vec3 SPHFluidParticle::spikyKernelGradient(vec3 position, float h){
	vec3 relPos = pos - position;
	float r = glm::dot(relPos, glm::normalize(relPos));
	float q = r / h;
	float sigma = 1 / PI;
	
	return -( (45 * sigma * relPos ) / (pow(h, 6))*r ) * pow(h - r, 2);


}

float SPHFluidParticle::viscosityLaplacianKernel(vec3 position, float h){
	vec3 relPos = pos - position;
	float r = glm::dot(relPos, glm::normalize(relPos));
	float q = r / h;
	float retVal = 0.f;
	float sigma = 45.f / (PI);
	if (q >= 0 && q < 1){
		retVal = ((sigma * r) / pow(h, 5)) * ( 1 - q );
	}
	else
		retVal = 0;
	return retVal;
}