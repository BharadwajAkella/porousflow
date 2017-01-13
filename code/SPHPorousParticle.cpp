#include "SPHPorousParticle.h"
#include <math.h>

#include <cmath>

using glm::vec3;
using std::cout;
using std::endl;
using std::string;
const float PI = 3.14592f;
const float threshold = pow(10, -4);

using namespace libconfig;

inline float min(float a, float b){
	if (a < b)
		return a;
	return b;
}

inline float max(float a, float b){
	if (a > b)
		return a;
	return b;
}

void SPHPorousParticle::readConfig(){
	Config cfg;
	cfg.readFile("porousConfig.cfg");
	const Setting& root = cfg.getRoot();
	deltaTime = root["porousDeltaTime"];
	kc = root["kc"];
	kp = root["kp"];
	alpha = root["alpha"];
	beta = root["beta"];
	gamma = root["gamma"];
	radius = root["radius"];
	h = radius * 2;
	fluidDensity = root["fluidDensity"];
	density = root["density"];
	restDensity = root["restDensity"];
	porosity = root["porosity"];
	restPorosity = root["restPorosity"];
	volume = (4.0 / 3) * PI * radius * radius * radius;
	permeability = root["permeability"];
	viscocity = root["viscocity"];
	g = vec3(root["gx"], root["gy"], root["gz"]);
	mass = root["mass"];

	capillaryPotential = 0.f;
	pressure = 0.f;
	massChange = 0.f;
	saturation = 0.f;
	updateFluidMass();

	neighbors = 0;
	particles = 0;

}

SPHPorousParticle::SPHPorousParticle(){
	readConfig();
}

void SPHPorousParticle::updateFluidMass(){
	fluidMass = fluidDensity * porosity * volume * saturation;
}

void SPHPorousParticle::updateSaturation(){
	if (fluidMass <= fluidDensity * porosity * volume && fluidDensity * porosity * volume > 0){
		saturation = (fluidMass) / (fluidDensity * porosity * volume);
	}
}

void SPHPorousParticle::updateVelocity(){
	//velocity = calculateVelocity();
	velocity = newVelocity;
}

void SPHPorousParticle::updateDensity(){
	density = restDensity + saturation * porosity * fluidDensity;
}

void SPHPorousParticle::updateCapillaryPotential(){
	capillaryPotential = kc * pow(1 - saturation, alpha);
}

void SPHPorousParticle::updatePorosity(){
	//if (!onboundary)
		float porosity = restPorosity * (restDensity / density);
}

void SPHPorousParticle::updatePressure(){
	pressure = kp * saturation * (pow((density / restDensity), gamma) - 1);
}

/*
vec3 SPHPorousParticle::calculateCapillaryPotentialGradient(){
	vec3 capillaryPotentialGradient = vec3(0.f);
	//cout << "for particle " << pIndex << endl;
	for (size_t i = 0; i < (*neighbors)[pIndex].size(); i++){
		int jpIndex = (*neighbors)[pIndex][i].idx;
		SPHPorousParticle* j = &((*particles)[jpIndex]);		
		if (j != this){
			float volumeJ = j->volume;
			float capillaryPotentialJ = j->capillaryPotential;
			float dW = kernelGradient(j->pos, h);
			vec3 direction = glm::normalize(pos - j->pos);
			
			//cout << i << ") contribution = " << volumeJ * capillaryPotentialJ * dW <<
			//		"in the direction (" << direction.x << ", " << direction.y << ", " << direction.z << ") " << endl;
			
			capillaryPotentialGradient += volumeJ * capillaryPotentialJ * dW * direction - pow(j->saturation, beta)*density*g;
		}
	}
	return capillaryPotentialGradient;
}

vec3 SPHPorousParticle::calculatePressureGradient(){
	vec3 pressureGradient = vec3(0.f);
	for (size_t i = 0; i < (*neighbors)[pIndex].size(); i++){
		int jpIndex = (*neighbors)[pIndex][i].idx;
		SPHPorousParticle* j = &((*particles)[jpIndex]);
		if (j != this){
			float dW = kernelGradient(j->pos, h);
			float densityJ = j->density;
			float pressureJ = j->pressure;
			float volumeJ = j->volume;
			float massJ = j->mass;
			vec3 direction = glm::normalize(pos - j->pos);

			float temp = pressure / (density * density);
			float tempJ = pressureJ / (densityJ * densityJ);
			//pressureGradient += massJ * (temp + tempJ) * dW;
			pressureGradient += volumeJ * pressureJ * dW * direction;
		}
	}
	//pressureGradient = density * pressureGradient;
	//cout << "pressuregradient x,y = " << pressureGradient.x << ", " << pressureGradient.z << endl;
	return pressureGradient;
}

vec3 SPHPorousParticle::calculateVelocity(){
	vec3 pressureGradient = calculatePressureGradient();
	vec3 capillaryPotentialGradient = calculateCapillaryPotentialGradient();
	vec3 velocity = -(permeability / (porosity * viscocity)) * (pressureGradient - capillaryPotentialGradient);

	//cout << "capillaryPotentialGradient x,y = " << capillaryPotentialGradient.x << ", " << capillaryPotentialGradient.z << " \t";
	//cout << "pressuregradient x,y = " << pressureGradient.x << ", " << pressureGradient.z << " \t";
	//cout << "velocity = " << velocity.x << ", " << velocity.y << ", " << velocity.z << endl << endl;
	return velocity;
}
*/

void SPHPorousParticle::diffuse(){
	//if (!onboundary){
		newVelocity = vec3(0);
		float newDensity, newSaturation;
		dmBydt = 0;
		for (size_t i = 0; i < (*neighbors)[pIndex].size(); i++){
			int jpIndex = (*neighbors)[pIndex][i].idx;
			SPHPorousParticle* j = &((*particles)[jpIndex]);
			if (j != this){
				if (pIndex == 2 || jpIndex == 2)
					cout << "";
				float W = kernel(j->pos, h);
				float dW = kernelGradient(j->pos, h);
				float d2W = kernelLaplacian(j->pos, h);
				float densityJ = j->density;
				float pressureJ = j->pressure;
				float volumeJ = j->volume;
				float massJ = j->mass;
				float capillaryPotentialJ = j->capillaryPotential;
				vec3 direction = glm::normalize(pos - j->pos);
				vec3 pressureGradient = volumeJ * pressureJ * dW * direction;
				vec3 capillaryPotentialGradient = volumeJ * capillaryPotentialJ * dW * direction;
				float satFactor = pow(((saturation == 1) ? saturation - threshold : saturation), 5);
				vec3 velocityToNeighbor = -(permeability / (porosity * viscocity)) * (pressureGradient - capillaryPotentialGradient - density*g);
				float diffusionCoef = glm::dot(velocityToNeighbor, (-direction)) * pow(abs(saturation - j->saturation), beta);
				dmBydt = diffusionCoef * j->volume * j->fluidMass * d2W;

				float maxFluidMass = fluidDensity * porosity * volume;
				float maxFluidMassJ = j->fluidDensity * j->porosity * j->volume;

				float change = dmBydt *deltaTime;

				if (abs(dmBydt) > threshold){
					numActiveNeighbors++;
					j->active = true;
				}
				else{
					/*if (saturation < threshold )
						cout << "" << endl;*/
				}
				massChange -= change;
				j->massChange += change;

				float remainingCapacity = maxFluidMass - (fluidMass);
				float remainingCapacityJ = maxFluidMassJ - (j->fluidMass);
				float temp1, temp2, maxOvershoot;

				if (fluidMass + massChange < 0 || (j->fluidMass + j->massChange > maxFluidMassJ)){
					if (fluidMass + massChange < 0){
						temp1 = abs(fluidMass + massChange);
					}
					else{
						temp1 = 0;
					}
					if (j->fluidMass + j->massChange > maxFluidMassJ){
						temp2 = abs(j->fluidMass + j->massChange - maxFluidMassJ);
					}
					else{
						temp2 = 0;
					}
					maxOvershoot = max(temp1, temp2);
					j->massChange -= maxOvershoot;
					massChange += maxOvershoot;
					if (abs(change) > threshold)
						change += maxOvershoot;
				}
				else if (fluidMass + massChange > maxFluidMass || (j->fluidMass + j->massChange < 0)){
					if (fluidMass + massChange > maxFluidMass){
						temp1 = abs(fluidMass + massChange - maxFluidMass);
					}
					else{
						temp1 = 0;
					}
					if (j->fluidMass + j->massChange < 0){
						temp2 = abs(j->fluidMass + j->massChange);
					}
					else{
						temp2 = 0;
					}
					maxOvershoot = max(temp1, temp2);
					j->massChange += maxOvershoot;
					massChange -= maxOvershoot;
					if (abs(change) > threshold)
						change += maxOvershoot;
				}
				if (abs(massChange) < threshold)
					massChange = 0;
				if (abs(j->massChange) < threshold)
					j->massChange = 0;
				if ((fluidMass + massChange < 0) || (fluidMass + massChange > maxFluidMass) || (j->fluidMass + j->massChange < 0) || (j->fluidMass + j->massChange > maxFluidMassJ)){
					cout << "";
				}

				/*if (abs(change) > threshold){
					cout << pIndex << " change --> " << massChange << "\t fluid mass --> " << fluidMass << endl;
					cout << j->pIndex << " change --> " << j->massChange << "\t fluid mass --> " << j->fluidMass << endl << endl;
					}*/
				newVelocity += velocityToNeighbor;
			}
		}
	//}
}

float SPHPorousParticle::calculateRateOfChangeOfMass(){
	float dmBydt = 0;
	float fMass = 0.f;
	for (size_t i = 0; i < (*neighbors)[pIndex].size(); i++){
		int jpIndex = (*neighbors)[pIndex][i].idx;
		SPHPorousParticle* j = &((*particles)[jpIndex]);
		if (j != this){
			fMass += j->fluidMass;
			vec3 relativePosition = glm::normalize(j->pos - pos);
			float diffusion = glm::dot(j->velocity, relativePosition) * pow(j->saturation, beta);
			float d2W = kernelLaplacian(j->pos, h);

			dmBydt += diffusion * j->volume * j->fluidMass * d2W;

			if (dmBydt > threshold || dmBydt < -threshold){
				massChangeSign = 1;
			}
			//cout << " diffusion = " << diffusion << " , element = " << pow(j->saturation, beta) << endl;
		}
	}
	return dmBydt;
}

void SPHPorousParticle::update(){
	float maxFluidMass = fluidDensity * porosity * volume;
	float massToDistribute = 0.f;
	if (saturation < 0.2){
		//cout << "" << endl;
	}
	// if after update, more than available mass is removed
	if (fluidMass + massChange < 0){
		massToDistribute = fluidMass + massChange;
		massChange = -fluidMass;		
	}
	// if after update overflow happens
	if (fluidMass + massChange > maxFluidMass){
		massToDistribute = fluidMass + massChange - maxFluidMass;
		massChange = maxFluidMass - fluidMass;
	}
	if (massToDistribute){
		int neighSize = (*neighbors)[pIndex].size();
		for (size_t i = 0; i < neighSize; i++){
			int jpIndex = (*neighbors)[pIndex][i].idx;
			SPHPorousParticle* j = &((*particles)[jpIndex]);
			if (j->active){
				float finalMassChangeJ = j->massChange + (massToDistribute / numActiveNeighbors);
				if (finalMassChangeJ + j->fluidMass < 0)
					finalMassChangeJ = -(j->fluidMass);
				if (finalMassChangeJ + j->fluidMass > maxFluidMass)
					finalMassChangeJ = maxFluidMass - j->fluidMass;
				j->massChange = finalMassChangeJ;
				j->active = false;
			}

		}
	}
}

//void SPHPorousParticle::update(){
//
//	float dmBydt = calculateRateOfChangeOfMass();
//	float futureFluidMass = fluidMass + deltaTime * dmBydt;
//	
//	//If fluid is to be removed from this particle
//	if (dmBydt < 0){
//		//Check if it has any fluid in the first place	
//		if (fluidMass > 0){
//			//If it does, check if the current removal of fluid tries to remove more than available fluid
//			if (futureFluidMass < 0){
//				fluidMass = 0;
//				massChange = fluidMass;
//				cout << "something has gone wrong. Possibly, timestep is too large." << endl;
//			}
//			else{
//				massChange = futureFluidMass - fluidMass;
//				fluidMass = futureFluidMass;
//			}
//		}
//		//If there was no fluid in the particle, do not allow to reduce the mass into negative values
//		else{
//			fluidMass = 0;
//			massChange = 0;
//			cout << "there wasn't any fluid in the particle to begin with" << endl;
//		}
//	}
//	//Else if fluid is to be added
//	else{
//		//Check if there is an overflow of fluid
//		if (futureFluidMass > porosity * volume * fluidDensity){
//			massChange = porosity * volume * fluidDensity - fluidMass;
//			fluidMass = porosity * volume * fluidDensity;
//		}
//		//else just add the fluid
//		else{
//			massChange = futureFluidMass - fluidMass;
//			fluidMass = futureFluidMass;
//		}
//	}
//	//cout << "initial = " << fluidMass << ", and then masschange = " << massChange << " != " << deltaTime * dmBydt << "for ---" << pIndex << endl;
//	if (massChangeSign == 1){
//		if (massChange < threshold){
//			massChangeSign = -1;
//		}
//		else if (massChange > threshold){
//			massChangeSign = 1;
//		}
//		else{
//			massChangeSign = 0;
//		}
//	}
//
//	//cout << "fluid mass = " << fluidMass << endl;
//
//}


void SPHPorousParticle::updateProperties(){
	updateDensity();
	updatePorosity();
	updateSaturation();
	updateCapillaryPotential();
	updatePressure();
	updateVelocity();
}


float SPHPorousParticle::kernel(vec3 position, float h){
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

float SPHPorousParticle::kernelGradient(vec3 position, float h){
	vec3 relPos = pos - position;
	float r = glm::dot(relPos, glm::normalize(relPos));
	float q = r / h;
	float sigma = 945.f / (PI * 32.f);
	if (q >= 0 && q < 1){
		return -((sigma * r) / pow(h, 9)) * pow(h*h - r*r, 2);
	}
	else
		return 0;
}

float SPHPorousParticle::kernelLaplacian(vec3 position, float h){
	vec3 relPos = pos - position;
	float r = glm::dot(relPos, glm::normalize(relPos));
	float q = r / h;
	float sigma = 945.f / (PI * 8.f);
	if (q >= 0 && q < 1){
		return (sigma / pow(h, 9)) * (h*h - r*r)* (r*r - 0.75f*(h*h - r*r));
	}
	else
		return 0;
}


/*float SPHPorousParticle::kernel(vec3 position, float h){
	vec3 relPos = pos - position;
	float r = glm::dot(relPos, glm::normalize(relPos));
	float q = r / h;

	float sigma = 1/PI;
	if (q >= 0 && q < 1){
		return ((4 * sigma) / pow(h, 8)) * pow(h*h - r*r, 3);
	}
	else
		return 0;
}

float SPHPorousParticle::kernelGradient(vec3 position, float h){
	vec3 relPos = pos - position;
	float r = glm::dot(relPos, glm::normalize(relPos)) / h;
	float q = r / h;
	float sigma = 1 / PI;
	if (q >= 0 && q < 1){
		return -((24 * sigma * r) / pow(h, 8)) * pow(h*h - r*r, 2);
	}
	else
		return 0;
}

float SPHPorousParticle::kernelLaplacian(vec3 position, float h){
	vec3 relPos = pos - position;
	float r = glm::dot(relPos, glm::normalize(relPos)) / h;
	float q = r / h;
	float sigma = 1 / PI;
	if (q >= 0 && q < 1){
		return -((48 * sigma) / pow(h, 8)) * (h*h - r*r)*(h*h - 3*r*r);
	}
	else
		return 0;
}*/