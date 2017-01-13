#include <glm/gtc/matrix_transform.hpp>
using glm::vec3;
using glm::mat4;
#include <math.h>
const float PI = 3.141f;

inline float convertDegreesToRadians(float degrees){
	return (PI / 180.f)*degrees;
}

inline float convertRadiansToDegrees(float radians){
	return (180.f/PI)*radians;
}

vec3 computeSphericalCoordinates(float radius, float alpha, float beta, bool anglesInDegrees = true){
	if (anglesInDegrees){
		alpha = convertDegreesToRadians(alpha);
		beta = convertDegreesToRadians(beta);
	}	
	return vec3(radius * sin(alpha) * cos(beta), radius * sin(alpha) * sin(beta), radius * cos(alpha));
}

float computeRadius(float xComp, float yComp, float zComp){
	return sqrt(xComp * xComp + yComp * yComp + zComp* zComp);
}