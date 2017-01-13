#include "Camera.h"
#include <iostream>
#include "math_helper_functions.cpp"
using namespace std;
const float scaleStep = 0.5;
Camera::Camera(){
	radius = 1.f;
	alpha = 45.0;
	beta = 45.0;
	eye = computeSphericalCoordinates(radius, alpha, beta);
	target = vec3(0.f, 0.f, 0.f);
	up = vec3(0.f, 1.f, 0.f);
	
}



Camera::Camera(const vec3 p_eye, const vec3 p_target, const vec3 p_up){
	setCamera(p_eye, p_target, p_up);
}

void Camera::setCamera(const vec3 p_eye, const vec3 p_target, const vec3 p_up){
	eye = p_eye;
	radius = computeRadius(eye.x, eye.y, eye.z);
	target = p_target;
	glm::normalize(target);
	up = p_up;
	glm::normalize(up);
	alpha = acos(eye.z / radius);	
	if (alpha == 0.f){
		beta = 0.f;
	}
	else{
		beta = acos(eye.x / (radius * sin(alpha)));
	}	
	alpha = convertRadiansToDegrees(alpha);
	beta = convertRadiansToDegrees(beta);
	eye = computeSphericalCoordinates(radius, alpha, beta);

} 

void Camera::handleCameraByMouse(float dx, float dy){
	alpha += 50*dx;
	beta += 50*dy;
	eye = computeSphericalCoordinates(radius, alpha, beta);
}

void Camera::handleCameraByMouse(float dx){
	radius -= 10 *dx;
	eye = computeSphericalCoordinates(radius, alpha, beta);
}

bool Camera::handleCameraByKeyboard(int key){
	bool returnValue = false;

	vec3 left = glm::cross(target, up); // direction perpendicular to the plane of target and up
	glm::normalize(left);
	left *= scaleStep;

	switch (key){
	case GLUT_KEY_UP:
		alpha += 10;
		eye = computeSphericalCoordinates(radius, alpha, beta);
		returnValue = true;
		break;
	case GLUT_KEY_DOWN:
		alpha -= 10;
		eye = computeSphericalCoordinates(radius, alpha, beta);
		returnValue = true;
		break;
	case GLUT_KEY_LEFT:
		beta -= 10;
		eye = computeSphericalCoordinates(radius, alpha, beta);
		returnValue = true; 
		break;
	case GLUT_KEY_RIGHT:
		beta += 10;
		eye = computeSphericalCoordinates(radius, alpha, beta);
		returnValue = true; 
		break;
	}
	return returnValue;
}