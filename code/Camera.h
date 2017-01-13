#ifndef CAMERA_H
#define	CAMERA_H

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <glm/gtc/matrix_transform.hpp>
using glm::vec3;
using glm::mat4;
#include <math.h>



class Camera{
public:
	Camera();
	Camera(vec3 eye, vec3 target, vec3 up);
	bool handleCameraByKeyboard(int key);
	void handleCameraByMouse(float dx, float dy);
	void handleCameraByMouse(float dx);

	const vec3 getEye(){
		return eye;
	}
	const vec3 getTarget(){
		return target;
	}
	const vec3 getUp(){
		return up;
	}
	void setCamera(vec3 eye, vec3 target, vec3 up);
float radius, alpha, beta;
private:
	vec3 eye;
	vec3 target;
	vec3 up;
	
};

#endif