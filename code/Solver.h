#ifndef GLFRAMEWORK_SOLVER_H
#define GLFRAMEWORK_SOLVER_H
#include <glm/gtc/matrix_transform.hpp>

using glm::vec3;

class Solver{
public:
	float timeStep;
	float elapsedTime;
	float ks;
	float kd;

	vec3 position;
	vec3 velocity;


	Solver();
	Solver(float pTimeStep, vec3 pPosition, vec3 pFirstDerivative);

	float dist(vec3 a, vec3 b);
	vec3 findAcceleration(vec3 movingMassPos, vec3 fixedPos, float restLength, bool damp);

	void eulerUpdate(vec3 movingMassPos, vec3 fixedPos, float restLength, bool damp);

	void implicitEulerUpdate(vec3 movingMassPos, vec3 fixedPos, float restLength, bool damp);

	void velocityVerletUpdate(vec3 movingMassPos, vec3 fixedPos, float restLength, bool damp);

	void RK4Update(vec3 movingMassPos, vec3 fixedPos, float restLength, bool damp);

};


#endif