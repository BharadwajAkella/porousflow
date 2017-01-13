#include "Solver.h"
#include <iostream>
using namespace std;

Solver::Solver(float pTimeStep, vec3 pPosition, vec3 pVelocity){
	timeStep = pTimeStep;
	velocity = pVelocity;
	position = pPosition;
	
}

Solver::Solver(){
	timeStep = 0.1f;
	elapsedTime = 0;
	position = vec3(0.f);
}

float Solver::dist(vec3 a, vec3 b){
	vec3 diff = a - b;
	float distSquared = glm::dot(diff, diff);
	return sqrt(distSquared);
}

void printvec(vec3 vect){
	cout << vect.x << ", " << vect.y << ", " << vect.z << endl;
}

vec3 Solver::findAcceleration(vec3 movingMassPos, vec3 fixedPos, float restLength, bool damp){
		
	vec3 d = glm::normalize(movingMassPos - fixedPos);
	float springContrib = ks * (dist(movingMassPos, fixedPos) - restLength);	
		
	if (damp){
		
		float dampContrib = kd * glm::dot(velocity, d);
		return -(springContrib + dampContrib) * d;
	}

	return -springContrib * d;
}

void Solver::eulerUpdate(vec3 movingMassPos, vec3 fixedPos, float restLength, bool damp){
	vec3 acceleration = findAcceleration(movingMassPos, fixedPos, restLength, damp);
	position += timeStep * velocity;
	velocity += timeStep * acceleration;
}

void Solver::velocityVerletUpdate(vec3 movingMassPos, vec3 fixedPos, float restLength, bool damp){
	vec3 currentAcceleration = findAcceleration(movingMassPos, fixedPos, restLength, damp);
	position += velocity * timeStep + 0.5f * (currentAcceleration * timeStep * timeStep);
	vec3 nextAcceleration = findAcceleration(position, fixedPos, restLength, damp);
	velocity += timeStep * (nextAcceleration + currentAcceleration) * 0.5f;
}

void Solver::implicitEulerUpdate(vec3 movingMassPos, vec3 fixedPos, float restLength, bool damp){
	vec3 acceleration = findAcceleration(movingMassPos, fixedPos, restLength, damp);
	velocity += timeStep * acceleration;
	position += timeStep * velocity;
}

void Solver::RK4Update(vec3 movingMassPos, vec3 fixedPos, float restLength, bool damp){
	
	vec3 a1 = findAcceleration(movingMassPos, fixedPos, restLength, damp);
	vec3 k1 = velocity + 0.5f * timeStep * a1;
	vec3 x1 = position + 0.5f* timeStep * k1;

	vec3 a2 = findAcceleration(x1, fixedPos, restLength, damp);
	vec3 k2 = velocity + 0.5f * timeStep * a2;
	vec3 x2 = position + 0.5f * timeStep * k2;

	vec3 a3 = findAcceleration(x2, fixedPos, restLength, damp);
	vec3 k3 = velocity + 0.5f * timeStep * a3;
	vec3 x3 = position + 0.5f * timeStep * k3;

	vec3 a4 = findAcceleration(x3, fixedPos, restLength, damp);
	vec3 k4 = velocity + 0.5f * timeStep * a4;
	vec3 x4 = position + 0.5f * timeStep * k4;


	position += timeStep * (k1 + 2.f * k2 + 2.f * k3 + k4) / 6.f;
	velocity += timeStep * (a1 + 2.f * a2 + 2.f * a3 + a4) / 6.f;
}