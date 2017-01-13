#ifndef FRAMEWORK_POROUSFLOW_SCENE
#define FRAMEWORK_POROUSFLOW_SCENE


#include <iostream>
#include <GL/glew.h>
#include <GL/freeglut.h>

#include "Scene.h"
#include "glslprogram.h"
#include "Camera.h"
#include "FreeImage.h"
#include "Mesh.h"
#include "Solver.h"
#include "texture.h"
#include "Vertex.h"
#include "kdtree.h"
#include "hashgrid.h"
#include "SPHPorousParticle.h"
#include "SPHFluidParticle.h"
#include "app_perf.h"
#include <glm/gtc/matrix_transform.hpp>
#include <libconfig.h++>
using namespace libconfig;

const int side = 5;
const int MAXPARAMS = 20;
const int MAXITERS = 2000;

//const float threshold = pow(10.f, -5.f);

class PorousFlowScene : public Scene{

public:
	PorousFlowScene();
	~PorousFlowScene();
	void renderSceneCB();
	void updatePorousObjects();
	void updateFluidObjects();
	void keyboardCB(unsigned char key, int x, int y);
	void specialKeyboardCB(int key, int x, int y);
	void motionCB(int x, int y);
	void mouseCB(int button, int state, int x, int y);
	void passiveMouseCB(int x, int y);
	void idleCB();
	bool initTexture(const char* filename);
	void initObjects();
	void initPorousObjects();
	void initFluidObjects();
	void timerCB(int value);
	double perfRecord[MAXPARAMS];
private:
	void setMatrices();
	void initializeCamera();
	void compileShaders();
	template<class Searcher>
	void updatePorousSearcher(Searcher &searcher) const;
	template<class Searcher>
	void updateFluidSearcher(Searcher &searcher) const;
	void updatePorousNeighbors(KdTree tree, std::vector<std::vector<NeighborData> >* neighbors, float queryRadius);
	void updateFluidNeighbors(KdTree tree, std::vector<std::vector<NeighborData> >* neighbors, float queryRadius);
	bool checkForCollision(glm::vec3, glm::vec3, glm::vec3, float, float);
	void handleFluidSolidCollision();
	void fluidGenerator();
	void setFluidBounds(int);
	SPHFluidParticle createFluidParticle(vec3 pos, vec4 color, int index);
	
	//*************DATA MEMBERS**************
	//General scene objects
	Object *obj, *obj1;
	Solver* solver;
	Texture* texture = NULL;
	Camera mainCamera;
	bool isCameraActive = true;
	void record(int param, std::string name, Time& start);

	//animation and timer related members
	int timerTicks = 0;
	bool animate;

	GLSLProgram program;

	//mouse and motion related members
	int		last_x,
			last_y;
	bool	mouseUp = true, 
			mouseDown = false;
	bool generateFluid = true, updateFluid = true;
	float deltaTime, collElasticity, wallCollElasticity, epsilon, epsilon1, fluidOffset, pressureThreshold, satThreshold, fluidVelx, fluidVely, fluidVelz;
	float lBound, rBound, tBound, bBound, fBound, baBound, hFluid, hPorous, maxSpeed, maxAccel, maxPoreSpeed, ks, kd, ksh, depletionFactor;
	int side, fluidSide, MAXPARAMS, MAXITERS;
	
	
	KdTree porousTree, fluidTree;
	std::vector<std::vector<NeighborData> > porousNeighbors, fluidNeighbors;

	//SPHPorousParticle particles[numparticles];
	vector<SPHPorousParticle> particles;
	vector<SPHFluidParticle> fluidParticles;
	vector<Vertex> porousVertices;
	vector<Vertex> fluidVertices;
	
};
#endif