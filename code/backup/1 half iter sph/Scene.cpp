#include "PorousFlowScene.h"
#include <set>
#include <algorithm>

#define PI 3.1412f
#include "kdtree.h"
using std::cout; using std::endl;

const int windowWidth = 640, windowHeight = 480;
const int satThreshold = 0.9f;


void PorousFlowScene::record(int param, std::string name, Time& start)
{
	Time stop;
	stop.SetSystemTime();
	stop = stop - start;
	perfRecord[param] += stop.GetMSec();
	//printf ("%s:  %s\n", name.c_str(), stop.GetReadableTime().c_str() );

}

PorousFlowScene::PorousFlowScene() {
	Config maincfg;
	maincfg.readFile("mainSceneConfig.cfg");
	const Setting& root = maincfg.getRoot();
	epsilon = root["epsilon"];
	deltaTime = root["deltaTime"];
	side = root["side"];
	MAXPARAMS = root["maxparams"];
	MAXITERS = root["maxiters"];
	collElasticity = root["collElasticity"];
	fluidOffset = root["fluidOffset"];
	pressureThreshold = root["pressureThreshold"];

	initializeCamera();	
	
	for (unsigned int i = 0; i < MAXPARAMS; i++){
		perfRecord[i] = 0.0;
	}

	particles.resize(side * side * side);
	animate = false;
}

PorousFlowScene::~PorousFlowScene(){

}
int cycles = 0;
void PorousFlowScene::renderSceneCB() {
	glClear(GL_COLOR_BUFFER_BIT);
	program.use();
	setMatrices();
	if (animate && cycles <= 20){
		updateFluidObjects();
		//updatePorousObjects();
		//cout << endl;
		//cycles++;
	}

	obj1->fillBuffers(obj1->getVertexArrayHandle(), obj1->getVertexBufferHandle(), &fluidVertices[0], fluidVertices.size());
	obj1->render();

	obj->fillBuffers(obj->getVertexArrayHandle(), obj->getVertexBufferHandle(), &porousVertices[0], porousVertices.size());
	obj->render();
	
	glBindVertexArray(0);
	glUseProgram(0);
	glutSwapBuffers();
}

void PorousFlowScene::updateFluidObjects(){	
	Time start;
	start.SetSystemTime();
	PERF_PUSH("calc viscous and inter vel/pos");
		for (int i = 0; i < fluidParticles.size(); i++){
			fluidParticles[i].calculateViscousForce();
			fluidParticles[i].updateIntermediateVelocity();
			fluidParticles[i].updatePosition();
		}
	PERF_POP();
	record(1, "Visc velpos", start);

	float error;
	start.SetSystemTime();
	PERF_PUSH("iteration");
		float newDensity, restDensity;
		int numIter = 1;
		//while (true){
			error = 0.f;
			for (int i = 0; i < fluidParticles.size(); i++){
				fluidParticles[i].updateDensity();
				fluidParticles[i].updatePressure();
				newDensity = fluidParticles[i].density;
				restDensity = fluidParticles[i].restDensity;

				error = (error * i + abs(newDensity - restDensity)) / (i + 1);
			}
			for (int i = 0; i < fluidParticles.size(); i++){
				fluidParticles[i].calculatePressureForce();
				fluidParticles[i].updateVelocity();
				fluidParticles[i].updatePosition();	
			}

			numIter++;
		//}
	PERF_POP();
	record(2, "iteration", start);
	
	fluidVertices.clear();
	for (int i = 0; i < fluidParticles.size(); i++){
		/*if (fluidParticles[i].pos.y <= -3.f){
			fluidParticles[i].velocity = - collElasticity * fluidParticles[i].velocity;
			fluidParticles[i].pos.y = -2.99f;
		}*/
		fluidVertices.push_back(fluidParticles[i]);
	}	

	start.SetSystemTime();
	PERF_PUSH("update neigh");
		updateFluidNeighbors(fluidTree, &fluidNeighbors, fluidParticles[0].h);
	PERF_POP();
	record(4, "update neigh", start);

	start.SetSystemTime();
	PERF_PUSH("collision handling");
		handleFluidSolidCollision();
	PERF_POP();
	record(3, "coll handle", start);
}

void PorousFlowScene::handleFluidSolidCollision(){
	std::set<int> toDelete;
	for (int i = 0; i < fluidParticles.size(); i++){
		for (int j = 0; j < fluidNeighbors[i].size(); j++){
			int id = fluidNeighbors[i][j].idx;
  
			if (fluidParticles[id].onboundary == true && fluidParticles[i].onboundary == false && fluidParticles[i].passive == false){
				bool collision = checkForCollision(fluidParticles[i].pos, fluidParticles[id].pos, fluidParticles[i].radius, fluidParticles[id].radius);
				if (collision){
					
					SPHPorousParticle *boundaryParticle = (fluidParticles[id].boundaryFriend);
					float maxCapacity = boundaryParticle->porosity * boundaryParticle->volume * boundaryParticle->fluidDensity;
					float remainingCapacity = maxCapacity - boundaryParticle->fluidMass;
					if (fluidParticles[i].mass > remainingCapacity){
						boundaryParticle->saturation = 1.f;
						boundaryParticle->updateFluidMass();
						fluidParticles[i].mass -= remainingCapacity;
						fluidParticles[i].velocity = -collElasticity * fluidParticles[i].velocity;
						fluidParticles[i].pos += deltaTime * fluidParticles[i].velocity;
						fluidParticles[i].volume = fluidParticles[i].mass / fluidParticles[i].density;
					}
					else{
						boundaryParticle->fluidMass += fluidParticles[i].mass;
						boundaryParticle->updateSaturation();
						toDelete.insert(i);
					}
				}
			}
		}
	}
	if (toDelete.size() > 0){
		//std::sort(toDelete.begin(), toDelete.end());
		bool exit = false;
		for (std::set<int>::iterator it = toDelete.end();;){
			it--;
			if (it == toDelete.begin())
				exit = true;
			fluidParticles.erase(fluidParticles.begin() + (*it));
			if (exit)
				break;
		}
		for (int i = 0; i < fluidParticles.size(); i++){
			fluidParticles[i].pIndex = i;
		}
		updateFluidNeighbors(fluidTree, &fluidNeighbors, fluidParticles[0].h);
	}
}

bool PorousFlowScene::checkForCollision(vec3 p1, vec3 p2, float r1, float r2){
	float dist = glm::dot(p1 - p2, p1 - p2);
	
	if (dist < epsilon)
		return true;
	/*if (r1 + r2 > 2*dist)
		return true;*/
	return false;
}

void PorousFlowScene::updatePorousObjects(){
	//cout << endl << endl;
	float totalMassChange = 0.f;
	int activeParticles = 0;
	int posActiveParticles = 0;
	int negActiveParticles = 0;
	for (int i = 0; i < particles.size(); i++){
		particles[i].updateProperties();
	}
	for (int i = 0; i < particles.size(); i++){
		particles[i].dummy();
	}
	for (int i = 0; i < particles.size(); i++){
		particles[i].fluidMass += particles[i].massChange;
		particles[i].updateSaturation();
		//cout << i << "'s total mass change = " << particles[i].massChange << " and saturation = " << particles[i].saturation << endl << endl;
		particles[i].massChange = 0.f;
		porousVertices[i].color = vec4(1 - particles[i].saturation, 0.f, particles[i].saturation, 1.f);
	}	
}

void PorousFlowScene::setMatrices(){
	mat4 projectionMatrix = glm::perspective(45.0f,
		(windowWidth * 1.0f) / (windowHeight * 1.0f), (const float) 0.1,
		(const float) 100.0);
	mat4 viewMatrix = glm::lookAt(mainCamera.getEye(), mainCamera.getTarget(),
		mainCamera.getUp());
	mat4 modelMatrix = glm::translate(mat4(1.f), vec3(0.f, 0.1f, 0.f));
	mat4 mvp = projectionMatrix * viewMatrix * modelMatrix;
	mat4 normalMatrix = viewMatrix * modelMatrix;

	program.setUniform("normalMatrix", normalMatrix);
	program.setUniform("mvpMatrix", mvp);
}

void PorousFlowScene::keyboardCB(unsigned char key, int x, int y) {
	switch (key) {
	case 27:
		for (int i = 0; i < MAXPARAMS; i++){
			if (perfRecord[i] != 0){
				cout << i << " --> " << perfRecord[i] << endl;
			}
		}
		exit(0);
	case 'w':
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		break;
	case 'f':
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		break;
	case 'q':
	case 'Q':
		animate = false;
		break;
	case ' ':
		animate = !animate;
		break;

	}
}

void PorousFlowScene::specialKeyboardCB(int key, int x, int y) {
	if (isCameraActive) {
		mainCamera.handleCameraByKeyboard(key);
	}
	glutPostRedisplay();
}

void PorousFlowScene::motionCB(int x, int y) {
	float dx = 0.0f, dy = 0.0f;
	GLint viewport[4];
	int keyModifiers = 0;

	keyModifiers = glutGetModifiers();

	glGetIntegerv(GL_VIEWPORT, viewport);

	dx = (x - last_x) / (float)viewport[2] * 2;
	dy = -(y - last_y) / (float)viewport[3] * 2;

	last_x = x;
	last_y = y;
	if (mouseUp) {
		dx = 0;
		dy = 0;
		mouseUp = !mouseUp;
	}
	if (keyModifiers & GLUT_ACTIVE_ALT) {
		if (mouseDown == true) {
			mainCamera.handleCameraByMouse(dx);
		}
		else {
			mainCamera.handleCameraByMouse(dx, dy);
		}

	}

	glutPostRedisplay();

}

void PorousFlowScene::mouseCB(int button, int state, int x, int y) {
	mouseDown = false;
	if (GLUT_UP == state) {
		mouseUp = true;
	}
	if (state == GLUT_DOWN && button == GLUT_RIGHT_BUTTON) {
		mouseDown = true;
	}
	glutPostRedisplay();
}


void PorousFlowScene::passiveMouseCB(int x, int y) {

}

void PorousFlowScene::idleCB() {
	renderSceneCB();
}

void PorousFlowScene::initializeCamera() {
	mainCamera.setCamera(vec3(0.0, 0.0, -10.0), vec3(0, 0, 0),
		vec3(0, 1.0, 0));
}

void PorousFlowScene::compileShaders() {
	program.compileShader("vs.glsl", GLSLShader::VERTEX);
	program.compileShader("fs.glsl", GLSLShader::FRAGMENT);
	program.link();
	program.validate();
	program.use();
}

bool PorousFlowScene::initTexture(const char* filename) {
	texture = new Texture(GL_TEXTURE_2D, filename);
	if (!texture->Load()) {
		return false;
	}
	return true;
}

void PorousFlowScene::initFluidObjects(){

	for (float i = 0; i < side; i++){
		for (float j = 0; j < side; j++){
			for (float k = 0; k < 3; k ++){
				SPHFluidParticle v;
				v.pos = vec3(-(side / 2) + j, 3 - (side / 2) + k + fluidOffset, -(side / 2) + i);
				v.color = vec4(0.f, 0.f, 1.f, 1.f);
				v.normal = vec3(0, 1, 0);

				//v.velocity = vec3(0, 1.f, 0);
				fluidParticles.push_back(v);
			}
		}
	}

	updateFluidNeighbors(fluidTree, &fluidNeighbors, fluidParticles[0].h);
	for (int i = 0; i < fluidParticles.size(); i++){
		fluidParticles[i].pIndex = i;
		fluidParticles[i].neighbors = &fluidNeighbors;
		fluidParticles[i].particles = &fluidParticles;
	}
	for (int i = 0; i < fluidParticles.size(); i++){
		fluidParticles[i].updateDensity();
		//fluidParticles[i].restDensity = fluidParticles[i].density;
		fluidVertices.push_back(fluidParticles[i]);
	}
}

void PorousFlowScene::updateFluidNeighbors(KdTree tree, std::vector<std::vector<NeighborData> >* neighbors, float queryRadius){
	tree.init(); 
	updateFluidSearcher(tree);
	tree.queryRadius(queryRadius);
	(*neighbors).resize(fluidParticles.size());
	for (int i = 0; i < fluidParticles.size(); ++i) {
		tree.neighbors(fluidParticles[i].pos, (*neighbors)[i]);
	}
}

void PorousFlowScene::updatePorousNeighbors(KdTree tree, std::vector<std::vector<NeighborData> >* neighbors, float queryRadius){
	tree.init();
	updatePorousSearcher(tree);
	tree.queryRadius(queryRadius);
	(*neighbors).resize(particles.size());
	for (int i = 0; i < particles.size(); ++i) {
		tree.neighbors(particles[i].pos, (*neighbors)[i]);
	}
}
void PorousFlowScene::initPorousObjects(){
	int index = -1;
	for (int i = 0; i < side; i++){
		for (int j = 0; j < side; j++){
			for (int k = 0; k < side; k++){
				index++;
				particles[index].pos = vec3(-(side / 2) + j, -(side / 2) + k , -(side / 2) + i);
				particles[index].pIndex = index;

				if (k == side - 1 /*index == 0 || index == 1*/ ){
					particles[index].color = vec4(0.f, 0.f, 1.f, 1.f);
					particles[index].fluidMass = particles[index].volume * particles[index].porosity * particles[index].fluidDensity;
					particles[index].updateSaturation();
				}
				if (i == side - 1 || j == side - 1 || i == 0 || j == 0 || k == 0 || k == side - 1){
					SPHFluidParticle v;
					v.pos = particles[index].pos;
					v.color = vec4(0);
					v.onboundary = true;
					v.boundaryFriend = &(particles[index]);
					fluidParticles.push_back(v);
				}

				particles[index].updateCapillaryPotential();
				particles[index].color = vec4(1 - particles[index].saturation, 0.f, particles[index].saturation, 1.f);

				particles[index].normal = vec3(0, 1, 0);
				porousVertices.push_back(particles[index]);
			}
		}
	}

	updatePorousNeighbors(porousTree, &porousNeighbors, particles[0].h);
	index = -1;
	for (int i = 0; i < particles.size(); i++){
		index++;
		particles[index].neighbors = &porousNeighbors;
		particles[index].particles = &particles;
	}
}

void PorousFlowScene::initObjects() {
	compileShaders();

	initPorousObjects();
	initFluidObjects();

	obj1 = new Object(&fluidVertices[0], fluidVertices.size(), NULL, 0, GL_POINTS);
	obj1->fillBuffers(obj1->getVertexArrayHandle(), obj1->getVertexBufferHandle(),
		&obj1->vertices[0], obj1->vertices.size());

	obj = new Object(&porousVertices[0], porousVertices.size(), NULL, 0, GL_POINTS);
	obj->fillBuffers(obj->getVertexArrayHandle(), obj->getVertexBufferHandle(),
					 &obj->vertices[0], obj->vertices.size());

	
}

void PorousFlowScene::timerCB(int value) {
	//glutTimerFunc(60, timer, 0);
	glutPostRedisplay();
}

template<class Searcher>
void PorousFlowScene::updatePorousSearcher(Searcher &searcher) const {
	// create search data structure 
	searcher.clear();
	for (int i = 0; i < particles.size(); ++i) {
		searcher.insert(i, particles[i].pos);
	}
	searcher.init();
}

template<class Searcher>
void PorousFlowScene::updateFluidSearcher(Searcher &searcher) const {
	// create search data structure 
	searcher.clear();
	for (int i = 0; i < fluidParticles.size(); ++i) {
		searcher.insert(i, fluidParticles[i].pos);
	}
	searcher.init();
}