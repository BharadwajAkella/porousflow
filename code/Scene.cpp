#include "PorousFlowScene.h"
#include <set>
#include <algorithm>

#define PI 3.1412f
#include "kdtree.h"
using std::cout; using std::endl;

const int windowWidth = 640, windowHeight = 480;
const int timerReset = 100;

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
	epsilon1 = root["epsilon1"];
	deltaTime = root["deltaTime"];
	side = root["side"];
	fluidSide = root["fluidSide"];
	MAXPARAMS = root["maxparams"];
	MAXITERS = root["maxiters"];
	collElasticity = root["collElasticity"];
	fluidOffset = root["fluidOffset"];
	pressureThreshold = root["pressureThreshold"];
	satThreshold = root["satThreshold"];
	fluidVelx = root["fluidVelx"];
	fluidVely = root["fluidVely"];
	fluidVelz = root["fluidVelz"];
	lBound = root["lBound"];
	rBound = root["rBound"];
	tBound = root["tBound"];
	bBound = root["bBound"];
	fBound = root["fBound"];
	baBound = root["baBound"];
	wallCollElasticity = root["wallCollElasticity"];
	maxSpeed = root["maxSpeed"];
	maxAccel = root["maxAccel"];
	maxPoreSpeed = root["maxPoreSpeed"];
	ks = root["ks"];
	kd = root["kd"];
	ksh = root["ksh"];
	depletionFactor = root["depletionFactor"];
	initializeCamera();

	for (unsigned int i = 0; i < MAXPARAMS; i++){
		perfRecord[i] = 0.0;
	}

	//particles.resize(side * side * side);
	animate = false;
}

PorousFlowScene::~PorousFlowScene(){

}
int cycles = 0;
void PorousFlowScene::renderSceneCB() {
	glClear(GL_COLOR_BUFFER_BIT);

	program.use();
	setMatrices();
	if (animate /*&& cycles <= 20*/){
		timerTicks = (timerTicks + 1) % timerReset;

		/*if (generateFluid){
		fluidGenerator();
		}*/
		if (updateFluid){
			updateFluidObjects();
		}
		updatePorousObjects();
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
	PERF_PUSH("update neigh");
	updateFluidNeighbors(fluidTree, &fluidNeighbors, fluidParticles[0].h);
	PERF_POP();
	record(4, "update neigh", start);


	start.SetSystemTime();
	PERF_PUSH("calc viscous and inter vel/pos");

	for (int i = 0; i < fluidParticles.size(); i++){
		fluidParticles[i].calculateViscousForce();
		//cout << "particle " << i << " vel = " << fluidParticles[i].viscousForce.x << ", " << fluidParticles[i].viscousForce.y << ", " << fluidParticles[i].viscousForce.z << ", " << endl;
		fluidParticles[i].calculateBodyForce();
		float accel = glm::dot(fluidParticles[i].acceleration, glm::normalize(fluidParticles[i].acceleration));
		if (accel > maxAccel){
			fluidParticles[i].acceleration = (fluidParticles[i].acceleration) * (maxAccel / accel);
		}
		fluidParticles[i].updateVelocity();
		//fluidParticles[i].updatePosition();
		//cout << "particle "<< i << " vel = " << fluidParticles[i].velocity.x << ", " << fluidParticles[i].velocity.y << ", " << fluidParticles[i].velocity.z << ", " << endl;
	}
	PERF_POP();
	record(1, "Visc velpos", start);

	float error;
	start.SetSystemTime();
	PERF_PUSH("iteration");
	float newDensity, restDensity, oldPressure, newPressure;
	int numIter = 1;
	//while (true){
	error = 0.f;
	for (int i = 0; i < fluidParticles.size(); i++){
		if (fluidParticles[i].active){
			fluidParticles[i].updateDensity();
			oldPressure = fluidParticles[i].pressure;
			fluidParticles[i].updatePressure();
			newPressure = fluidParticles[i].pressure;
			newDensity = fluidParticles[i].density;
			restDensity = fluidParticles[i].restDensity;

			/*if (generateFluid){
			cout << oldPressure << " p changes to --> " << newPressure << endl;
			cout << restDensity << " d changes to --> " << newDensity << endl;
			}*/
			error = (error * i + abs(newDensity - restDensity)) / (i + 1);

			/*if (error < pressureThreshold){
			}
			else{
			cout << "bad pressure force accumulation - " << newDensity << " , " << endl;
			if (numIter >= 10)
			cout << "more than 10 iterations " << endl;
			}*/
		}
	}

	for (int i = 0; i < fluidParticles.size(); i++){
		if (fluidParticles[i].active){
			fluidParticles[i].calculatePressureForce();
		}
	}

	numIter++;
	//}
	PERF_POP();
	record(2, "iteration", start);

	start.SetSystemTime();
	PERF_PUSH("collision handling");
	handleFluidSolidCollision();
	PERF_POP();
	record(3, "coll handle", start);

	for (int i = 0; i < fluidParticles.size(); i++){
		if (fluidParticles[i].active){
			float accel = glm::dot(fluidParticles[i].acceleration, glm::normalize(fluidParticles[i].acceleration));
			if (accel > maxAccel){
				fluidParticles[i].acceleration = (fluidParticles[i].acceleration) * (maxAccel / accel);
			}
			fluidParticles[i].updateVelocity();
			//cout << "particle " << i << " vel = " << fluidParticles[i].velocity.x << ", " << fluidParticles[i].velocity.y << ", " << fluidParticles[i].velocity.z << ", " << endl;
			fluidParticles[i].updatePosition();
			setFluidBounds(i);
		}
	}

	fluidVertices.clear();
	for (int i = 0; i < fluidParticles.size(); i++){
		fluidVertices.push_back(fluidParticles[i]);
	}
}

void PorousFlowScene::handleFluidSolidCollision(){
	std::set<int> toDelete;
	float spCoef = 10.f, dampCoef = 0.5f, shearCoef = 0.2f;
	for (int i = 0; i < fluidParticles.size(); i++){
		SPHFluidParticle currParticle = fluidParticles[i];

		if (!currParticle.onboundary && currParticle.active){
			for (int j = 0; j < fluidNeighbors[i].size(); j++){
				int id = fluidNeighbors[i][j].idx;
				SPHFluidParticle fBParticle = fluidParticles[id];
				SPHPorousParticle *boundaryParticle = (fluidParticles[id].boundaryFriend);
				if (fluidParticles[id].onboundary){
					vec3 norm = glm::normalize(boundaryParticle->normal);
					bool collision = checkForCollision(currParticle.pos, boundaryParticle->pos, norm, currParticle.radius, boundaryParticle->radius);
					if (collision){
						vec3 relPos = currParticle.pos - boundaryParticle->pos;
						float dist = glm::dot(relPos, glm::normalize(relPos));

						vec3 relVelocity = currParticle.velocity - fBParticle.velocity;
						float relSpeed = glm::dot(relVelocity, glm::normalize(relVelocity));
						if (relSpeed < 1.f){
							vec3 accNormalComp = glm::dot(currParticle.acceleration, norm) * norm;
							vec3 accOtherComp = currParticle.acceleration - accNormalComp;
							currParticle.acceleration = accOtherComp;
						}
						else{
							float penetrationDepth = currParticle.radius + boundaryParticle->radius - dist;
							float relSpeed = glm::dot(relVelocity, glm::normalize(relVelocity));
							fluidParticles[i].pos += collElasticity * penetrationDepth * norm;
							
							vec3 repulsiveForce = -ks * penetrationDepth * norm;
							vec3 dampingForce = kd * (-relVelocity);
							vec3 relTanVel = relVelocity - glm::dot(relVelocity, norm) * norm;
							vec3 shearForce = ksh * relTanVel;
							vec3 acc = (repulsiveForce + shearForce + dampingForce)/currParticle.mass;
							currParticle.acceleration += acc;
						}
						

						/*if (relSpeed > 0)
							fluidParticles[i].velocity += -(1 + collElasticity*penetrationDepth / (deltaTime * relSpeed)) * glm::dot(relVelocity, norm)*norm;
						else
							fluidParticles[i].velocity += -(1 + collElasticity) * glm::dot(relVelocity, norm)*norm;*/

						float maxCapacity = boundaryParticle->porosity * boundaryParticle->volume * boundaryParticle->fluidDensity;
						float remainingCapacity = maxCapacity - boundaryParticle->fluidMass;
						if (currParticle.mass > remainingCapacity){
							boundaryParticle->saturation = 1.f;
							boundaryParticle->updateFluidMass();
							currParticle.mass -= remainingCapacity;
							vec3 contactNormal = glm::normalize(boundaryParticle->pos - currParticle.pos);
							float speedNormal = glm::dot(boundaryParticle->velocity, contactNormal);

							currParticle.velocity += currParticle.acceleration * deltaTime;
							currParticle.pos += deltaTime * currParticle.velocity;
							currParticle.volume = currParticle.mass / currParticle.density;
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
	}
}

//p2,r2 --> boundary and p1,r1 --> fluid
bool PorousFlowScene::checkForCollision(vec3 p1, vec3 p2, vec3 normal, float r1, float r2){
	float dist = glm::dot(p1 - p2, p1 - p2);
	float sign = glm::dot(p1 - p2, normal);

	/*if (dist < epsilon)
		return true;*/
	////normal sphere collision
	if (r1 + r2 > dist)
		return true;
	return false;
}

void printmass(vector< vector<float> > v){
	for (int i = 0; i < v.size(); i++){
		cout << i << "   ---->   " << v[i][0] << " \t \t   " << v[i][1] << endl;
	}
	cout << endl;
}

void PorousFlowScene::updatePorousObjects(){
	//cout << endl << endl;
	float totalMassChange = 0.f;
	int activeParticles = 0;
	int posActiveParticles = 0;
	int negActiveParticles = 0;
	float totalmass = 0.f;
	vector< vector<float> > mass;
	mass.resize(particles.size() + 1);
	for (int i = 0; i < particles.size(); i++){
		totalmass += particles[i].fluidMass;
		mass[i].resize(2);
		mass[i][0] = particles[i].fluidMass;
		//cout << "particle " << i << " 's fluid mass = " << particles[i].fluidMass  << endl;
		particles[i].updateProperties();
	}
	mass[particles.size()].resize(2);
	mass[particles.size()][0] = totalmass;
	//cout << endl << "mass before update --> " << totalmass << endl;
	totalmass = 0.f;
	for (int i = 0; i < particles.size(); i++){
		particles[i].diffuse();
		if (particles[i].onboundary){
			vec3 vel = particles[i].velocity;
			float speedAlongNormal = glm::dot(vel, glm::normalize(particles[i].normal));
			if (glm::dot(particles[i].velocity, particles[i].normal) > 0 && speedAlongNormal > maxPoreSpeed && particles[i].saturation >= satThreshold ){
				/*	If velocity along normal is greater than threshold and if the particles is sufficiently saturated
					then the porous particle emits a fluid particle*/

				//Generate a new fluid particle
				vec3 newParticlePos = particles[i].pos + particles[i].velocity * deltaTime * epsilon1;
				SPHFluidParticle p = createFluidParticle(particles[i].pos, vec4(0, 0, 1.f, 1.f), fluidParticles.size());
				//Give a fraction of porous particle's fluid mass to the fluid
				p.mass = depletionFactor * particles[i].fluidMass;
				particles[i].massChange -= p.mass;
				//Give an artificial y velocity to the fluid particle
				p.velocity = vec3(0, -50.f, 0);
				//Add the new particle to the fluid particles list
				fluidParticles.push_back(p);
			}
		}
		totalmass += particles[i].massChange;
	}

	totalmass = 0.f;
	//Update fluid mass using massChange member of the porous particle class
	for (int i = 0; i < particles.size(); i++){
		//cout << " initial fluid mass = " << particles[i].fluidMass << "  ";
		particles[i].fluidMass += particles[i].massChange;
		//cout << "later fluid mass = " << particles[i].fluidMass  <<  " initial saturation = " << particles[i].saturation << "  ";
		particles[i].updateSaturation();
		//cout << i << "'s total mass change = " << particles[i].massChange << " and saturation = " << particles[i].saturation << endl << endl;
		particles[i].massChange = 0.f;
		totalmass += particles[i].fluidMass;
		mass[i][1] = particles[i].fluidMass;
		//cout << "particle " << i << " 's fluid mass = " << particles[i].fluidMass << endl;
		porousVertices[i].color = vec4(1 - particles[i].saturation, 0.f, particles[i].saturation, 1.f);
	}
	mass[particles.size()][1] = totalmass;
	//cout << endl << "mass after update --> " << totalmass << endl << endl;
	//printmass(mass);
}

SPHFluidParticle PorousFlowScene::createFluidParticle(vec3 pos, vec4 color, int index){
	SPHFluidParticle v;
	v.pos = pos;
	v.color = color;
	v.pIndex = index;
	v.particles = &fluidParticles;
	v.neighbors = &fluidNeighbors;
	return v;
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
	case 'W':
		generateFluid = !generateFluid;
		break;
	case 'f':
	case 'F':
		updateFluid = !updateFluid;
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

void PorousFlowScene::setFluidBounds(int i){
	float speed = glm::dot(fluidParticles[i].velocity, glm::normalize(fluidParticles[i].velocity));

	if (!fluidParticles[i].onboundary){
		if (fluidParticles[i].pos.x <= lBound){
			float penetrationDepth = abs(fluidParticles[i].pos.x - lBound);
			fluidParticles[i].pos.x = lBound;
			fluidParticles[i].velocity += (1 + wallCollElasticity* penetrationDepth / (deltaTime * speed)) * vec3(1.f, 0, 0);
		}
		if (fluidParticles[i].pos.x >= rBound){
			fluidParticles[i].pos.x = rBound;
			float penetrationDepth = abs(fluidParticles[i].pos.x - rBound);
			fluidParticles[i].velocity += (1 + wallCollElasticity* penetrationDepth / (deltaTime * speed)) * vec3(-1.f, 0, 0);
		}
		if (fluidParticles[i].pos.y <= bBound){
			fluidParticles[i].pos.y = bBound;
			float penetrationDepth = abs(fluidParticles[i].pos.y - bBound);
			fluidParticles[i].velocity += (1 + wallCollElasticity* penetrationDepth / (deltaTime * speed)) * vec3(0, 1.f, 0);
		}
		if (fluidParticles[i].pos.y >= tBound){
			fluidParticles[i].pos.y = tBound;
			float penetrationDepth = abs(fluidParticles[i].pos.y - tBound);
			fluidParticles[i].velocity += (1 + wallCollElasticity* penetrationDepth / (deltaTime * speed)) * vec3(0, -1.f, 0);
		}
		if (fluidParticles[i].pos.z >= fBound){
			fluidParticles[i].pos.z = fBound;
			float penetrationDepth = abs(fluidParticles[i].pos.z - fBound);
			fluidParticles[i].velocity += (1 + wallCollElasticity* penetrationDepth / (deltaTime * speed)) * vec3(0, 0, -1.f);
		}
		if (fluidParticles[i].pos.z <= baBound){
			fluidParticles[i].pos.z = baBound;
			float penetrationDepth = abs(fluidParticles[i].pos.z - baBound);
			fluidParticles[i].velocity += (1 + wallCollElasticity* penetrationDepth / (deltaTime * speed)) * vec3(0, 0, 1.f);
		}
		float speed = glm::dot(fluidParticles[i].velocity, glm::normalize(fluidParticles[i].velocity));
		if (speed > maxSpeed){
			fluidParticles[i].velocity = (fluidParticles[i].velocity) * (maxSpeed / speed);
		}
	}
}

void PorousFlowScene::fluidGenerator(){
	if (timerTicks == 0){
		for (float j = 0; j < side; j++){
			for (float k = 0; k < side; k++){
				SPHFluidParticle v;
				v.pos = vec3(-(side / 2) + j, 1 - (side / 2) + k, fluidOffset - (side / 2));
				v.color = vec4(0.f, 0.f, 1.f, 1.f);
				v.normal = vec3(0, 1, 0);
				v.velocity = vec3(fluidVelx, fluidVely, fluidVelz);
				fluidParticles.push_back(v);
			}
		}
		updateFluidNeighbors(fluidTree, &fluidNeighbors, fluidParticles[0].h);
		for (int i = 0; i < fluidParticles.size(); i++){
			fluidParticles[i].pIndex = i;
			fluidParticles[i].neighbors = &fluidNeighbors;
			fluidParticles[i].particles = &fluidParticles;
			fluidVertices.push_back(fluidParticles[i]);
		}

		for (int i = 0; i < fluidParticles.size(); i++){
			fluidParticles[i].initDensity();
			//fluidParticles[i].restDensity = fluidParticles[i].density;			
		}
	}
}

void PorousFlowScene::initFluidObjects(){
	int sideLen = fluidSide;
	for (float i = 0; i < 4 * sideLen; i += 1){
		for (float j = 0; j < sideLen; j += 1){
			for (float k = 0; k < sideLen; k += 1){
				SPHFluidParticle v;
				v.pos = vec3(-(sideLen / 2) + k, -(sideLen / 2) + i + fluidOffset, -(sideLen / 2) + j);
				v.color = vec4(0.f, 0.f, 1.f, 1.f);
				v.normal = vec3(0, 1, 0);
				v.velocity = vec3(fluidVelx, fluidVely, fluidVelz);
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
		fluidParticles[i].initDensity();
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
	int sideLen = side;
	particles.resize(sideLen * sideLen * sideLen);
	for (int i = 0; i < sideLen; i++){
		for (int j = 0; j < sideLen; j++){
			for (int k = 0; k < sideLen; k++){
				index++;
				particles[index].normal = vec3(0);
				bool boundary = false;
				if (i == 0){
					particles[index].normal += vec3(0, -1, 0);
				}
				if (i == sideLen - 1){
					particles[index].normal += vec3(0, 1, 0);
				}
				if (j == 0){
					particles[index].normal += vec3(0, 0, -1);
				}
				if (j == sideLen - 1){
					particles[index].normal += vec3(0, 0, 1);
				}
				if (k == 0){
					particles[index].normal += vec3(-1, 0, 0);
				}
				if (k == sideLen - 1){
					particles[index].normal += vec3(1, 0, 0);
				}
				if (particles[index].normal == vec3(0)){
					particles[index].normal = vec3(0, 1, 0);
				}
				particles[index].normal = glm::normalize(particles[index].normal);
				particles[index].pos = vec3(-(sideLen / 2) + k, -(sideLen / 2) + i, -(sideLen / 2) + j);
				particles[index].pIndex = index;

				if (i == sideLen - 1 || j == sideLen - 1 || i == 0 || j == 0 || k == 0 || k == sideLen - 1){
					SPHFluidParticle v;
					v.pos = particles[index].pos;
					v.color = vec4(0);
					v.onboundary = true;
					v.boundaryFriend = &(particles[index]);

					fluidParticles.push_back(v);
					particles[index].onboundary = true;
					//particles[index].color = vec4(1.f, 1.f, 1.f, 0);
					/**/
					//particles[index].h = 1.f;
				}
				else{
					particles[index].color = vec4(1 - particles[index].saturation, 0.f, particles[index].saturation, 1.f);
				}
				//if (i == sideLen - 1 || k == sideLen - 1 /*|| k==0*/){
				//	particles[index].color = vec4(0.f, 0.f, 1.f, 1.f);
				//	particles[index].saturation = 1.0;//particles[index].volume * particles[index].porosity * particles[index].fluiddensity;
				//	particles[index].updateFluidMass();
				//}
				particles[index].color = vec4(1 - particles[index].saturation, 0.f, particles[index].saturation, 1.f);
				particles[index].updateCapillaryPotential();
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
	//fluidGenerator();

	fluidVertices.clear();
	updateFluidNeighbors(fluidTree, &fluidNeighbors, fluidParticles[0].h);
	for (int i = 0; i < fluidParticles.size(); i++){
		fluidParticles[i].neighbors = &fluidNeighbors;
		fluidParticles[i].particles = &fluidParticles;
		fluidParticles[i].initDensity();
		fluidVertices.push_back(fluidParticles[i]);
	}

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