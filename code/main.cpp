#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <fstream>

#include "Scene.cpp"
#include "Mesh.h"
#include <vector>

using std::cout; 
using std::endl;

PorousFlowScene* scene = new PorousFlowScene();

static void renderSceneCB()
{
	scene->renderSceneCB();
}

void keyboardCB(unsigned char key, int x, int y){
	scene->keyboardCB(key, x, y);
}

void specialFuncCB(int key, int x, int y){
	scene->specialKeyboardCB(key, x, y);
}

void motionCB(int x, int y) {
	scene->motionCB(x, y);
}

void mouseCB(int button, int state, int x, int y){
	scene->mouseCB(button, state, x, y);
}

void timerCB(int value){
	scene->timerCB(value);
}

static void InitializeGlutCallbacks(){
	glutDisplayFunc(renderSceneCB);
	glutIdleFunc(renderSceneCB);
	glutKeyboardFunc(keyboardCB);
	glutSpecialFunc(specialFuncCB);
	glutMotionFunc(motionCB);
	glutTimerFunc(60, timerCB, 0);
	glutMouseFunc(mouseCB);
}

void initialize(int argc, char **argv){
	

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(windowWidth, windowHeight);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Springs and Integrators");
	InitializeGlutCallbacks();

	
}

void writeToFile(){
	
}

int main(int argc, char** argv){
	//cout << "Do you want to print to :" << endl << "1 Screen" << endl << "2 File" << endl;
	//int printTo;
	//std::cin >> printTo;

	//if (printTo == 2){
		//std::ofstream out("out.txt");
		//std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
		//std::cout.rdbuf(out.rdbuf());
	//}
	initialize(argc, argv);
	GLenum res = glewInit();
	if (res != GLEW_OK) {
		cout << stderr << " Error: " << glewGetErrorString(res) << endl;
		return 1;
	}

	cout << "GL version: " << glGetString(GL_VERSION) << endl;
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glFrontFace(GL_CW);
	/*glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);*/
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	scene->initObjects();
	scene->initTexture("tex.png");


	glutMainLoop();
	
	delete scene;

	return 0;
}
