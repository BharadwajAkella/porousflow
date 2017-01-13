#ifndef SCENE_H
#define SCENE_H

#include "Object.h"
#include "callbacks.h"

#include <vector>

class Scene : public Callbacks{
public:
	Scene(){}

	~Scene(){}

	virtual void renderSceneCB(){}

	virtual void keyboardCB(unsigned char key, int x, int y){}

	virtual void specialKeyboardCB(int key, int x, int y){}

	virtual void motionCB(int x, int y){}

	virtual void mouseCB(int button, int state, int x, int y){}

	virtual void addObject(Object* obj){}

	virtual void passiveMouseCB(int x, int y){}

	virtual void idleCB(){}

	virtual void initializeCamera(){}

	virtual void compileShaders(){}

	virtual void initObjects(){}

	virtual void timerCB(int value){}

	void updateObjects(){}	

};
#endif
