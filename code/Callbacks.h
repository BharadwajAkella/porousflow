#ifndef CALLBACKS_H
#define CALLBACKS_H

#include <stdio.h>
#include <GL/glew.h>
#include <GL/freeglut.h>


class Callbacks
{
public:

	virtual void specialKeyboardCB(int Key, int x, int y) = 0;

	virtual void keyboardCB(unsigned char Key, int x, int y) = 0;

	virtual void passiveMouseCB(int x, int y) = 0;

	virtual void renderSceneCB() = 0;

	virtual void idleCB() = 0;

	virtual void mouseCB(int Button, int State, int x, int y) = 0;
};

#endif
