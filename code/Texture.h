#ifndef TEXTURE_H
#define	TEXTURE_H

#include <string>

#include <GL/glew.h>
#include "FreeImage.h"
using std::string;

class Texture{
public:
	Texture(GLenum p_textureTarget, const string& p_filename);
	~Texture();
	bool Load();
	void Bind(GLenum texUnit);
private:
	string filename;
	GLenum textureTarget;
	GLuint textureObject;

};

#endif