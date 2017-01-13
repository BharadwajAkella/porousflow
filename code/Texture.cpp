#include "texture.h"
#include <iostream>

Texture::Texture(GLenum p_textureTarget, const string &p_filename){
	textureTarget = p_textureTarget;
	filename = p_filename;
}

bool Texture::Load(){
	
	FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;
	FIBITMAP *dib(0);

	fif = FreeImage_GetFileType(filename.c_str(), 0);

	if (fif == FIF_UNKNOWN)
		fif = FreeImage_GetFIFFromFilename(filename.c_str());
	if (fif == FIF_UNKNOWN)
		return false;
	if (FreeImage_FIFSupportsReading(fif))
		dib = FreeImage_Load(fif, filename.c_str());
	if (!dib)
		return false;

	GLuint* pixels  ;
	GLuint width ;
	GLuint height ;
	pixels = (GLuint*)FreeImage_GetBits(dib);
	width = FreeImage_GetWidth(dib);
	height = FreeImage_GetHeight(dib);
	glGenTextures(1, &textureObject);
	glBindTexture(textureTarget, textureObject);
	glTexImage2D(textureTarget, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
	glTexParameterf(textureTarget, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameterf(textureTarget, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	FreeImage_Unload(dib);
	return true;
}

void Texture::Bind(GLenum texUnit){
	glActiveTexture(texUnit);
	glBindTexture(textureTarget, textureObject);
}

Texture::~Texture(){
	glDeleteTextures(1, &textureObject);
}
