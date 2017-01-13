#ifndef GLSLPROGRAM_H
#define GLSLPROGRAM_H

//#ifdef WIN32
#pragma warning( disable : 4290 )
//#endif

#include <string>
using std::string;

#include <map>
#include <glm/glm.hpp>
using glm::vec2;
using glm::vec3;
using glm::vec4;
using glm::mat3;
using glm::mat4;

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <stdexcept>
#include <iostream>

class GLSLProgramException : public std::runtime_error{
public:
	GLSLProgramException(const string& msg) : 
		std::runtime_error(msg){
			std::cout << msg << std::endl;
		}
};

namespace GLSLShader {
	enum GLSLShaderType {
		VERTEX = GL_VERTEX_SHADER,
		FRAGMENT = GL_FRAGMENT_SHADER,
		GEOMETRY = GL_GEOMETRY_SHADER,
		TESS_CONTROL = GL_TESS_CONTROL_SHADER,
		TESS_EVALUATION = GL_TESS_EVALUATION_SHADER,
		COMPUTE = GL_COMPUTE_SHADER
	};
};

class GLSLProgram{
private:
	int programHandle;
	bool linked;
	std::map<string, int> uniformLocations;

	GLint getUniformLocation(const char* name);
	bool fileExists(const string & filename);
	
	GLSLProgram(const GLSLProgram &other);
	GLSLProgram & operator=(const GLSLProgram &other) { return *this; }

public:
	GLSLProgram();
	~GLSLProgram();

	//void compileShader(const char *filename) throw (GLSLProgramException);
	void compileShader(const char *filename, GLSLShader::GLSLShaderType type) throw (GLSLProgramException);
	void compileShader(const string & source, GLSLShader::GLSLShaderType type, const char *filename = NULL) 
																			  throw (GLSLProgramException);
	
	void link() throw (GLSLProgramException);
	void validate() throw (GLSLProgramException);
	void use() throw (GLSLProgramException);

	int getHandle();
	bool isLinked();

	void bindAttribLocation(GLuint location, const char *name);
	void bindFragDataLocation(GLuint location, const char *name);

	void setUniform(const char *name, float x, float y, float z);
	void setUniform(const char *name, const vec2 &v);
	void setUniform(const char *name, const vec3 &v);
	void setUniform(const char *name, const vec4 &v);
	void setUniform(const char *name, const mat3 &v);
	void setUniform(const char *name, const mat4 &v);
	void setUniform(const char *name, GLuint val);
	void setUniform(const char *name, int val);
	void setUniform(const char *name, bool val);
	void setUniform(const char *name, float val);

	/*void printActiveUniforms();
	void printActiveUniformBlocks();
	void printActiveAttribs();*/

	const char* getTypeString(GLenum type);

};


#endif