#include "glslprogram.h"



#include <fstream>
using std::ifstream;
using std::ios;

#include <sstream>
#include <sys/stat.h>

GLSLProgram::GLSLProgram()  {
	programHandle = 0;
	linked = false;
}

GLSLProgram::~GLSLProgram(){
	if (programHandle == 0){
		return;
	}

	//Find out the number of shaders attached
	GLint numShaders = 0;
	glGetProgramiv(programHandle, GL_ATTACHED_SHADERS, &numShaders);

	//Get the names of those shaders
	GLuint *shaderNames = new GLuint[numShaders];
	glGetAttachedShaders(programHandle, numShaders, NULL, shaderNames);

	//Delete the shaders
	for (int i = 0; i < numShaders; i++){
		glDeleteShader(shaderNames[i]);
	}

	//Delete the program
	glDeleteProgram(programHandle);

	delete[] shaderNames;

}

void GLSLProgram::compileShader(const char *filename, GLSLShader::GLSLShaderType type) throw (GLSLProgramException){
	if (! fileExists(filename)){
		string message = string("Shader ") + filename + " not found. ";
		throw(GLSLProgramException(message));
	}
	if (programHandle <= 0){
		programHandle = glCreateProgram();
		if (programHandle = 0){
			throw(GLSLProgramException("Could not create shader program"));
		}
	}

	ifstream file(filename, ios::in);
	if (!file){
		string message = string ("Could not open the file ") + filename;
		throw(GLSLProgramException(message));
	}

	std::stringstream code;
	code << file.rdbuf();
	file.close();
	
	compileShader(code.str(), type, filename);
}

void GLSLProgram::compileShader(const string &source, GLSLShader::GLSLShaderType type, const char *filename)
																					throw (GLSLProgramException){
	if (programHandle <= 0){
		programHandle = glCreateProgram();
		if (programHandle == 0) {
			throw GLSLProgramException("Unable to create shader program.");
		}
	}

	GLuint shaderHandle = glCreateShader(type);
	
	//Attach source code to the shader
	const char *code = source.c_str();
	glShaderSource(shaderHandle, 1, &code, NULL);

	//Compile shader
	glCompileShader(shaderHandle);

	//Check for errors
	int result;
	glGetShaderiv(shaderHandle, GL_COMPILE_STATUS, &result);
	
	if (GL_FALSE == result){
		int length = 0; 
		string log;
		glGetShaderiv(shaderHandle, GL_INFO_LOG_LENGTH, &length);
		if ( length > 0){
			char *tempLog = new char[length];
			int cursor = 0;
			glGetShaderInfoLog(shaderHandle, length, &cursor, tempLog);
			log = string(tempLog);
			delete[] tempLog;
		}
		string message;
		if (filename){
			message = string(filename) + " : shader compilation failed \n";
		}
		else{
			message = " Shader compilation failed \n";
		}
		message += (log);
		throw(GLSLProgramException(message));
	}
	else{
		glAttachShader(programHandle, shaderHandle);
	}


	


}

void GLSLProgram::link() throw (GLSLProgramException){
	if (linked)
		return;
	if (programHandle <= 0){
		throw(GLSLProgramException("The shader program is not yet compiled."));
	}
	glLinkProgram(programHandle);

	int status = 0; 
	glGetProgramiv(programHandle, GL_LINK_STATUS, &status);
	if (GL_FALSE == status){
		int length = 0;
		string log;
		glGetProgramiv(programHandle, GL_INFO_LOG_LENGTH, &length);

		if (length > 0){
			char *tempLog = new char[length];
			int cursor = 0;
			glGetProgramInfoLog(programHandle, length, &cursor, tempLog);
			log = string(tempLog);
			delete[] tempLog;
		}
		throw GLSLProgramException(string("Program link failed:\n") + log);
	}
	else{
		uniformLocations.clear();
		linked = true;
	}
}

void GLSLProgram::use() throw(GLSLProgramException){
	if ( programHandle <= 0 || !linked ){
		throw GLSLProgramException("Shader has not been linked to the program");
	}
	glUseProgram(programHandle);
}

int GLSLProgram::getHandle(){
	return programHandle;
}

bool GLSLProgram::isLinked(){
	return linked;
}

void GLSLProgram::bindAttribLocation( GLuint location, const char * name ){
	glBindAttribLocation(programHandle, location, name);
}

void GLSLProgram::bindFragDataLocation(GLuint location, const char * name){
	glBindFragDataLocation(programHandle, location, name);
}

void GLSLProgram::setUniform(const char *name, float x, float y, float z){
	GLint loc = getUniformLocation(name);
	glUniform3f(loc, x, y, z);
}

void GLSLProgram::setUniform(const char *name, const vec3 & v){
	this->setUniform(name, v.x, v.y, v.z);
}

void GLSLProgram::setUniform(const char *name, const vec4 & v){
	GLint loc = getUniformLocation(name);
	glUniform4f(loc, v.x, v.y, v.z, v.w);
}

void GLSLProgram::setUniform(const char *name, const vec2 & v){
	GLint loc = getUniformLocation(name);
	glUniform2f(loc, v.x, v.y);
}

void GLSLProgram::setUniform(const char *name, const mat4 & m){
	GLint loc = getUniformLocation(name);
	glUniformMatrix4fv(loc, 1, GL_FALSE, &m[0][0]);
}

void GLSLProgram::setUniform(const char *name, const mat3 & m){
	GLint loc = getUniformLocation(name);
	glUniformMatrix3fv(loc, 1, GL_FALSE, &m[0][0]);
}

void GLSLProgram::setUniform(const char *name, float val){
	GLint loc = getUniformLocation(name);
	glUniform1f(loc, val);
}

void GLSLProgram::setUniform(const char *name, int val){
	GLint loc = getUniformLocation(name);
	glUniform1i(loc, val);
}

void GLSLProgram::setUniform(const char *name, bool val){
	GLint loc = getUniformLocation(name);
	glUniform1i(loc, val);
}
void GLSLProgram::setUniform(const char *name, GLuint val){
	GLint loc = getUniformLocation(name);
	glUniform1ui(loc, val);
}

/*
void GLSLProgram::printActiveUniforms(){
	GLint numUniforms = 0;
	glGetProgramInterfaceiv(programHandle, GL_UNIFORM, GL_ACTIVE_RESOURCES, &numUniforms);

	GLenum properties[] = { GL_NAME_LENGTH, GL_TYPE, GL_LOCATION, GL_BLOCK_INDEX };

	for (int i = 0; i < numUniforms; i++){
		GLint results[4];
		glGetProgramResourceiv(programHandle, GL_UNIFORM, i, 4, properties, 4, NULL, results);

		if (results[3] != -1){
			continue;
		}
		GLint nameLength = results[0] + 1;
		char *name = new char[nameLength];
		glGetProgramResourceName(programHandle, GL_UNIFORM, i, 4, NULL, name);
		printf("%-5d %s (%s)\n", results[2], name, getTypeString(results[1]));
		delete[] name;
	}
}
*/  

void GLSLProgram::validate() throw (GLSLProgramException){
	if (!isLinked()){
		throw GLSLProgramException("Program is not linked.");
	}
	GLint status;
	glValidateProgram(programHandle);
	glGetProgramiv(programHandle, GL_VALIDATE_STATUS, &status);

	if (GL_FALSE == status){
		int length = 0;
		string log;
		glGetProgramiv(programHandle, GL_INFO_LOG_LENGTH, &length);

		if (length > 0){
			char *tempLog = new char[length];
			int cursor = 0;
			glGetProgramInfoLog(programHandle, length, &cursor, tempLog);
			log = string(tempLog);
			delete[] tempLog;
		}
		throw GLSLProgramException(string("Program failed to validate\n") + log);
	}
}

int GLSLProgram::getUniformLocation(const char *name){
	std::map<string, int> ::iterator pos;
	pos = uniformLocations.find(name);
	if (pos == uniformLocations.end()){
		uniformLocations[name] = glGetUniformLocation(programHandle, name);
	}
	return uniformLocations[name];
}

bool GLSLProgram::fileExists(const string &filename){
	struct stat info;
	int ret = -1;

	ret = stat(filename.c_str(), &info);
	return 0 == ret;
}