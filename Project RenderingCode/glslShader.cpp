#include <gl/glew.h>
#include "glslShader.h"
#include <iostream>

using namespace std;

glslShader::glslShader()
{
    mProg = 0;
}

glslShader::glslShader(const char *vsource, const char *fsource)
{
    mProg = compileProgram(vsource, 0, fsource);
}

glslShader::glslShader(const char *vsource, const char *gsource, const char *fsource,
                         GLenum gsInput, GLenum gsOutput)
{
    mProg = compileProgram(vsource, gsource, fsource, gsInput, gsOutput);
}

glslShader::~glslShader()
{
    if (mProg) {
        glDeleteProgram(mProg);
    }
}

void glslShader::loadFromFile(const char *v_path, const char *f_path)
{
	if (mProg) {
        glDeleteProgram(mProg);
    }
	string v= readFromFile(v_path);
	string f= readFromFile(f_path);
	mProg = compileProgram(v.c_str(), 0, f.c_str());
}
void glslShader::loadFromFile(const char *v_path, const char *tcs_path, const char *tes_path, const char *g_path, const char *f_path) {
	if (mProg) {
		glDeleteProgram(mProg);
	}
	string v = readFromFile(v_path);
	string tcs = readFromFile(tcs_path);
	string tes = readFromFile(tes_path);
	string g = readFromFile(g_path);
	string f = readFromFile(f_path);
	mProg = compileProgram(v.c_str(), g.c_str(), f.c_str(),tcs.c_str(),tes.c_str(),GL_TRIANGLES,GL_TRIANGLE_STRIP);
}

void glslShader::enable()
{
    glUseProgram(mProg);
}

void glslShader::disable()
{
    glUseProgram(0);
}

void glslShader::setUniform1i(const char *name, int value)
{
    GLint loc = glGetUniformLocation(mProg, name);
    if (loc >= 0) {
        glUniform1i(loc, value);
    } else {
#if _DEBUG
        cout << "Error setting parameter " << name << endl;
#endif
    }
}

void glslShader::setUniform1f(const char *name, float value)
{
    GLint loc = glGetUniformLocation(mProg, name);
    if (loc >= 0) {
        glUniform1f(loc, value);
    } else {
#if _DEBUG
        cout << "Error setting parameter " << name << endl;
#endif
    }
}

void glslShader::setUniform2f(const char *name, float x, float y)
{
    GLint loc = glGetUniformLocation(mProg, name);
    if (loc >= 0) {
        glUniform2f(loc, x, y);
    } else {
#if _DEBUG
        cout << "Error setting parameter " << name << endl;
#endif
    }
}

void glslShader::setUniform3f(const char *name, float x, float y, float z)
{
    GLint loc = glGetUniformLocation(mProg, name);
	cout<<name<<" "<<loc<<endl;
    if (loc >= 0) {
        glUniform3f(loc, x, y, z);
    } else {
#if _DEBUG
        cout << "Error setting parameter " << name << endl;
#endif
    }
}

void glslShader::setUniform4f(const char *name, float x, float y, float z, float w)
{
    GLint loc = glGetUniformLocation(mProg, name);
    if (loc >= 0) {
        glUniform4f(loc, x, y, z, w);
    } else {
#if _DEBUG
        cout << "Error setting parameter " << name << endl;
#endif
    }
}

void glslShader::setUniformMatrix4fv(const char * name, GLfloat * m, bool transpose)
{
	GLint loc = glGetUniformLocation(mProg, name);
	cout<<name<<" "<<loc<<endl;
	if (loc >= 0) {
		glUniformMatrix4fv(loc, 1, transpose, m);
	}
	else {
#if _DEBUG
		cout << "Error setting parameter " << name << endl;
#endif
	}
}

void glslShader::setUniformMatrix3fv(const GLchar *name, GLfloat *m, bool transpose)
{
    GLint loc = glGetUniformLocation(mProg, name);
    if (loc >= 0) {
        glUniformMatrix3fv(loc, 1, transpose, m);
    } else {
#if _DEBUG
        cout << "Error setting parameter " << name << endl;
#endif
    }
}

void glslShader::setUniformfv(const GLchar *name, GLfloat *v, int elementSize, int count)
{
    GLint loc = glGetUniformLocation(mProg, name);
    if (loc == -1) {
#ifdef _DEBUG
        cout << "Error setting parameter " << name << endl;
#endif
        return;
    }

    switch (elementSize) {
        case 1:
            glUniform1fv(loc, count, v);
            break;
        case 2:
            glUniform2fv(loc, count, v);
            break;
        case 3:
            glUniform3fv(loc, count, v);
            break;
        case 4:
            glUniform4fv(loc, count, v);
            break;
    }
}




GLuint glslShader::checkCompileStatus(GLuint shader, GLint *status)
{
    glGetShaderiv(shader, GL_COMPILE_STATUS, status);
    if (!(*status)) {
         char log[2048];
         int len;
         glGetShaderInfoLog(shader, 2048, (GLsizei*)&len, log);
         cout << "Error: shader("<< (int)shader << "), Info log: " << log << endl;
         glDeleteShader(shader);
         return 0;
    }
    return 1;
}

GLuint glslShader::compileProgram(const char *vsource, const char *gsource, const char *fsource,
                            GLenum gsInput, GLenum gsOutput)
{
	
    GLuint vertexShader   = glCreateShader(GL_VERTEX_SHADER);
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

    GLint compiled = 0;

    glShaderSource(vertexShader, 1, &vsource, 0);
    glShaderSource(fragmentShader, 1, &fsource, 0);

    glCompileShader(vertexShader);
    if (checkCompileStatus(vertexShader, &compiled) == 0) {
        cout << "<compileProgram compilation error with vertexShader>:" << endl;
        cout << vsource << endl;
       return 0;
    }

    glCompileShader(fragmentShader);
    if (checkCompileStatus(fragmentShader, &compiled) == 0) {
        cout << "<compileProgram compilation error with fragmentShader>:" << endl;
        cout << fsource << endl;
       return 0;
    }

    GLuint program = glCreateProgram();

    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    if (gsource) {
        GLuint geomShader = glCreateShader(GL_GEOMETRY_SHADER_EXT);
        glShaderSource(geomShader, 1, &gsource, 0);
        glCompileShader(geomShader);
        glGetShaderiv(geomShader, GL_COMPILE_STATUS, (GLint*)&compiled);
        if (checkCompileStatus(geomShader, &compiled) == 0) {
            cout << "<compileProgram compilation error with geomShader>:" << endl;
            cout << gsource << endl;
           return 0;
        }

        glAttachShader(program, geomShader);

        glProgramParameteriEXT(program, GL_GEOMETRY_INPUT_TYPE_EXT, gsInput);
        glProgramParameteriEXT(program, GL_GEOMETRY_OUTPUT_TYPE_EXT, gsOutput); 
        glProgramParameteriEXT(program, GL_GEOMETRY_VERTICES_OUT_EXT, 4); 
    }

    glLinkProgram(program);

    // check if program linked
    GLint success = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &success);

    if (!success) {
        char temp[1024];
        glGetProgramInfoLog(program, 1024, 0, temp);
        cout << "Failed to link program:" << endl << temp << endl;
        glDeleteProgram(program);
        program = 0;

    }

    return program;
}
GLuint glslShader::compileProgram(const char *vsource, const char *gsource, const char *fsource,
	const char *tc_source, const char *te_source, GLenum gsInput, GLenum gsOutput)
{
	//if(!glExtChecker::instance()->shadersupported) return 0;
	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	GLuint tessconShader = glCreateShader(GL_TESS_CONTROL_SHADER);
	GLuint tessevalShader = glCreateShader(GL_TESS_EVALUATION_SHADER);

	GLint compiled = 0;
	           
	glShaderSource(vertexShader, 1, &vsource, 0);
	glShaderSource(fragmentShader, 1, &fsource, 0);
	glShaderSource(tessconShader, 1, &tc_source, 0);
	glShaderSource(tessevalShader, 1, &te_source, 0);

	glCompileShader(vertexShader);
	if (checkCompileStatus(vertexShader, &compiled) == 0) {
		cout << "<compileProgram compilation error with vertexShader>:" << endl;
		cout << vsource << endl;
		return 0;
	}

	glCompileShader(fragmentShader);
	if (checkCompileStatus(fragmentShader, &compiled) == 0) {
		cout << "<compileProgram compilation error with fragmentShader>:" << endl;
		return 0;
	}
	glCompileShader(tessconShader);
	if (checkCompileStatus(tessconShader, &compiled) == 0) {
		cout << "<compileProgram compilation error with tessconShader>:" << endl;
		return 0;
	}
	glCompileShader(tessevalShader);
	if (checkCompileStatus(tessevalShader, &compiled) == 0) {
		cout << "<compileProgram compilation error with tessevalShader>:" << endl;
		return 0;
	}

	GLuint program = glCreateProgram();

	glAttachShader(program, vertexShader);
	glAttachShader(program, fragmentShader);
	glAttachShader(program, tessconShader);
	glAttachShader(program, tessevalShader);
	if (gsource) {
		GLuint geomShader = glCreateShader(GL_GEOMETRY_SHADER_EXT);
		glShaderSource(geomShader, 1, &gsource, 0);
		glCompileShader(geomShader);
		glGetShaderiv(geomShader, GL_COMPILE_STATUS, (GLint*)&compiled);
		if (checkCompileStatus(geomShader, &compiled) == 0) {
			cout << "<compileProgram compilation error with geomShader>:" << endl;
			cout << gsource << endl;
			return 0;
		}

		glAttachShader(program, geomShader);

		glProgramParameteriEXT(program, GL_GEOMETRY_INPUT_TYPE_EXT, gsInput);
		glProgramParameteriEXT(program, GL_GEOMETRY_OUTPUT_TYPE_EXT, gsOutput);
		glProgramParameteriEXT(program, GL_GEOMETRY_VERTICES_OUT_EXT, 4);
	}

	glLinkProgram(program);

	// check if program linked
	GLint success = 0;
	glGetProgramiv(program, GL_LINK_STATUS, &success);

	if (!success) {
		char temp[1024];
		glGetProgramInfoLog(program, 1024, 0, temp);
		cout << "Failed to link program:" << endl << temp << endl;
		glDeleteProgram(program);
		program = 0;
	}
	glPatchParameteri(GL_PATCH_VERTICES, 3);
	return program;
}

#include <fstream>

string glslShader::readFromFile(const char *filePath) {
	std::string content;
    std::ifstream fileStream(filePath, std::ios::in);

    if(!fileStream.is_open()) {
        std::cerr << "Could not read file " << filePath << ". File does not exist." << std::endl;
        return "";
    }

    std::string line = "";
    while(!fileStream.eof()) {
        std::getline(fileStream, line);
        content.append(line + "\n");
    }

    fileStream.close();
    return content;
}