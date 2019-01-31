#pragma once

#include <gl/glut.h>
#include <string>

class glslShader  {
public:
	glslShader();
	glslShader(const char *vsource, const char *fsource);
    glslShader(const char *vsource, const char *gsource, const char *fsource,
                GLenum gsInput = GL_POINTS, GLenum gsOutput = GL_TRIANGLE_STRIP);

	~glslShader();


	void enable();
	void disable();
	void loadFromFile(const char *v_path, const char *f_path);
	void loadFromFile(const char *v_path, const char *tcs_path, const char *tes_path, const char *g_path, const char *f_path);
	void setUniform1i(const char *name, GLint x);
    void setUniform1f(const char *name, GLfloat x);
    void setUniform2f(const char *name, GLfloat x, GLfloat y);
    void setUniform3f(const char *name, float x, float y, float z);
    void setUniform4f(const char *name, float x, float y, float z, float w);
    void setUniformfv(const char *name, GLfloat *v, int elementSize, int count=1);
    void setUniformMatrix4fv(const char *name, GLfloat *m, bool transpose);
	void setUniformMatrix3fv(const char *name, GLfloat *m, bool transpose);

    inline GLuint getProgId() { return mProg; }

private:
	std::string readFromFile(const char *filePath);
    GLuint checkCompileStatus(GLuint shader, GLint *status);
    GLuint compileProgram(const char *vsource, const char *gsource, const char *fsource,
                          GLenum gsInput = GL_POINTS, GLenum gsOutput = GL_TRIANGLE_STRIP);

	GLuint compileProgram(const char *vsource, const char *gsource, const char *fsource,
				const char *tc_source, const char *te_source, GLenum gsInput = GL_POINTS, GLenum gsOutput = GL_TRIANGLE_STRIP);
    GLuint mProg;
};//class
