#ifndef __GL_H_
#define __GL_H_

#include <stdbool.h>

#ifdef HAVE_LIBBCM_HOST
#include "GLES/gl.h"
void gluLookAt(float eyeX, float eyeY, float eyez, float centerX, float centerY, float centerZ, float upX, float upY, float upZ);
void gluPerspective(float fovy, float aspect, float zNear, float zFar);
#else
#include <GL/glx.h>
#endif

typedef void State;

void *ogl_state_new();
bool ogl_init(State *state);
void ogl_flip(State *state);
void ogl_exit(State *state);

void camera_move(double x, double y, double z);
void camera_rotate(double yaw, double pitch);

#endif
