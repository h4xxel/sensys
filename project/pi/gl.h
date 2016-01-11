#ifndef __GL_H_
#define __GL_H_

#include <stdbool.h>

#ifdef HAVE_LIBBCM_HOST
#include "GLES/gl.h"
#else
#include <GL/glx.h>
#endif

typedef void State;

void *ogl_state_new();
bool ogl_init(State *state);
void ogl_flip(State *state);
void ogl_exit(State *state);

#endif
