#ifndef __GL_H_
#define __GL_H_

#include <stdbool.h>

#ifdef HAVE_LIBBCM_HOST
#include "GLES/gl.h"
#include <GLES/glu.h>
#else
#include <GL/glx.h>
#include <GL/glu.h>
#endif

typedef void State;

void *ogl_state_new();
bool ogl_init(State *state);
void ogl_flip(State *state);
void ogl_exit(State *state);

void camera_move(double x, double y, double z);
void camera_rotate(double yaw, double pitch);
void camera_zoom_in(double zoom);
void camera_reset();

void print_camera();
void init_grid();

void launch_worker();
int run_triangle();

#endif
