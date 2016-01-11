/*
Copyright (c) 2012, Broadcom Europe Ltd
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
	 * Redistributions of source code must retain the above copyright
		notice, this list of conditions and the following disclaimer.
	 * Redistributions in binary form must reproduce the above copyright
		notice, this list of conditions and the following disclaimer in the
		documentation and/or other materials provided with the distribution.
	 * Neither the name of the copyright holder nor the
		names of its contributors may be used to endorse or promote products
		derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// A rotating cube rendered with OpenGL|ES. Three images used as textures on the cube faces.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>

#include "gl.h"
#include "vector.h"

#include "cube_texture_and_coords.h"

#define PATH "./"

#define IMAGE_SIZE 128

#ifndef M_PI
	#define M_PI 3.141592654
#endif
	

extern Vector3 gyro_data;
extern IMUVector accumulated_imu[6];


//static void init_ogl(CUBE_STATE_T *state);
//static GLfloat inc_and_wrap_angle(GLfloat angle, GLfloat angle_inc);
//static GLfloat inc_and_clip_distance(GLfloat distance, GLfloat distance_inc);
static volatile int terminate;


/***********************************************************
 * Name: init_ogl
 *
 * Arguments:
 *       CUBE_STATE_T *state - holds OGLES model info
 *
 * Description: Sets the display, OpenGL|ES context and screen stuff
 *
 * Returns: void
 *
 ***********************************************************/

static void redraw_crosshair(double rx, double ry, double rz, double px, double py, double pz) {
	glLoadIdentity();
	glTranslatef(px, py, pz);
	glRotatef(rx*180.0/M_PI, 1.f, 0, 0.0f);
	glRotatef(ry*180.0/M_PI, 0, 1.f, 0.0f);
	glRotatef(rz*180.0/M_PI, 0, 0, 1.0f);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, crosshair);
	glColorPointer(4, GL_UNSIGNED_BYTE, 0, crosshair_color);
	glDrawArrays(GL_LINES, 0, 6);
	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);
}

int run_triangle () {
	int i;
	State *state;
	
	state = ogl_state_new();
	
	ogl_init(state);
	for (i = 0; !terminate;) {
		for (i = 0; i < 2; i++)
			redraw_crosshair(accumulated_imu[i].gyro.x, accumulated_imu[i].gyro.y, accumulated_imu[i].gyro.z, accumulated_imu[i].acc.x, accumulated_imu[i].acc.y, accumulated_imu[i].acc.z);
		ogl_flip(state);
	}
	
	ogl_exit();
	return 0;
}

