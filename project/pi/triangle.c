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

#include "bcm_host.h"

#include "GLES/gl.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"

#include "vector.h"

#include "cube_texture_and_coords.h"

#define PATH "./"

#define IMAGE_SIZE 128

#ifndef M_PI
	#define M_PI 3.141592654
#endif
	

extern Vector3 gyro_data;

typedef struct
{
	uint32_t screen_width;
	uint32_t screen_height;
// OpenGL|ES objects
	EGLDisplay display;
	EGLSurface surface;
	EGLContext context;
	GLuint tex[6];
// model rotation vector and direction
	GLfloat rot_angle_x_inc;
	GLfloat rot_angle_y_inc;
	GLfloat rot_angle_z_inc;
// current model rotation angles
	GLfloat rot_angle_x;
	GLfloat rot_angle_y;
	GLfloat rot_angle_z;
// current distance from camera
	GLfloat distance;
	GLfloat distance_inc;
// pointers to texture buffers
	char *tex_buf1;
	char *tex_buf2;
	char *tex_buf3;
} CUBE_STATE_T;

//static void init_ogl(CUBE_STATE_T *state);
//static GLfloat inc_and_wrap_angle(GLfloat angle, GLfloat angle_inc);
//static GLfloat inc_and_clip_distance(GLfloat distance, GLfloat distance_inc);
static void exit_func(void);
static volatile int terminate;
static CUBE_STATE_T _state, *state=&_state;


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
static void init_ogl(CUBE_STATE_T *state)
{
	int32_t success = 0;
	EGLBoolean result;
	EGLint num_config;

	static EGL_DISPMANX_WINDOW_T nativewindow;

	DISPMANX_ELEMENT_HANDLE_T dispman_element;
	DISPMANX_DISPLAY_HANDLE_T dispman_display;
	DISPMANX_UPDATE_HANDLE_T dispman_update;
	VC_RECT_T dst_rect;
	VC_RECT_T src_rect;

	static const EGLint attribute_list[] =
	{
		EGL_RED_SIZE, 8,
		EGL_GREEN_SIZE, 8,
		EGL_BLUE_SIZE, 8,
		EGL_ALPHA_SIZE, 8,
		EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
		EGL_NONE
	};
	
	EGLConfig config;

	// get an EGL display connection
	state->display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
	assert(state->display!=EGL_NO_DISPLAY);

	// initialize the EGL display connection
	result = eglInitialize(state->display, NULL, NULL);
	assert(EGL_FALSE != result);

	// get an appropriate EGL frame buffer configuration
	result = eglChooseConfig(state->display, attribute_list, &config, 1, &num_config);
	assert(EGL_FALSE != result);

	// create an EGL rendering context
	state->context = eglCreateContext(state->display, config, EGL_NO_CONTEXT, NULL);
	assert(state->context!=EGL_NO_CONTEXT);

	// create an EGL window surface
	success = graphics_get_display_size(0 /* LCD */, &state->screen_width, &state->screen_height);
	assert( success >= 0 );

	dst_rect.x = 0;
	dst_rect.y = 0;
	dst_rect.width = state->screen_width;
	dst_rect.height = state->screen_height;
		
	src_rect.x = 0;
	src_rect.y = 0;
	src_rect.width = state->screen_width << 16;
	src_rect.height = state->screen_height << 16;        

	dispman_display = vc_dispmanx_display_open( 0 /* LCD */);
	dispman_update = vc_dispmanx_update_start( 0 );
			
	dispman_element = vc_dispmanx_element_add ( dispman_update, dispman_display,
		0/*layer*/, &dst_rect, 0/*src*/,
		&src_rect, DISPMANX_PROTECTION_NONE, 0 /*alpha*/, 0/*clamp*/, 0/*transform*/);
		
	nativewindow.element = dispman_element;
	nativewindow.width = state->screen_width;
	nativewindow.height = state->screen_height;
	vc_dispmanx_update_submit_sync( dispman_update );
		
	state->surface = eglCreateWindowSurface( state->display, config, &nativewindow, NULL );
	assert(state->surface != EGL_NO_SURFACE);

	// connect the context to the surface
	result = eglMakeCurrent(state->display, state->surface, state->surface, state->context);
	assert(EGL_FALSE != result);

	// Set background color and clear buffers
	glClearColor(0.3f, 0.3f, 0.3f, 1.0f);

	// Enable back face culling.
	//glEnable(GL_CULL_FACE);

	glMatrixMode(GL_MODELVIEW);
}

/***********************************************************
 * Name: init_model_proj
 *
 * Arguments:
 *       CUBE_STATE_T *state - holds OGLES model info
 *
 * Description: Sets the OpenGL|ES model to default values
 *
 * Returns: void
 *
 ***********************************************************/
 #if 0
static void init_model_proj(CUBE_STATE_T *state)
{
	float nearp = 1.0f;
	float farp = 500.0f;
	float hht;
	float hwd;

	glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );

	glViewport(0, 0, (GLsizei)state->screen_width, (GLsizei)state->screen_height);
		
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	hht = nearp * (float)tan(45.0 / 2.0 / 180.0 * M_PI);
	hwd = hht * (float)state->screen_width / (float)state->screen_height;

	glFrustumf(-hwd, hwd, -hht, hht, nearp, farp);
	
	glEnableClientState( GL_VERTEX_ARRAY );
	//glVertexPointer( 3, GL_BYTE, 0, quadx );

	//reset_model(state);
}

/***********************************************************
 * Name: inc_and_wrap_angle
 *
 * Arguments:
 *       GLfloat angle     current angle
 *       GLfloat angle_inc angle increment
 *
 * Description:   Increments or decrements angle by angle_inc degrees
 *                Wraps to 0 at 360 deg.
 *
 * Returns: new value of angle
 *
 ***********************************************************/
static GLfloat inc_and_wrap_angle(GLfloat angle, GLfloat angle_inc)
{
	angle += angle_inc;

	if (angle >= 360.0)
		angle -= 360.f;
	else if (angle <=0)
		angle += 360.f;

	return angle;
}

/***********************************************************
 * Name: inc_and_clip_distance
 *
 * Arguments:
 *       GLfloat distance     current distance
 *       GLfloat distance_inc distance increment
 *
 * Description:   Increments or decrements distance by distance_inc units
 *                Clips to range
 *
 * Returns: new value of angle
 *
 ***********************************************************/
static GLfloat inc_and_clip_distance(GLfloat distance, GLfloat distance_inc)
{
	distance += distance_inc;

	if (distance >= 120.0f)
		distance = 120.f;
	else if (distance <= 40.0f)
		distance = 40.0f;

	return distance;
}

#endif

/***********************************************************
 * Name: redraw_scene
 *
 * Arguments:
 *       CUBE_STATE_T *state - holds OGLES model info
 *
 * Description:   Draws the model and calls eglSwapBuffers
 *                to render to screen
 *
 * Returns: void
 *
 ***********************************************************/

static void redraw_crosshair(double rx, double ry, double rz, CUBE_STATE_T *state) {
	glClear(GL_COLOR_BUFFER_BIT);

	glLoadIdentity();
	glRotatef(rx, 1.f, 0, 0.0f);
	glRotatef(ry, 0, 1.f, 0.0f);
	glRotatef(rx, 0, 0, 1.0f);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, crosshair);
	glColorPointer(4, GL_UNSIGNED_BYTE, 0, crosshair_color);
	glDrawArrays(GL_LINES, 0, 6);
	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);
	
	eglSwapBuffers(state->display, state->surface);
}

static void exit_func(void)
// Function to be passed to atexit().
{
	// clear screen
	glClear( GL_COLOR_BUFFER_BIT );
	eglSwapBuffers(state->display, state->surface);

	// Release OpenGL resources
	eglMakeCurrent( state->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT );
	eglDestroySurface( state->display, state->surface );
	eglDestroyContext( state->display, state->context );
	eglTerminate( state->display );

	// release texture buffers
	free(state->tex_buf1);
	free(state->tex_buf2);
	free(state->tex_buf3);

	printf("\ncube closed\n");
} // exit_func()

//==============================================================================

int run_triangle ()
{
	int i;
	bcm_host_init();

	// Clear application state
	memset( state, 0, sizeof( *state ) );
		
	// Start OGLES
	init_ogl(state);

	// Setup the model world
//	init_model_proj(state);

	// initialise the OGLES texture(s)
//	init_textures(state);

	for (i = 0; !terminate; i++)
	{
//		update_model(state);
//		redraw_scene(state);
		redraw_crosshair(gyro_data.x, gyro_data.y, gyro_data.z, state);
	}
	exit_func();
	return 0;
}

