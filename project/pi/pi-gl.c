#include <stdbool.h>
#include "bcm_host.h"

#include "GLES/gl.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"

typedef struct State State;
struct State {
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
};


void *ogl_state_new() {
	return malloc(sizeof(State));
}

bool init_ogl(State *state)
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
	
	bcm_host_init();
	// Clear application state
	memset( state, 0, sizeof( *state ) );
	
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
	return true;
}

void ogl_flip(State *state) {
	eglSwapBuffers(state->display, state->surface);
	glClear(GL_COLOR_BUFFER_BIT);
}

void ogl_exit(void) {
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
}
