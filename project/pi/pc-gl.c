/*
Copyright (c) 2015 Steven Arnow <s@rdw.se>
'glx_window.c' - This file is part of libdangit

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

	1. The origin of this software must not be misrepresented; you must not
	claim that you wrote the original software. If you use this software
	in a product, an acknowledgment in the product documentation would be
	appreciated but is not required.

	2. Altered source versions must be plainly marked as such, and must not be
	misrepresented as being the original software.

	3. This notice may not be removed or altered from any source
	distribution.
*/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <X11/X.h>
#include <X11/Xlib.h>
#include <GL/glx.h>

#define SCREEN_W 800
#define SCREEN_H 600

typedef struct State State;
struct State {
	Display	*dpy;
	Window	win;
};

void *ogl_state_new() {
	return malloc(sizeof(State));
}

bool ogl_init(State *state) {
	Display *dpy;
	Window rwin, main_win;
	XVisualInfo *xv;
	Colormap cm;
	XSetWindowAttributes xswa;
	GLXContext glc;
	int cd, attrib[16], i = 0;
	XSizeHints sh;

	if (!(dpy = XOpenDisplay(NULL))) {
		fprintf(stderr, "Unable to open a connection to X11");
		return false;
	}

	rwin = DefaultRootWindow(dpy);
	cd = 8;
	attrib[i++] = GLX_RGBA;
	attrib[i++] = GLX_DOUBLEBUFFER;

	attrib[i++] = GLX_RED_SIZE;
	attrib[i++] = cd;
	attrib[i++] = GLX_GREEN_SIZE;
	attrib[i++] = cd;
	attrib[i++] = GLX_BLUE_SIZE;
	attrib[i++] = cd;
	attrib[i++] = GLX_ALPHA_SIZE;
	attrib[i++] = 8;
	attrib[i++] = GLX_STENCIL_SIZE;
	attrib[i++] = 8;
	attrib[i++] = GLX_DEPTH_SIZE;
	attrib[i++] = 16;
	attrib[i++] = None;

	if (!(xv = glXChooseVisual(dpy, 0, attrib))) {
		/* TODO: Print requested XVisual info */
		fprintf(stderr, "GLX was unable to find a suitable visual");
		goto badvisual;
	}
	
	cm = XCreateColormap(dpy, rwin, xv->visual, AllocNone);
	xswa.colormap = cm;
	xswa.event_mask = ExposureMask | KeyPressMask;
	
	if (!(main_win = XCreateWindow(dpy, rwin, 0, 0, SCREEN_W, SCREEN_H,
			0, xv->depth, InputOutput, xv->visual, CWColormap | CWEventMask, &xswa))) {
		fprintf(stderr, "X11 was unable to create the window");
		goto nowindow;
	}

	sh.min_width = sh.max_width = sh.base_width = SCREEN_W;
	sh.min_height = sh.max_height = sh.base_height = SCREEN_H;
	sh.flags = PBaseSize | PMinSize | PMaxSize;
	XSetWMNormalHints(dpy, main_win, &sh);

	XMapWindow(dpy, main_win);
	XStoreName(dpy, main_win, "sensys");
	if (!(glc = glXCreateContext(dpy, xv, NULL, GL_TRUE))) {
		fprintf(stderr, "glX was unable to create the OpenGL Context");
		goto nocontext;
	}

	glXMakeCurrent(dpy, main_win, glc);

	state->win = main_win;
	state->dpy = dpy;
	
	XFree(xv);
	
	glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
	
	glDepthMask(GL_TRUE);
	glDepthFunc(GL_LESS);

	return true;
	
	// Error handlers below
nocontext:
	XDestroyWindow(dpy, main_win); 
nowindow:
	XFreeColormap(dpy, cm);
	XFree(xv);
badvisual:
	XCloseDisplay(dpy);
	return false;
}


void ogl_flip(State *state) {
	glXSwapBuffers(state->dpy, state->win);
	glClear(GL_COLOR_BUFFER_BIT);
}

void ogl_exit(void) {
}
