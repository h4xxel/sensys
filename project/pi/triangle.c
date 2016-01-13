#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>

#include "gl.h"
#include "vector.h"
#include "bone.h"

#include "cube_texture_and_coords.h"

#define PATH "./"

#define IMAGE_SIZE 128

#ifndef M_PI
	#define M_PI 3.141592654
#endif

#define INITIAL_ZOOM 3.0

Vector3f xy_grid[(51+51) * 2];
Vector3f xz_grid[(51+51) * 2];
Vector3f yz_grid[(51+51) * 2];

extern Vector3 gyro_data;
extern IMUVector accumulated_imu[6];
extern IMUPosition imu_position[6];
extern struct BoneRender *bonerender;
extern int bones;
extern IMUVector gravity[6];
extern IMUVector imu_data[6];
extern Vector3 accel_no_grav[6];

double camera_yaw, camera_pitch, camera_zoom = INITIAL_ZOOM;
Vector3 camera = {0.0, 0.0, INITIAL_ZOOM};

static void draw_line(Vector3 *start, Vector3 *end, uint8_t r, uint8_t g, uint8_t b);
static volatile int terminate;
static State *state;


void create_grid(Vector3f *grid, float dx, float dy, float dz, float px, float py, float pz, float lx, float ly, float lz) {
	int i;

	for (i = 0; i < 51; i++) {
		grid[i*2].x = px + dx * i;
		grid[i*2+1].x = px + dx * i + lx;
		grid[i*2].y = py + dy * i;
		grid[i*2+1].y = py + dy * i + ly;
		grid[i*2].z = pz + dz * i;
		grid[i*2+1].z = pz + dz * i + lz;
	}
}


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

static void redraw_crosshair(double rx, double ry, double rz, double px, double py, double pz);


static void redraw_bones() {
	int i, j;

	bone_recalculate();
	glPushMatrix();

//	for (i = 0; i < bones; i++) {
		glEnableClientState(GL_VERTEX_ARRAY);
		
		glVertexPointer(3, GL_FLOAT, 0, bonerender);
		glDrawArrays(GL_LINES, 0, 2 * bones);

		glDisableClientState(GL_VERTEX_ARRAY);

//	}

	for (i = 0; i < bones; i++) {
		if ((j = bone[i].imu_id) < 0)
			continue;
		redraw_crosshair(accumulated_imu[j].gyro.x, accumulated_imu[j].gyro.y, accumulated_imu[j].gyro.z, imu_position[j].pos.x, imu_position[j].pos.y, imu_position[j].pos.z);
		draw_line(&imu_position[j].pos, &gravity[j].acc, 0, 0, 255);
		draw_line(&imu_position[j].pos, &gravity[j].gyro, 0 ,255, 0);
		draw_line(&imu_position[j].pos, &accel_no_grav[j], 255, 0, 0);
		//redraw_crosshair(accumulated_imu[j].gyro.x, accumulated_imu[j].gyro.y, accumulated_imu[j].gyro.z, 0.3 + 0.3 * i, 0, 0);
		
		
	}

	glPopMatrix();
}


static void redraw_crosshair(double rx, double ry, double rz, double px, double py, double pz) {
	glPushMatrix();
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
	glPopMatrix();
}

static void draw_line(Vector3 *start, Vector3 *end, uint8_t r, uint8_t g, uint8_t b) {
	uint8_t color[8] = {0, 0, 0, 255, r, g, b, 255};
	float point[6] = {0.0, 0.0, 0.0, end->x, end->y, end->z};
	
	glPushMatrix();
	glTranslatef(start->x, start->y, start->z);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	
	glVertexPointer(3, GL_FLOAT, 0, point);
	glColorPointer(4, GL_UNSIGNED_BYTE, 0, color);
	
	glDrawArrays(GL_LINES, 0, 2);
	
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
	glPopMatrix();
}

void draw_grid() {
	glColor4f(0.2f, 0.2f, 0.2f, 1.0f);
	glLineWidth(2);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, xy_grid);
	glDrawArrays(GL_LINES, 0, 2 * 51 * 2);
	glVertexPointer(3, GL_FLOAT, 0, xz_grid);
	glDrawArrays(GL_LINES, 0, 2 * 51 * 2);
	//glVertexPointer(3, GL_FLOAT, 0, yz_grid);
	//glDrawArrays(GL_LINES, 0, 2 * 51 * 2);
	glDisableClientState(GL_VERTEX_ARRAY);
	glColor4f(1.0, 1.0f, 1.0f, 1.0f);
	glLineWidth(8);
}


static void camera_apply() {
	#if 1
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
        
	gluPerspective(45.0, 1.0, 0.001, 100.0);
	gluLookAt(camera.x, camera.y, camera.z, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	glRotatef(90, 1.0, 0.0, 0.0);
	glPushMatrix();
	#endif
}

void camera_move(double x, double y, double z) {
	camera.x += x;
	camera.y += y;
	camera.z += z;
}

void camera_rotate(double pitch, double yaw) {
	camera_yaw += yaw;
	camera_pitch += pitch;
	if(pitch > M_PI)
		pitch = M_PI;
	if(pitch < 0.0)
		pitch = 0.0;
	camera.y = camera_zoom*cos(camera_yaw);
	camera.x = camera_zoom*sin(camera_yaw)*sin(camera_pitch);
	camera.z = camera_zoom*sin(camera_yaw)*cos(camera_pitch);
}


void camera_reset() {
	camera_yaw = -1.350875;
	camera_pitch = 0.753992;
	camera_zoom = INITIAL_ZOOM;
	camera_rotate(0, 0);
}


void camera_zoom_in(double zoom) {
	camera_zoom += zoom;
	if(camera_zoom < 0.5)
		camera_zoom = 0.5;

	camera.y = camera_zoom*cos(camera_yaw);
	camera.x = camera_zoom*sin(camera_yaw)*sin(camera_pitch);
	camera.z = camera_zoom*sin(camera_yaw)*cos(camera_pitch);
}

static void _atexit_cleanup_ogl() {
	ogl_exit(state);
}

int run_triangle () {
	state = ogl_state_new();
	
	ogl_init(state);
	atexit(_atexit_cleanup_ogl);
	glPushMatrix();
	while(!terminate) {
		camera_apply();
		draw_grid();
	//	for (i = 0; i < 2; i++)
	//		redraw_crosshair(accumulated_imu[i].gyro.x, accumulated_imu[i].gyro.y, accumulated_imu[i].gyro.z, imu_position[i].pos.x, imu_position[i].pos.y, imu_position[i].pos.z);
		//redraw_bones();
		redraw_crosshair(accumulated_imu[0].gyro.x, accumulated_imu[0].gyro.y, accumulated_imu[0].gyro.z, 0, 0, 0); 
		Vector3 zero = {0, 0, 0};
		draw_line(&zero, &gravity[0].acc, 0, 0, 255);
		draw_line(&zero, &gravity[0].gyro, 0 ,255, 0);
		draw_line(&zero, &accel_no_grav[0], 255, 0, 0);
		ogl_flip(state);
	}
	
	return 0;
}


void print_camera() {
	fprintf(stdout, "camera: %lf %lf %lf\n", camera_yaw, camera_pitch, camera_zoom);
}


void init_grid() {
	glLineWidth(8);
	create_grid(xy_grid, 0.2, 0., 0., -5., -5., 0., 0., 10.f, 0.f);
	create_grid(&xy_grid[51*2], 0., 0.2, 0., -5., -5., 0., 10.f, 0.f, 0.f);
	
	create_grid(xz_grid, 0.2, 0., 0., -5., 0., -5., 0., 0.f, 10.f);
	create_grid(&xz_grid[51*2], 0., 0., .2, -5., 0., -5., 10.f, 0.f, 0.f);

	create_grid(yz_grid, 0., 0.2, 0., 0., -5., -5., 0., 0.f, 10.f);
	create_grid(&yz_grid[51*2], 0., 0., 0.2, 0., -5., -5., 0.f, 10.f, 0.f);

	camera_reset();
}
