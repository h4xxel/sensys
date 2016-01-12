#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "gl.h"
#include "radiolink/protocol.h"
#include "vector.h"
#define	SAMPLERATE	(1./50)

#define POW2(x) ((x)*(x))

#define G_THRESHOLD 0.1

int serial_fd;
Vector3 gyro_data;

Vector3 velocity[6];

IMUPosition imu_position[6];
IMUVector accumulated_imu[6];
IMUVector imu_data[6];


double acc_scaling[4] = { 2./32767, 2./32767, 4./32767, 8./32767 };
double gyro_scaling[4] = { (125.*M_PI/180.0)/32767, (250.*M_PI/180.0)/32767, (500.*M_PI/180.0)/32767, (1000.*M_PI/180.0)/32767 };


static void calibrate_gyro(IMUPosition *imu) {
	double len = sqrt(POW2(imu->pos.x) + POW2(imu->pos.y) + POW2(imu->pos.z));
	
	if(fabs(len - 1.0) < G_THRESHOLD) {
		
	}
}

static void scalar_dot_matrix33(double scalar, double matrix[9]) {
	int i;
	
	for(i = 0; i < 9; i++)
		matrix[i] *= scalar;
}

static void matrix33_times_vector3(double matrix[9], Vector3 *vec) {
	double x, y, z;
	
	x = vec->x;
	y = vec->y;
	z = vec->z;
	
	vec->x = (matrix[0]*x + matrix[1]*y + matrix[2]*z);
	vec->y = (matrix[3]*x + matrix[4]*y + matrix[5]*z);
	vec->z = (matrix[6]*x + matrix[7]*y + matrix[8]*z);
}

static void vector3_minus_vector3(Vector3 *a, Vector3 *b) {
	a->x -= b->x;
	a->y -= b->y;
	a->z -= b->z;
}

static void remove_gravity(Vector3 *acc, double u, double v) {
	Vector3 gravity = {0.0, -1.0, 0.0};
	
	double a, b, c, d, e, f, g, h, i;
	double det, res;
	
	double rot_inv[9];
	
	a = cos(v);
	b = -sin(v);
	c = 1;
	d = cos(u)*sin(v);
	e = cos(u)*cos(v);
	f = -sin(u);
	g = sin(u)*sin(v);
	h = sin(u)*cos(v);
	i = cos(u);
	
	det = a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g;
	
	rot_inv[0] = e*i - f*h;
	rot_inv[1] = c*h - b*i;
	rot_inv[2] = b*f - c*e;
	
	rot_inv[3] = f*g - d*i;
	rot_inv[4] = a*i - c*g;
	rot_inv[5] = c*d - a*f;
	
	rot_inv[6] = d*h - e*g;
	rot_inv[7] = b*g - a*h;
	rot_inv[8] = a*e - b*d;
	
	scalar_dot_matrix33(1.0/det, rot_inv);
	
	matrix33_times_vector3(rot_inv, &gravity);
	
	//Oops, let's rotate some more..
	rot_inv[0] = 1;
	rot_inv[1] = 0;
	rot_inv[2] = 0;
	rot_inv[3] = 0;
	rot_inv[4] = 0;
	rot_inv[5] = 1;
	rot_inv[6] = 0;
	rot_inv[7] = -1;
	rot_inv[8] = 0;
	
	matrix33_times_vector3(rot_inv, &gravity);
	
	vector3_minus_vector3(acc, &gravity);
}

static void process_one_imu(struct SensorData sd, int samples, int range) {
	struct IMUVector iv;
	int i = sd.sensor_id;
	
	if (sd.sensor_id > 5)
		return;
	
	iv.gyro.x = gyro_scaling[range] * sd.gyro_x * samples * SAMPLERATE;
	iv.gyro.y = gyro_scaling[range] * sd.gyro_y * samples * SAMPLERATE;
	iv.gyro.z = gyro_scaling[range] * sd.gyro_z * samples * SAMPLERATE;

	imu_data[sd.sensor_id].gyro = iv.gyro;
	
	accumulated_imu[i].gyro.x += imu_data[i].gyro.x;
	accumulated_imu[i].gyro.y += imu_data[i].gyro.y;
	accumulated_imu[i].gyro.z += imu_data[i].gyro.z;
	
	iv.acc.x = acc_scaling[range] * sd.acc_x * samples /** SAMPLERATE*/;
	iv.acc.y = acc_scaling[range] * sd.acc_y * samples /** SAMPLERATE*/;
	iv.acc.z = acc_scaling[range] * sd.acc_z * samples /** SAMPLERATE*/;
	imu_data[i].acc = iv.acc;
	
	//TODO: correct angles
	remove_gravity(&iv.acc, accumulated_imu[i].gyro.x, -accumulated_imu[i].gyro.y);
	fprintf(stderr, "acc: %lf %lf %lf (%lf %lf)\n", iv.acc.x, iv.acc.y, iv.acc.z, accumulated_imu[i].gyro.x, accumulated_imu[i].gyro.y);

	velocity[i].x += iv.acc.x * SAMPLERATE;
	velocity[i].y += iv.acc.y * SAMPLERATE;
	velocity[i].z += iv.acc.z * SAMPLERATE;
	
/*	accumulated_imu[i].acc.x += velocity[i].x * SAMPLERATE;
	accumulated_imu[i].acc.y += velocity[i].y * SAMPLERATE;
	accumulated_imu[i].acc.z += velocity[i].z * SAMPLERATE;*/
	imu_position[i].pos.x += velocity[i].x * SAMPLERATE;
	imu_position[i].pos.y += velocity[i].y * SAMPLERATE;
	imu_position[i].pos.z += velocity[i].z * SAMPLERATE;
	
	return;
}




static void process_imu() {
	struct DecodedPacket dp;
	int ch, i;

	for (;;) {
		dp = protocol_recv_decoded_packet();
		process_one_imu(dp.sen1, dp.samples, dp.range);
		process_one_imu(dp.sen2, dp.samples, dp.range);
		while ((ch = getchar()) >= ' ' || ch == '\n')
			switch(ch) {
				case '\n':
					for (i = 0; i < 6; i++)
						accumulated_imu[i].gyro.x = accumulated_imu[i].gyro.y = accumulated_imu[i].gyro.z = 0.;
					break;
				
				case 'w':
					camera_rotate(0.0, M_PI/100.0);
					break;
				case 'a':
					camera_rotate(2.0*M_PI/100.0, 0.0);
					break;
				case 's':
					camera_rotate(0.0, -M_PI/100.0);
					break;
				case 'd':
					camera_rotate(-2.0*M_PI/100.0, 0.0);
					break;
			}
	}
}


void *sensor_data_worker(void *arg) {
	Vector3 vec;

	process_imu();

	#if 0
	for (;;) {
		get_gyro(serial_fd, &vec);
		gyro_data.x += (vec.x);
		gyro_data.y += (-vec.z);
		gyro_data.z += (vec.y);
	}
	#endif
}


void launch_worker(int fd) {
	pthread_t thr;

	serial_fd = fd;

	pthread_create(&thr, NULL, sensor_data_worker, NULL);
}
