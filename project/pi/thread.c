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
IMUVector gravity[6];
Vector3 accel_no_grav[6];

double acc_scaling[4] = { 2./32767, 2./32767, 4./32767, 8./32767 };
double gyro_scaling[4] = { (125.*M_PI/180.0)/32767, (250.*M_PI/180.0)/32767, (500.*M_PI/180.0)/32767, (1000.*M_PI/180.0)/32767 };

/*
static void calibrate_gyro(IMUVector *accumulated, IMUVector *raw) {
	double len = sqrt(POW2(imu->pos.x) + POW2(imu->pos.y) + POW2(imu->pos.z));
	
	if(fabs(len - 1.0) < G_THRESHOLD) {
		
	}
}
*/

static void norm_vector(Vector3 *vec) {
	double len = sqrt(POW2(vec->x) + POW2(vec->y) + POW2(vec->z));
	
	vec->x /= len;
	vec->y /= len;
	vec->z /= len;
}

static double dot_product(Vector3 *a, Vector3 *b) {
	return a->x*b->x + a->y*b->y + a->z*b->z;
}

static Vector3 cross_product(Vector3 *a, Vector3 *b) {
	double u1, u2, u3, v1, v2, v3;
	Vector3 cross;
	
	u1 = a->x;
	u2 = a->y;
	u3 = a->z;
	
	v1 = b->x;
	v2 = b->y;
	v3 = b->z;
	
	cross.x = u2*v3 - u3*v2;
	cross.y = u3*v1 - u1*v3;
	cross.z = u1*v2 - u2*v1;
	
	return cross;
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


static double vec3_len(Vector3 *a) {
	return sqrt(POW2(a->x) + POW2(a->y) + POW2(a->z));
}

static void matrix33_plus_matrix33(double a[9], double b[9]) {
	int i;
	for(i = 0; i < 9; i++)
		a[i] += b[i];
}

static void matrix33_times_matrix33(double a[9], double b[9], double res[9]) {
	int i, j, k;
	double sum;
	
	for(i = 0; i < 3; i++) {
		for(j = 0; j < 3; j++) {
			sum = 0;
			for(k = 0; k < 3; k++)
				sum += a[i*3 + k] * b[k*3 + j];
			res[i*3 + j] = sum;
		}
	}
}

static void set_gyro(IMUVector *accumulated, IMUVector *raw) {
	Vector3 a;
	/*Vector3 b = {0.0, 0., 1.0};
	
	a = raw->acc;
	norm_vector(&a);
	
	//Thanks to http://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d/476311#476311
	Vector3 cross = cross_product(&a, &b);
	double s = vec3_len(&cross);
	double c = dot_product(&a, &b);
	double rot[9], v[9], v2[9];
	Vector3 rot_vector;
	
	rot[0] = 1;
	rot[1] = 0;
	rot[2] = 0;
	
	rot[3] = 0;
	rot[4] = 1;
	rot[5] = 0;
	
	rot[6] = 0;
	rot[7] = 0;
	rot[8] = 1;
	
	v[0] = 0;
	v[1] = -cross.z;
	v[2] = cross.y;
	
	v[3] = cross.z;
	v[4] = 0;
	v[5] = -cross.x;
	
	v[6] = -cross.y;
	v[7] = cross.x;
	v[8] = 0;
	
	matrix33_times_matrix33(v, v, v2);
	scalar_dot_matrix33((1 - c)/(s*s), v2);
	
	matrix33_plus_matrix33(rot, v);
	matrix33_plus_matrix33(rot, v2);
	
	//Thanks to: http://nghiaho.com/?page_id=846
	rot_vector.x = (atan2(rot[7], rot[8]));
	rot_vector.y = -(atan2(-rot[6], sqrt(POW2(rot[7]) + POW2(rot[8]))));
	rot_vector.z = (atan2(rot[3], rot[0]));
	
	accumulated->gyro = rot_vector;*/
	
	a = raw->acc;
	norm_vector(&a);
	
	double theta, phi;
	
	phi = acos(a.x) + M_PI/2;
	theta = atan2(a.y, a.z);
	
	
	accumulated->gyro.x = phi;
	accumulated->gyro.y = theta + M_PI;
}


void recal_gyro() {
	int i;

	for (i = 0; i < 6; i++) {
		set_gyro(&accumulated_imu[i], &imu_data[i]);
	}
	return;
	
}


static Vector3 remove_gravity(Vector3 *acc, double u, double v) {
	Vector3 grav = {0.0, -1.0, 0.0};
	
	double a, b, c, d, e, f, g, h, i;
	double det;
	
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
	
	matrix33_times_vector3(rot_inv, &grav);
	
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

	matrix33_times_vector3(rot_inv, &grav);
	
	vector3_minus_vector3(acc, &grav);
	return grav;
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
	Vector3 grav = remove_gravity(&iv.acc, -accumulated_imu[i].gyro.x, -accumulated_imu[i].gyro.y);
	gravity[sd.sensor_id].gyro = grav;
	gravity[sd.sensor_id].acc = imu_data[sd.sensor_id].acc;
	accel_no_grav[sd.sensor_id] = iv.acc;
	//fprintf(stderr, "acc: %lf %lf %lf (%lf %lf)\n", iv.acc.x, iv.acc.y, iv.acc.z, accumulated_imu[i].gyro.x, accumulated_imu[i].gyro.y);

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
	int ch, i, j;


	dp = protocol_recv_decoded_packet();
	process_one_imu(dp.sen1, dp.samples, dp.range);
	process_one_imu(dp.sen2, dp.samples, dp.range);

	for(i = 0; i < 6; i++) {
		set_gyro(&accumulated_imu[i], &imu_data[i]);
		gravity[i].acc = imu_data[i].acc;
		velocity[i].x = 0.0;
		velocity[i].y = 0.0;
		velocity[i].z = 0.0;
		
		imu_position[i].pos.x = 0;
		imu_position[i].pos.y = 0;
		imu_position[i].pos.z = 0;
		
		accumulated_imu[i].acc.x = 0;
		accumulated_imu[i].acc.y = 0;
		accumulated_imu[i].acc.z = 0;
	}
//	for(;;);

	for (j = 0;; j++) {
		dp = protocol_recv_decoded_packet();
		process_one_imu(dp.sen1, dp.samples, dp.range);
		process_one_imu(dp.sen2, dp.samples, dp.range);
		while ((ch = getchar()) >= ' ' || ch == '\n')
			switch(ch) {
				case '\n':
					for (i = 0; i < 6; i++)
						accumulated_imu[i].gyro.x = accumulated_imu[i].gyro.y = accumulated_imu[i].gyro.z = 0.;
					recal_gyro();
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
				case 'p':
					print_camera();
					break;
				case 'z':
					camera_reset();
					break;
				case '+':
					camera_zoom_in(-0.5);
					break;
				case '-':
					camera_zoom_in(0.5);
					break;
			}
	}
}


void *sensor_data_worker(void *arg) {
	process_imu();
	
	return NULL;
}


void launch_worker() {
	pthread_t thr;
	pthread_create(&thr, NULL, sensor_data_worker, NULL);
}
