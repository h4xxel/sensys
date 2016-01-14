#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "bone.h"
#include "gl.h"
#include "radiolink/protocol.h"
#include "radiolink/radiolink.h"
#include "vector.h"
#include "linalg.h"

#define	SAMPLERATE	(1./50)

#define G_LENGTH_THRESHOLD 0.1
#define G_ANGLE_THRESHOLD 0.9
#define G_AVERAGING 20

int serial_fd;
Vector3 gyro_data;

Vector3 velocity[6];

IMUPosition imu_position[6];
IMUVector accumulated_imu[6];
IMUVector imu_data[6];
IMUVector gravity[6];
Vector3 accel_no_grav[6];
Vector3 accel_average[6];
extern Vector3 old_imudata[6];
extern Vector3 old_imupos[6];
int average_samples[6];

double acc_scaling[4] = { 2./32767, 2./32767, 4./32767, 8./32767 };
double gyro_scaling[4] = { (125.*M_PI/180.0)/32767, (250.*M_PI/180.0)/32767, (500.*M_PI/180.0)/32767, (1000.*M_PI/180.0)/32767 };

static void set_gyro(IMUVector *accumulated, IMUVector *grav) {
	Vector3 a;
	double theta, phi;
	
	a = grav->acc;
	norm_vector(&a);
	
	theta = acos(a.x) - M_PI/2.0;
	phi = atan2(a.y, a.z);
	
	
	accumulated->gyro.x = -phi;
	accumulated->gyro.y = -theta;
}

void average_accelerometer(IMUVector *grav, Vector3 *acc, int *samples) {
	double len;
	
	len = sqrt(POW2(acc->x) + POW2(acc->y) + POW2(acc->z));
	if(fabs(len - 1.0) > G_LENGTH_THRESHOLD)
		return;
	if(dot_product(acc, &grav->gyro) < G_ANGLE_THRESHOLD)
		return;
	
	grav->acc.x = (grav->acc.x + acc->x)/2;
	grav->acc.y = (grav->acc.y + acc->y)/2;
	grav->acc.z = (grav->acc.z + acc->z)/2;
	
	(*samples)++;
}


void force_recal_gyros() {
	int i;

	for (i = 0; i < 6; i++) {
		set_gyro(&accumulated_imu[i], &imu_data[i]);
	}
}

void recal_gyros() {
	int i;

	for (i = 0; i < 6; i++) {
		if(average_samples[i] < G_AVERAGING)
			continue;
	
		fprintf(stderr, "Reset %i\n", i);
		set_gyro(&accumulated_imu[i], &gravity[i]);
		average_samples[i] = 0;
	}
}

void reset_gyros() {
	int i;
	
	for(i = 0; i < 6; i++) {
		accumulated_imu[i].gyro.x = 0.0;
		accumulated_imu[i].gyro.y = 0.0;
		accumulated_imu[i].gyro.z = 0.0;
	}
}

static Vector3 remove_gravity(Vector3 *acc, double u, double v) {
	Vector3 grav = {0.0, -1.0, 0.0};
	
	double rot[9];

	rot[0] = cos(v);
	rot[1] = -sin(v);
	rot[2] = 1;
	rot[3] = cos(u)*sin(v);
	rot[4] = cos(u)*cos(v);
	rot[5] = -sin(u);
	rot[6] = sin(u)*sin(v);
	rot[7] = sin(u)*cos(v);
 	rot[8]= cos(u);
	
	matrix33_times_vector3(rot, &grav);
	
	//Oops, let's rotate some more..
	rot[0] = 1;
	rot[1] = 0;
	rot[2] = 0;
	rot[3] = 0;
	rot[4] = 0;
	rot[5] = 1;
	rot[6] = 0;
	rot[7] = -1;
	rot[8] = 0;

	matrix33_times_vector3(rot, &grav);
		
	vector3_minus_vector3(acc, &grav);
	return grav;
}

static void process_one_imu(struct SensorData sd, int samples, int range) {
	struct IMUVector iv;
	int i = sd.sensor_id;
	
	if (sd.sensor_id > 5)
		return;
	
	iv.gyro.x = -gyro_scaling[range] * sd.gyro_x * samples * SAMPLERATE;
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
	Vector3 grav = remove_gravity(&iv.acc, accumulated_imu[i].gyro.x, accumulated_imu[i].gyro.y);
	gravity[sd.sensor_id].gyro = grav;
	average_accelerometer(&gravity[sd.sensor_id], &imu_data[sd.sensor_id].acc, &average_samples[sd.sensor_id]);
	//gravity[sd.sensor_id].acc = imu_data[sd.sensor_id].acc;
	accel_no_grav[sd.sensor_id] = iv.acc;
	//fprintf(stderr, "acc: %lf %lf %lf (%lf %lf)\n", iv.acc.x, iv.acc.y, iv.acc.z, accumulated_imu[i].gyro.x, accumulated_imu[i].gyro.y);

	velocity[i].x += accel_no_grav[i].x * SAMPLERATE;
	velocity[i].y += accel_no_grav[i].y * SAMPLERATE;
	velocity[i].z += accel_no_grav[i].z * SAMPLERATE;
	
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
					for (i = 0; i < 6; i++) {
						accumulated_imu[i].gyro.x = accumulated_imu[i].gyro.y = accumulated_imu[i].gyro.z = 0.;
						imu_position[i].pos.x = velocity[i].x = 0;
						imu_position[i].pos.y = velocity[i].y = 0;
						imu_position[i].pos.z = velocity[i].z = 0;
						old_imudata[i] = imu_position[i].pos;
						old_imupos[i] = old_imudata[i];
					}
					
					reset_bone();
					force_recal_gyros();
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
				case 'r':
					radiolink_init(26);
					break;
				case '+':
					camera_zoom_in(-0.5);
					break;
				case '-':
					camera_zoom_in(0.5);
					break;
			}
		recal_gyros();
		bone_recalculate();
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
