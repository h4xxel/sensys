#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "radiolink/protocol.h"
#include "vector.h"
#define	SAMPLERATE	(1./50)

int serial_fd;
Vector3 gyro_data;


IMUPosition imu_position[6];
IMUVector accumulated_imu[6];
IMUVector imu_data[6];


double acc_scaling[4] = { 2./32767, 2./32767, 4./32767, 8./32767 };
double gyro_scaling[4] = { 125./32767, 250./32767, 500./32767, 1000./32767 };


static void process_one_imu(struct SensorData sd, int samples, int range) {
	struct IMUVector iv;
	
	if (sd.sensor_id > 5)
		return;

	iv.acc.x = acc_scaling[range] * sd.acc_x * samples /** SAMPLERATE*/;
	iv.acc.y = acc_scaling[range] * sd.acc_y * samples /** SAMPLERATE*/;
	iv.acc.z = acc_scaling[range] * sd.acc_z * samples /** SAMPLERATE*/;
	iv.gyro.x = gyro_scaling[range] * sd.gyro_x * samples * SAMPLERATE;
	iv.gyro.y = gyro_scaling[range] * sd.gyro_y * samples * SAMPLERATE;
	iv.gyro.z = gyro_scaling[range] * sd.gyro_z * samples * SAMPLERATE;

	imu_data[sd.sensor_id] = iv;
	return;
}


static void __add_imu() {
	int i;

	for (i = 0; i < 6; i++) {
		accumulated_imu[i].acc.x = imu_data[i].acc.x;
		accumulated_imu[i].acc.y = imu_data[i].acc.y;
		accumulated_imu[i].acc.z = imu_data[i].acc.z;
		accumulated_imu[i].gyro.x += imu_data[i].gyro.x;
		accumulated_imu[i].gyro.y += imu_data[i].gyro.y;
		accumulated_imu[i].gyro.z += imu_data[i].gyro.z;
	}
}


static void process_imu() {
	struct DecodedPacket dp;
	int ch, i;

	for (;;) {
		dp = protocol_recv_decoded_packet();
		process_one_imu(dp.sen1, dp.samples, dp.range);
		process_one_imu(dp.sen2, dp.samples, dp.range);
		__add_imu();
		while ((ch = fgetc(stdin)) >= ' ' || ch == '\n')
			if (ch == '\n')
				for (i = 0; i < 6; i++)
					accumulated_imu[i].gyro.x = accumulated_imu[i].gyro.y = accumulated_imu[i].gyro.z = 0.;
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
