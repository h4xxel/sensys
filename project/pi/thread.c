#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "vector.h"

int serial_fd;
Vector3 gyro_data;

void *sensor_data_worker(void *arg) {
	Vector3 vec;
	for (;;) {
		get_gyro(serial_fd, &vec);
		gyro_data.x += vec.x;
		gyro_data.y += vec.y;
		gyro_data.z += vec.z;
	}
}


void launch_worker(int fd) {
	pthread_t thr;

	serial_fd = fd;

	pthread_create(&thr, NULL, sensor_data_worker, NULL);
}
