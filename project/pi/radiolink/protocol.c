#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include "radiolink.h"
#include "protocol.h"

#include <wiringPi.h>
#include <wiringPiSPI.h>



static void __print_sensor(struct SensorData sd) {
	fprintf(stderr,"Gyro %i:	%i, %i, %i\n", sd.sensor_id, sd.gyro_x, sd.gyro_y, sd.gyro_z);
	fprintf(stderr,"Acc %i:		%i, %i, %i\n", sd.sensor_id, sd.acc_x, sd.acc_y, sd.acc_z);
	return;
}


static void __print_packet(struct DecodedPacket dp) {
	fprintf(stderr, "Sequence %i, samples: %i, range: %i\n", dp.sequence, dp.samples, dp.range);
	
	if (dp.sen1.sensor_id < 7)
		__print_sensor(dp.sen1);

	if (dp.sen2.sensor_id < 7)
		__print_sensor(dp.sen2);
}


static void __fill_in_sensordata(struct SensorData *sd, int16_t *data) {
	sd->gyro_x = data[0];
	sd->gyro_y = data[1];
	sd->gyro_z = data[2];
	sd->acc_x = data[3];
	sd->acc_y = data[4];
	sd->acc_z = data[5];
}


static struct DecodedPacket __decode_packet(uint8_t *data) {
	struct DecodedPacket dp;
	int16_t *sw = (void *) data;

	dp.sequence = data[0] & 0xF;
	dp.samples = (data[0] & 0xF0) >> 4;
	dp.samples += 1;
	dp.range = data[1] & 0x3;
	dp.sen1.sensor_id = (data[1] >> 2) & 0x7;
	dp.sen2.sensor_id = ((data[1] >> 2) & 0x38) >> 3;
	__fill_in_sensordata(&dp.sen1, &sw[1]);
	__fill_in_sensordata(&dp.sen2, &sw[7]);
	return dp;
}


struct DecodedPacket protocol_recv_decoded_packet() {
	uint8_t data[26];

	radiolink_recv(26, data);
	printf("got data\n");
	return __decode_packet(data);
}


int protocol_init() {
	struct DecodedPacket dp;
	uint8_t data[32];
	time_t t = 0;
	int cnt = 0;
	wiringPiSetup();
	wiringPiSPISetup(0, 8000000);
	sleep(1);
	
	radiolink_init(26);

	#if 0
	for (;;cnt++) {
		if (t != time(NULL))
			fprintf(stderr, "%i packets this second\n", cnt), cnt = 0, t = time(NULL);
		radiolink_recv(26, data);
//		printf("got data: %s\n", data);
		printf("Time is now %i\n", timer_now());
		dp = __decode_packet(data);
		__print_packet(dp);
	}
	#endif

	return 0;
}


uint8_t spi_send_recv(uint8_t byte) {
	wiringPiSPIDataRW(0, &byte, 1);
	return byte;
}
