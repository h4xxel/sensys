#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>
#include <stdbool.h>

#define PROTOCOL_IMU_INVALID 0x7
#define PROTOCOL_PACKET_SIZE 26

typedef struct ProtocolPacket ProtocolPacket;

struct ProtocolPacket {
	uint16_t id : 4;
	uint16_t samples : 4;
	uint16_t type : 2;
	uint16_t imu0 : 3;
	uint16_t imu1 : 3;
	
	uint16_t gyro0_x : 16;
	uint16_t gyro0_y : 16;
	uint16_t gyro0_z : 16;
	uint16_t accel0_x : 16;
	uint16_t accel0_y : 16;
	uint16_t accel0_z : 16;
	
	uint16_t gyro1_x : 16;
	uint16_t gyro1_y : 16;
	uint16_t gyro1_z : 16;
	uint16_t accel1_x : 16;
	uint16_t accel1_y : 16;
	uint16_t accel1_z : 16;
};

void protocol_packet_new(ProtocolPacket *packet, ImuRange range);
bool protocol_packet_isempty(ProtocolPacket *packet);
int protocol_pack_imu(ProtocolPacket *packet, Imu *imu, int imu_number);

#endif
