#include <stdint.h>
#include <stdbool.h>
#include "imu.h"
#include "protocol.h"

static uint32_t _packet_id = 0;

static uint32_t packet_id() {
	uint32_t id = _packet_id;
	_packet_id = (_packet_id + 1) & 0xF;
	
	return id;
}

void protocol_packet_new(ProtocolPacket *packet, ImuRange range) {
	packet->id = packet_id();
	packet->samples = 0;
	packet->type = range;
	
	packet->imu0 = PROTOCOL_IMU_INVALID;
	packet->imu1 = PROTOCOL_IMU_INVALID;
}

bool protocol_packet_isempty(ProtocolPacket *packet) {
	if(packet->imu0 == PROTOCOL_IMU_INVALID && packet->imu1 == PROTOCOL_IMU_INVALID)
		return true;
	else
		return false;
}

int protocol_pack_imu(ProtocolPacket *packet, Imu *imu, int imu_number) {
	if(packet->imu0 == PROTOCOL_IMU_INVALID) {
		packet->imu0 = imu_number;
		
		packet->gyro0_x = imu->gyro.x;
		packet->gyro0_y = imu->gyro.y;
		packet->gyro0_z = imu->gyro.z;
		
		packet->accel0_x = imu->accel.x;
		packet->accel0_y = imu->accel.y;
		packet->accel0_z = imu->accel.z;
	} else if(packet->imu1 == PROTOCOL_IMU_INVALID) {
		packet->imu1 = imu_number;

		packet->gyro1_x = imu->gyro.x;
		packet->gyro1_y = imu->gyro.y;
		packet->gyro1_z = imu->gyro.z;
		
		packet->accel1_x = imu->accel.x;
		packet->accel1_y = imu->accel.y;
		packet->accel1_z = imu->accel.z;
	} else
		return -1;
	
	
	return 0;
}