#include "protocol.h"

struct DecodedPacket protocol_recv_decoded_packet() {
	struct DecodedPacket packet = {
		.range = 0,
		.sen1.sensor_id = 0,
		.sen1.acc_x = 16384,
		.sen2.sensor_id = 1,
		.sen1.acc_y = 16384,
	};
	
	return packet;
}

int protocol_init() {
	return 0;
}

int radiolink_init(char __packet_size) {
	return 0;
}
