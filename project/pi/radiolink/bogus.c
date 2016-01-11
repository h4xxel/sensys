#include "protocol.h"

struct DecodedPacket protocol_recv_decoded_packet() {
	struct DecodedPacket packet = {
		.range = 0,
		.sen1.sensor_id = 0,
		.sen2.sensor_id = 1,
	};
	
	return packet;
}

int protocol_init() {
	return 0;
}
