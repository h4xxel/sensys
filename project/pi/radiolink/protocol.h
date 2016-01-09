#ifndef __PROTOCOL_H__
#define	__PROTOCOL_H__

struct SensorData {
	int			sensor_id;
	int			gyro_x;
	int			gyro_y;
	int			gyro_z;
	int			acc_x;
	int			acc_y;
	int			acc_z;
};


struct DecodedPacket {
	int			sequence;
	int			samples;
	int			range;
	struct SensorData	sen1;
	struct SensorData	sen2;

};

int protocol_init();
struct DecodedPacket protocol_recv_decoded_packet();

#endif
