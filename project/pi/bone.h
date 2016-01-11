#ifndef __BONE_H__
#define	__BONE_H__

#include "vector.h"

struct Bone {
	double			length;
	double			imu_pos;
	double			max_angle_x;
	double			min_angle_x;
	double			max_angle_y;
	double			min_angle_y;
	double			max_angle_z;
	double			min_angle_z;
	int			joint;
	int			imu_id;
};


struct BoneRender {
	Vector3 		p1, p2;
};


extern struct Bone *bone;
extern int bones;

void bone_parse(char *fname);

#endif