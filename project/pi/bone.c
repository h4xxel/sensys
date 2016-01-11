#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "vector.h"
#include "bone.h"

struct BoneRender *bonerender;
struct Bone *bone;
int bones;

extern struct IMUPosition imu_position[6];

Vector3 calculate_rotation_offsets(Vector3 angle, Vector3 pos) {
	Vector3 tmp;

	tmp.y = pos.y*cos(angle.x) - pos.z*sin(angle.x);
	tmp.z = pos.z*cos(angle.x) + pos.y*sin(angle.x);
	pos = tmp;
	
	tmp.z = pos.z*cos(angle.y) - pos.x * sin(angle.y);
	tmp.x = pos.x*cos(angle.y) - pos.z * sin(angle.y);
	pos = tmp;

	tmp.x = pos.x * cos(angle.z) - pos.y * sin(angle.z);
	tmp.y = pos.y * cos(angle.z) + pos.x * sin(angle.z);
	pos = tmp;
	return tmp;
}

void bone_recalculate() {
	int i;
	struct IMUPosition imu;

	for (i = 0; i < bones; i++) {
		if (bone[i].joint == -1)
			memset(&imu, 0, sizeof(imu));
		else
			imu = imu_position[bone[i].joint];
		bonerender[i].p1 = imu.pos;
		
	}
}


void bone_parse(char *fname) {
	FILE *fp;
	struct Bone b;

	if (!(fp = fopen(fname, "r")))
		return fprintf(stderr, "Unable to open %s\n", fname), (void) fname;
	while (!feof(fp)) {
		if (fscanf(fp, "%lf %lf: %lf %lf, %lf %lf, %lf %lf, %i %i\n", &b.length, &b.imu_pos, &b.max_angle_x, &b.min_angle_x, &b.max_angle_y, &b.min_angle_y, &b.max_angle_z, &b.min_angle_z, &b.joint, &b.imu_id) == 10) {
			bones++;
			bone = realloc(bone, sizeof(*bone) * bones);
			bone[bones - 1] = b;
			/* TODO: Process read data */
		}
	}

	bonerender = malloc(sizeof(*bonerender) * bones);

	fclose(fp);
}



