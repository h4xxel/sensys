#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "vector.h"
#include "bone.h"

struct BoneRender *bonerender;
struct Bone *bone;
int bones;

Vector3 global_pos;

extern struct IMUPosition imu_position[6];
extern struct IMUVector accumulated_imu[6];


Vector3 vec3f_to_vec3(Vector3f v3f) {
	Vector3 v3;

	v3.x = v3f.x;
	v3.y = v3f.y;
	v3.z = v3f.z;
	return v3;
}

Vector3f vec3_to_vec3f(Vector3 v3) {
	Vector3f v3f;

	v3f.x = v3.x;
	v3f.y = v3.y;
	v3f.z = v3.z;
	return v3f;
}

Vector3 calculate_rotation_offsets(Vector3 angle, Vector3 pos) {
	Vector3 tmp;

	tmp = pos;
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


void print_vec3f(Vector3f v3f) {
	fprintf(stderr, "%f %f %f\n", v3f.x, v3f.y, v3f.z);
}


double clamp_angle(double angle, double parent_angle, double max, double min) {
	double diff;

	diff = angle - parent_angle;
	
	if (diff > max)
		return angle + max;
	if (diff < min)
		return angle - min;
	return angle;
}


/* TODO: Allow for bones without an imu */
/* ACHTUNG JOMMPAKOD */
void bone_recalculate() {
	int i;
	struct IMUPosition imu, this, tmp;
	Vector3 calc_pos[bones], calc_off[bones];
	for (i = 0; i < 6; i++)
		imu_position[i].angle = accumulated_imu[i].gyro;

	for (i = 0; i < bones; i++) {
		if (bone[i].joint == -1)
			imu.pos = global_pos;
		else
			imu = imu_position[bone[bone[i].joint].imu_id], imu.pos = vec3f_to_vec3(bonerender[bone[i].joint].p2);
		this = imu_position[bone[i].imu_id];
		bonerender[i].p1 = vec3_to_vec3f(imu.pos);
		tmp.angle.x = clamp_angle(this.angle.x, imu.angle.x, bone[i].max_angle_x, bone[i].min_angle_x);
		tmp.angle.y = clamp_angle(this.angle.y, imu.angle.y, bone[i].max_angle_y, bone[i].min_angle_y);
		tmp.angle.z = clamp_angle(this.angle.z, imu.angle.z, bone[i].max_angle_z, bone[i].min_angle_z);
		imu_position[bone[i].imu_id].angle = tmp.angle;
		tmp.pos.x = bone[i].length;
		tmp.pos.y = 0.;
		tmp.pos.z = 0.;
		tmp.pos = calculate_rotation_offsets(tmp.angle, tmp.pos);
		tmp.pos.x += bonerender[i].p1.x;
		tmp.pos.y += bonerender[i].p1.y;
		tmp.pos.z += bonerender[i].p1.z;
		bonerender[i].p2 = vec3_to_vec3f(tmp.pos);
		tmp.pos.x = bone[i].imu_pos;
		tmp.pos.y = tmp.pos.z = 0.;
		calc_pos[i] = calculate_rotation_offsets(tmp.angle, tmp.pos);
		calc_pos[i].x += bonerender[i].p1.x;
		calc_pos[i].y += bonerender[i].p1.y;
		calc_pos[i].z += bonerender[i].p1.z;
		calc_off[i].x = calc_pos[i].x - this.pos.x;
		calc_off[i].y = calc_pos[i].y - this.pos.y;
		calc_off[i].z = calc_pos[i].z - this.pos.z;
	}

	double minx, maxx, miny, maxy, minz, maxz;
	minx = miny = minz = HUGE_VAL;
	maxx = maxy = maxz = -HUGE_VAL;
	
	for (i = 0; i < bones; i++) {
		minx = (minx > calc_off[i].x) ? calc_off[i].x : minx;
		miny = (miny > calc_off[i].y) ? calc_off[i].y : miny;
		minz = (minz > calc_off[i].z) ? calc_off[i].z : minz;
		maxx = (maxx < calc_off[i].x) ? calc_off[i].x : maxx;
		maxy = (maxy < calc_off[i].y) ? calc_off[i].y : maxy;
		maxz = (maxz < calc_off[i].z) ? calc_off[i].z : maxz;
	}

	double avgx, avgy, avgz;
	/* TODO: Better averageing function (weighted?) */
	avgx = (maxx - minx) / 2 + minx;
	avgy = (maxy - miny) / 2 + miny;
	avgz = (maxz - minz) / 2 + minz;

	for (i = 0; i < bones; i++) {
		imu_position[bone[i].imu_id].pos.x = calc_pos[i].x + avgx;
		imu_position[bone[i].imu_id].pos.y = calc_pos[i].y + avgy;
		imu_position[bone[i].imu_id].pos.z = calc_pos[i].z + avgz;
	}

	global_pos.x += avgx;
	global_pos.y += avgy;
	global_pos.z += avgz;

	fprintf(stderr, "Calculated variance: %lf %lf %lf\n", maxx - minx, maxy - miny, maxz - minz);
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



