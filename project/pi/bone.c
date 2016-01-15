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
Vector3 accel_no_grav[6];
Vector3 old_imudata[6];
Vector3 old_imupos[6];

#define	GLOBAL_POSITION		0


void print_bone_angle() {
	int i;
	for (i = 0; i < bones; i++) {
		Vector3 parent = { };
		double x, y, z;
		if (bone[i].joint >= 0)
			parent = accumulated_imu[bone[bone[i].joint].imu_id].gyro;
		x = accumulated_imu[bone[i].imu_id].gyro.x - parent.x;
		y = accumulated_imu[bone[i].imu_id].gyro.y - parent.y;
		z = accumulated_imu[bone[i].imu_id].gyro.z - parent.z;
		fprintf(stderr, "%i: %lf, %lf, %lf\n", i, x/M_PI*180., y/M_PI*180., z/M_PI*180);
	}
}


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


void reset_bone() {
	bonerender = memset(bonerender, 0, sizeof(*bonerender) * bones);
	global_pos.x = global_pos.y = global_pos.z = 0;
}


/* TODO: Allow for bones without an imu */
/* ACHTUNG JOMMPAKOD */
void bone_recalculate() {
	int i;
	struct IMUPosition imu, this, tmp;
	Vector3 relative_diff[bones];
	Vector3 calc_pos[bones], calc_off[bones];
	for (i = 0; i < 6; i++)
		imu_position[i].angle = accumulated_imu[i].gyro;
	
	Vector3 relative_sum = { }, absolute_sum = {};
	for (i = 0; i < bones; i++) {
		if (bone[i].joint == -1)
			imu.pos.x = 0, imu.pos.y = 0, imu.pos.z = 0;
		else
			imu = imu_position[bone[bone[i].joint].imu_id], imu.pos = vec3f_to_vec3(bonerender[bone[i].joint].p2);
		this = imu_position[bone[i].imu_id];
		bonerender[i].p1 = vec3_to_vec3f(imu.pos);
		//tmp.angle.x = clamp_angle(this.angle.x, imu.angle.x, bone[i].max_angle_x, bone[i].min_angle_x);
		//tmp.angle.y = clamp_angle(this.angle.y, imu.angle.y, bone[i].max_angle_y, bone[i].min_angle_y);
		//tmp.angle.z = clamp_angle(this.angle.z, imu.angle.z, bone[i].max_angle_z, bone[i].min_angle_z);
		tmp.angle = this.angle;
		//imu_position[bone[i].imu_id].angle = tmp.angle;
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

		relative_sum.x += calc_pos[i].x - old_imupos[i].x;
		relative_sum.y += calc_pos[i].y - old_imupos[i].y;
		relative_sum.z += calc_pos[i].z - old_imupos[i].z;

		absolute_sum.x += imu_position[i].pos.x - old_imudata[i].x;
		absolute_sum.y += imu_position[i].pos.y - old_imudata[i].y;
		absolute_sum.z += imu_position[i].pos.z - old_imudata[i].z;

		old_imupos[i] = calc_pos[i];
		old_imudata[i] = imu_position[i].pos;
		calc_off[i].x = calc_pos[i].x + global_pos.x - this.pos.x;
		calc_off[i].y = calc_pos[i].y + global_pos.y - this.pos.y;
		calc_off[i].z = calc_pos[i].z + global_pos.z - this.pos.z;
	}
		
	#if GLOBAL_POSITION
	Vector3 diff;

	diff.x = absolute_sum.x - relative_sum.x;
	diff.y = absolute_sum.y - relative_sum.y;
	diff.z = absolute_sum.z - relative_sum.z;
	fprintf(stderr, "rel %lf %lf %lf\n", relative_sum.x, relative_sum.y, relative_sum.z);
	fprintf(stderr, "abs %lf %lf %lf\n", absolute_sum.x, absolute_sum.y, absolute_sum.z);

	global_pos.x += diff.x /2;
	global_pos.y += diff.y/2;
	global_pos.z += diff.z/2;
	
	#endif
	for (i = 0; i < bones; i++) {
		imu_position[bone[i].imu_id].pos.x = calc_pos[i].x + global_pos.x;
		imu_position[bone[i].imu_id].pos.y = calc_pos[i].y + global_pos.y;
		imu_position[bone[i].imu_id].pos.z = calc_pos[i].z + global_pos.z;
	}

	//fprintf(stderr, "Global pos is now %lf, %lf, %lf\n", global_pos.x, global_pos.y, global_pos.z);

	//fprintf(stderr, "Calculated variance: %lf %lf %lf\n", maxx - minx, maxy - miny, maxz - minz);
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



