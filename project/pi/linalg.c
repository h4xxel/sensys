#include <math.h>
#include "vector.h"
#include "linalg.h"

void norm_vector(Vector3 *vec) {
	double len = sqrt(POW2(vec->x) + POW2(vec->y) + POW2(vec->z));
	
	vec->x /= len;
	vec->y /= len;
	vec->z /= len;
}

double dot_product(Vector3 *a, Vector3 *b) {
	return a->x*b->x + a->y*b->y + a->z*b->z;
}

Vector3 cross_product(Vector3 *a, Vector3 *b) {
	double u1, u2, u3, v1, v2, v3;
	Vector3 cross;
	
	u1 = a->x;
	u2 = a->y;
	u3 = a->z;
	
	v1 = b->x;
	v2 = b->y;
	v3 = b->z;
	
	cross.x = u2*v3 - u3*v2;
	cross.y = u3*v1 - u1*v3;
	cross.z = u1*v2 - u2*v1;
	
	return cross;
}

void scalar_dot_matrix33(double scalar, double matrix[9]) {
	int i;
	
	for(i = 0; i < 9; i++)
		matrix[i] *= scalar;
}

void matrix33_times_vector3(double matrix[9], Vector3 *vec) {
	double x, y, z;
	
	x = vec->x;
	y = vec->y;
	z = vec->z;
	
	vec->x = (matrix[0]*x + matrix[1]*y + matrix[2]*z);
	vec->y = (matrix[3]*x + matrix[4]*y + matrix[5]*z);
	vec->z = (matrix[6]*x + matrix[7]*y + matrix[8]*z);
}

void vector3_minus_vector3(Vector3 *a, Vector3 *b) {
	a->x -= b->x;
	a->y -= b->y;
	a->z -= b->z;
}


double vec3_len(Vector3 *a) {
	return sqrt(POW2(a->x) + POW2(a->y) + POW2(a->z));
}

void matrix33_plus_matrix33(double a[9], double b[9]) {
	int i;
	for(i = 0; i < 9; i++)
		a[i] += b[i];
}

void matrix33_times_matrix33(double a[9], double b[9], double res[9]) {
	int i, j, k;
	double sum;
	
	for(i = 0; i < 3; i++) {
		for(j = 0; j < 3; j++) {
			sum = 0;
			for(k = 0; k < 3; k++)
				sum += a[i*3 + k] * b[k*3 + j];
			res[i*3 + j] = sum;
		}
	}
}

double matrix33_determinant(double a[9]) {
	return a[0]*a[4]*a[8] - a[0]*a[5]*a[7] - a[1]*a[3]*a[8] + a[1]*a[5]*a[6] + a[2]*a[3]*a[7] - a[2]*a[4]*a[6];
}

void matrix33_invert(double a[9], double res[9]) {
	double det;
	
	res[0] = a[4]*a[8] - a[5]*a[7];
	res[1] = a[2]*a[7] - a[1]*a[8];
	res[2] = a[1]*a[5] - a[2]*a[4];
	
	res[3] = a[5]*a[6] - a[3]*a[8];
	res[4] = a[0]*a[8] - a[2]*a[6];
	res[5] = a[2]*a[3] - a[0]*a[5];
	
	res[6] = a[3]*a[7] - a[4]*a[6];
	res[7] = a[1]*a[6] - a[0]*a[7];
	res[8] = a[0]*a[4] - a[1]*a[3];
	
	det = matrix33_determinant(a);
	
	scalar_dot_matrix33(1.0/det, res);
}
