#ifndef __LINALG_H__
#define __LINALG_H__

#include "vector.h"

#define POW2(x) ((x)*(x))

void norm_vector(Vector3 *vec);
double dot_product(Vector3 *a, Vector3 *b);
Vector3 cross_product(Vector3 *a, Vector3 *b);
void scalar_dot_matrix33(double scalar, double matrix[9]);
void matrix33_times_vector3(double matrix[9], Vector3 *vec);
void vector3_minus_vector3(Vector3 *a, Vector3 *b);
double vec3_len(Vector3 *a);
void matrix33_plus_matrix33(double a[9], double b[9]);
void matrix33_times_matrix33(double a[9], double b[9], double res[9]);
double matrix33_determinant(double a[9]);
void matrix33_invert(double a[9], double res[9]);

#endif
