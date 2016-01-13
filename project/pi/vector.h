#ifndef __VECTOR_H__
#define	__VECTOR_H__

typedef struct Vector3 Vector3;
struct Vector3 {
	double x;
	double y;
	double z;
};

typedef struct Vector3f Vector3f;
struct Vector3f {
	float	x;
	float	y;
	float	z;
};

typedef struct IMUVector IMUVector;
struct IMUVector {
	Vector3 acc;
	Vector3 gyro;
};

typedef struct IMUPosition IMUPosition;
struct IMUPosition {
	Vector3 pos;
	Vector3 angle;
};


#endif
