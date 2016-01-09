#ifndef __VECTOR_H__
#define	__VECTOR_H__

typedef struct Vector3 Vector3;
struct Vector3 {
	double x;
	double y;
	double z;
};

typedef struct IMUVector IMUVector;
struct IMUVector {
	struct Vector3 acc;
	struct Vector3 gyro;
};


#endif
