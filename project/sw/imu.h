#ifndef __IMU_H__
#define __IMU_H__

#include <stdint.h>

typedef struct Sensor Sensor;
struct Sensor {
	int16_t x;
	int16_t y;
	int16_t z;
};

typedef struct Imu Imu;
struct Imu {
	Sensor gyro;
	Sensor accel;
};

typedef enum GyroReg GyroReg;
enum GyroReg {
	GYRO_REG_CHIP_ID = 0x0,
	
	GYRO_REG_RATE_X_LSB = 0x2,
	GYRO_REG_RATE_X_MSB,
	GYRO_REG_RATE_Y_LSB,
	GYRO_REG_RATE_Y_MSB,
	GYRO_REG_RATE_Z_LSB,
	GYRO_REG_RATE_Z_MSB,
	GYRO_REG_TEMP,
	
	GYRO_REG_INT_STATUS_0 = 0x9,
	GYRO_REG_INT_STATUS_1,
	GYRO_REG_INT_STATUS_2,
	GYRO_REG_INT_STATUS_3,
	
	GYRO_REG_FIFO_STATUS = 0xE,
	GYRO_REG_RANGE,
	GYRO_REG_BW,
	GYRO_REG_LPM1,
	GYRO_REG_LPM2,
	GYRO_REG_RATE_HBW,
	GYRO_REG_BGW_SOFTRESET,
	GYRO_REG_INT_EN_0,
	GYRO_REG_INT_EN_1,
	GYRO_REG_INT_MAP_0,
	GYRO_REG_INT_MAP_1,
	GYRO_REG_INT_MAP_2,
	
	GYRO_REG_INT_RST_LATCH = 0x21,
	GYRO_REG_HIGH_TH_X,
	GYRO_REG_HIGH_DUR_X,
	GYRO_REG_HIGH_TH_Y,
	GYRO_REG_HIGH_DUR_Y,
	GYRO_REG_HIGH_TH_Z,
	GYRO_REG_HIGH_DUR_Z,
	
	GYRO_REG_SOC = 0x31,
	GYRO_REG_A_FOC,
	GYRO_REG_TRIM_NVM_CTRL,
	GYRO_REG_BGW_SPI3_WDT,
	
	GYRO_REG_OFC_1 = 0x36,
	GYRO_REG_OFC_2,
	GYRO_REG_OFC_3,
	GYRO_REG_OFC_4,
	GYRO_REG_TRIM_GP0,
	GYRO_REG_TRIM_GP1,
	GYRO_REG_BIST,
	GYRO_REG_FIFO_CONFIG_0,
	GYRO_REG_FIFO_CONFIG_1,
	GYRO_REG_FIFO_DATA,
};

typedef enum AccelReg AccelReg;
enum AccelReg {
	ACCEL_REG_TEMP_L = 0x0B,
	ACCEL_REG_TEMP_H,
	
	ACCEL_REG_WHO_AM_I = 0xF,
	
	ACCEL_REG_ACT_THS = 0x1E,
	ACCEL_REG_ACT_TUS,
	ACCEL_REG_CTRL1,
	ACCEL_REG_CTRL2,
	ACCEL_REG_CTRL3,
	ACCEL_REG_CTRL4,
	ACCEL_REG_CTRL5,
	ACCEL_REG_CTRL6,
	ACCEL_REG_CTRL7,
	ACCEL_REG_STATUS,
	ACCEL_REG_OUT_X_L,
	ACCEL_REG_OUT_X_H,
	ACCEL_REG_OUT_Y_L,
	ACCEL_REG_OUT_Y_H,
	ACCEL_REG_OUT_Z_L,
	ACCEL_REG_OUT_Z_H,
	ACCEL_REG_FIFO_CTRL,
	ACCEL_REG_FIFO_SRC,
	ACCEL_REG_IG_CFG1,
	ACCEL_REG_IG_SRC1,
	ACCEL_REG_IG_THS_X1,
	ACCEL_REG_IG_THS_Y1,
	ACCEL_REG_IG_THS_Z1,
	ACCEL_REG_IG_DUR1,
	ACCEL_REG_IG_CFG2,
	ACCEL_REG_IG_SRC2,
	ACCEL_REG_IG_THS2,
	ACCEL_REG_IG_DUR2,
	ACCEL_REG_XL_REFERENCE,
	ACCEL_REG_XH_REFERENCE,
	ACCEL_REG_YL_REFERENCE,
	ACCEL_REG_YH_REFERENCE,
	ACCEL_REG_ZL_REFERENCE,
	ACCEL_REG_ZH_REFERENCE,
};

typedef enum ImuRange ImuRange;
enum ImuRange {
	IMU_RANGE_LOWEST,
	IMU_RANGE_LOW,
	IMU_RANGE_HIGH,
	IMU_RANGE_HIGHEST,
};

void imu_select(int num);
uint32_t imu_enumerate();
void imu_setup(int imu_number, ImuRange range);
void imu_sample(int imu_number, Imu *imu);

#endif
