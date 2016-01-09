#include <stdint.h>
#include "system/LPC11xx.h"
#include "util.h"
#include "i2c.h"
#include "main.h"
#include "imu.h"

void imu_select(int num) {
	LPC_GPIO0->MASKED_ACCESS[0xC8] = 0xFFF;
	LPC_GPIO2->MASKED_ACCESS[0x202] = 0xFFF;
	LPC_GPIO3->MASKED_ACCESS[0x20] = 0xFFF;
	
	switch(num) {
		case 0:
			LPC_GPIO0->MASKED_ACCESS[0x8] = 0x0;
			break;
		case 1:
			LPC_GPIO3->MASKED_ACCESS[0x20] = 0x0;
			break;
		case 2:
			LPC_GPIO0->MASKED_ACCESS[0x40] = 0x0;
			break;
		case 3:
			LPC_GPIO0->MASKED_ACCESS[0x80] = 0x0;
			break;
		case 4:
			LPC_GPIO2->MASKED_ACCESS[0x200] = 0x0;
			break;
		case 5:
			LPC_GPIO2->MASKED_ACCESS[0x2] = 0x0;
			break;
	}
	util_delay(10);
}

uint32_t imu_enumerate() {
	int i;
	uint32_t ret = 0;
	
	for(i = 5; i >= 0; i--) {
		ret <<= 1;
		imu_select(i);
		if(i2c_read_reg(ACCEL_ADDR, 0xF) != 0x41)
			continue;
		if(i2c_read_reg(GYRO_ADDR, 0x0) != 0xF)
			continue;
		
		ret |= 1;
	}
	
	return ret;
}

void imu_setup(int imu_number, ImuRange range) {
	imu_select(imu_number);
	
	i2c_write_reg(ACCEL_ADDR, ACCEL_REG_CTRL1, 0x6F);
	
	switch(range) {
		case IMU_RANGE_LOWEST:
			i2c_write_reg(GYRO_ADDR, GYRO_REG_RANGE, 0x84);
			i2c_write_reg(ACCEL_ADDR, ACCEL_REG_CTRL4, 0x00);
			break;
		case IMU_RANGE_LOW:
			i2c_write_reg(GYRO_ADDR, GYRO_REG_RANGE, 0x83);
			i2c_write_reg(ACCEL_ADDR, ACCEL_REG_CTRL4, 0x00);
			break;
		case IMU_RANGE_HIGH:
			i2c_write_reg(GYRO_ADDR, GYRO_REG_RANGE, 0x82);
			i2c_write_reg(ACCEL_ADDR, ACCEL_REG_CTRL4, 0x20);
			break;
		case IMU_RANGE_HIGHEST:
			i2c_write_reg(GYRO_ADDR, GYRO_REG_RANGE, 0x81);
			i2c_write_reg(ACCEL_ADDR, ACCEL_REG_CTRL4, 0x30);
			break;
	};
}

void imu_sample(int imu_number, Imu *imu) {
	int16_t x, y, z;
	
	//while(!sampleflag);
	//sampleflag = 0;
	
	imu_select(imu_number);
	
	x = i2c_read_reg(GYRO_ADDR, GYRO_REG_RATE_X_LSB);
	x |= i2c_read_reg(GYRO_ADDR, GYRO_REG_RATE_X_MSB) << 8;
	
	y = i2c_read_reg(GYRO_ADDR, GYRO_REG_RATE_Y_LSB);
	y |= i2c_read_reg(GYRO_ADDR, GYRO_REG_RATE_Y_MSB) << 8;
	
	z = i2c_read_reg(GYRO_ADDR, GYRO_REG_RATE_Z_LSB);
	z |= i2c_read_reg(GYRO_ADDR, GYRO_REG_RATE_Z_MSB) << 8;
	
	//temp = 23 + ((int8_t) i2c_read_reg(GYRO_ADDR, GYRO_REG_TEMP));
	
	imu->gyro.x = x;
	imu->gyro.y = y;
	imu->gyro.z = z;
	
	x = i2c_read_reg(ACCEL_ADDR, ACCEL_REG_OUT_X_L);
	x |= i2c_read_reg(ACCEL_ADDR, ACCEL_REG_OUT_X_H) << 8;
	
	y = i2c_read_reg(ACCEL_ADDR, ACCEL_REG_OUT_Y_L);
	y |= i2c_read_reg(ACCEL_ADDR, ACCEL_REG_OUT_Y_H) << 8;

	z = i2c_read_reg(ACCEL_ADDR, ACCEL_REG_OUT_Z_L);
	z |= i2c_read_reg(ACCEL_ADDR, ACCEL_REG_OUT_Z_H) << 8;
	
	imu->accel.x = x;
	imu->accel.y = y;
	imu->accel.z = z;
}
