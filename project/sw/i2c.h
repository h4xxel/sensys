#ifndef __I2C_H__
#define	__I2C_H__

#include <stdint.h>

void i2c_init(int frequency_khz);
uint8_t i2c_read_reg(uint8_t slave, uint8_t reg);
void i2c_write_reg(uint8_t slave, uint8_t reg, uint8_t data);

#endif
