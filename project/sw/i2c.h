#ifndef __I2C_H__
#define	__I2C_H__

#include <stdint.h>

void i2c_init(void);
uint8_t i2c_read_reg(uint8_t slave_sub_adr);
void i2c_write_reg(uint8_t slave_sub_adr, uint8_t data);

#endif
