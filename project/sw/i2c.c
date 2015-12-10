/*
	Original implementation by Paul Coada in 2014
	as part of the Autokorg project.
	Modified by Axel Isaksson in 2015
	
	Original license was GPLv3, relicensed with the permission of the
	original author as 3-clause BSD.
*/

#include <stdint.h>
#include "system/LPC11xx.h"
#include "uart.h"
#include "util.h"

typedef enum I2CStatus I2CStatus;
enum I2CStatus {
	I2C_STATUS_ERR = -1,
	I2C_STATUS_ACK = 0,
	I2C_STATUS_NACK,
};

void i2c_init(int frequency_khz) {
	// I2C basic configuration at 15.2
	int a;
	
	LPC_I2C->CONCLR		= 0x6C;		// Clearing: AA, SI, STA and I2EN
	
	LPC_SYSCON->SYSAHBCLKCTRL	|= (1<<5);	//Enables clock for I2C (3.5.14)
	LPC_SYSCON->PRESETCTRL		|= (0x2);	//I2C reset not allowed (3.5.2)

	LPC_IOCON->PIO0_4	= 0x1;		//FUNC: Select I2C function SCL (7.4.11)
	LPC_IOCON->PIO0_5	= 0x1;		//FUNC: Select I2C function SDA (7.4.12)
		
	/*	System clock at 48 MHz, and we want I2C in Fast-Mode (400 kHz).
	 * 	Thus our divider should be 120. HIGH and LOW should be of equal length,
	 * 	so each is set at 60 (15.7.5).
	 */
	
	a = SYSTEM_CLOCK/(frequency_khz * 1000);
	LPC_I2C->SCLH	= a/2;
	LPC_I2C->SCLL	= a/2;
	
	LPC_I2C->CONSET		|= (1<<6);	//I2EN: I2C interface enabled (SDA and SCL not ignored) (15.7.1)
	
}

void i2c_start() {
	LPC_I2C->CONSET		|= (1<<5);	//Send START flag (S). I2C interface is set to master mode. (15.7.1)
	while ((LPC_I2C->STAT & 0xF8) != 0x08);	//Waiting for start-sent state (0x08). We ignore reserved and 0-bits.
	LPC_I2C->CONCLR		= 0x28;		//clears START flag (bit) and Serial Interrupt bit (SI) (15.7.6)
}

void i2c_restart() {
	LPC_I2C->CONSET		|= (1<<5);	//Send START flag (S). I2C interface is set to master mode. (15.7.1)
	while ((LPC_I2C->STAT & 0xF8) != 0x10);	//Waiting for start-sent state (0x08). We ignore reserved and 0-bits.
	LPC_I2C->CONCLR		= 0x20;		//clears START flag
}

void i2c_stop() {
	LPC_I2C->CONCLR		= (1<<3);	//Clear SI (15.7.6)
	LPC_I2C->CONSET		|= (1<<4);	//Send STOP flag (P). Bit cleared when detected on bus. (15.7.1)
	while(LPC_I2C->CONSET & (1<<3));
	LPC_I2C->CONCLR		= (1<<3);
}

I2CStatus i2c_write(uint8_t byte) {
	LPC_I2C->CONCLR		= (1<<3);	//Clear SI (15.7.6)
	LPC_I2C->DAT = byte;	//Send slave address (SLA) 7 bits + data direction bit (W: 0, R: 1) (15.7.3)
	while(LPC_I2C->CONSET & (1<<3));
	switch(LPC_I2C->STAT & 0xF8) {
		case 0x18:
		case 0x40:
			return I2C_STATUS_ACK;
		
		case 0x20:
		case 0x48:
			return I2C_STATUS_NACK;
		
		default:
			return I2C_STATUS_ERR;
	}
	/*if (temp == 0x18)	break; 	// (SLA+W) sent, ACKed.
		else if (temp == 0x20)	break; 	// (SLA+W) sent, NACKed.
		else if (temp == 0x38)	break; 	// (SLA+W) sent, arbitration lost.
	}*/
}

void i2c_write_reg(uint8_t slave, uint8_t reg, uint8_t data) {
	i2c_start();
	i2c_write(slave << 1);
	i2c_write(reg);
	i2c_write(data);
	i2c_stop();
}

uint8_t i2c_read(I2CStatus ack) {
	int temp;
	
	if(ack == I2C_STATUS_ACK)
		LPC_I2C->CONCLR = 0x4;
	else
		LPC_I2C->CONSET = 0x4;
	
	LPC_I2C->CONCLR = 0x8;	//Clear SI (15.7.6)
	while(1) {
		temp = (LPC_I2C->STAT & 0xF8);
		if (temp == 0x50)	break;	// DATA (sub-adr value) received, ACK sent.
		else if (temp == 0x58)	break;	// DATA (sub-adr value) received, NACK sent.
	}
	
	return LPC_I2C->DAT;
}

uint8_t i2c_read_reg(uint8_t slave, uint8_t reg) {
	uint8_t data;
	i2c_start();
	i2c_write((slave << 1) | 0x1);
	i2c_write(reg);
	i2c_restart();
	data = i2c_read(I2C_STATUS_NACK);
	i2c_stop();
	return data;
}
