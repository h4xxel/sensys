#include <stdint.h>
#include "system/LPC11xx.h"
#include "util.h"
#include "uart.h"
#include "main.h"
#include "spi.h"
#include "radiolink.h"
#include "i2c.h"

#define GYRO_ADDR 0x68

uint32_t global_timer;
volatile uint32_t sampleflag = 0;

void initialize(void) {
	/* TODO: Set CPU clock etc. */
	
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);
	LPC_SYSCON->SYSAHBCLKDIV = 0x1;
	LPC_SYSCON->PDRUNCFG &= ~0x80;
	LPC_SYSCON->SYSPLLCTRL = 0x23;
	LPC_SYSCON->SYSPLLCLKSEL = 0x0;
	LPC_SYSCON->SYSPLLCLKUEN = 0x0;
	LPC_SYSCON->SYSPLLCLKUEN = 0x1;
	while(!(LPC_SYSCON->SYSPLLSTAT & 1));
	LPC_SYSCON->MAINCLKSEL = 0x3;
	LPC_SYSCON->MAINCLKUEN = 0x1;
	
	/*LPC_IOCON->PIO0_1 &= ~0x7;
	LPC_IOCON->PIO0_1 |= 0x1;
	LPC_SYSCON->CLKOUTDIV = 1;
	LPC_SYSCON->CLKOUTCLKSEL = 0x3;
	LPC_SYSCON->CLKOUTUEN = 0x1;*/
	
	/*********** Enable UART0 **********/
	uart_init();
	
	/********* Enable SPI0 ************/
	spi_init();

	/* Enable timers */
	LPC_SYSCON->SYSAHBCLKDIV |= (1 << 10);
	LPC_SYSCON->SYSAHBCLKDIV |= (1 << 9);
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 7);
	
	/*Disable systick*/
	SysTick->CTRL = 0;
	util_delay(100000);
	i2c_init(100);
	//uart_printf("i2c_init() done\n");
	radiolink_init(16);
	//uart_printf("radiolink_init() done\n");
}

void systick_irq() {
	global_timer++;
	sampleflag = 1;
}

void systick_enable() {
	SysTick->LOAD = SYSTEM_CLOCK / 50;
	SysTick->VAL = 0;
	SysTick->CTRL = 0x1 | 0x2 | 0x4;
}

int main(int ram, char **argv) {
	int16_t x, y, z;
	
	initialize();
	//uart_printf("Up and running!\n");
	systick_enable();
	
	//i2c_lol();

	LPC_GPIO0->DIR |= (1 << 3); // GPIO0_3 = IMUA0
	LPC_GPIO0->MASKED_ACCESS[(1 << 3)] = 0;	// Set A0 to 0 => addresses below should be valid
	//uart_printf("0x%X is value from accel (should be 0x41)\n", i2c_read_reg(0x1E, 0xF));
	
	i2c_write_reg(GYRO_ADDR, 0x0F, 0x84);
	
	uart_printf("OK 0x%X\r\n", i2c_read_reg(0x68, 0x0));
	
	for(;;) {
		while(!sampleflag);
		sampleflag = 0;
		
		x = i2c_read_reg(GYRO_ADDR, 0x2);
		x |= i2c_read_reg(GYRO_ADDR, 0x3) << 8;
		
		y = i2c_read_reg(GYRO_ADDR, 0x4);
		y |= i2c_read_reg(GYRO_ADDR, 0x5) << 8;
		
		z = i2c_read_reg(GYRO_ADDR, 0x6);
		z |= i2c_read_reg(GYRO_ADDR, 0x7) << 8;
		
		//temp = 23 + ((int8_t) i2c_read_reg(GYRO_ADDR, 0x8));
		
		uart_printf("%i %i %i\r\n", x, y, z);
	}
	
	return 0;
}
