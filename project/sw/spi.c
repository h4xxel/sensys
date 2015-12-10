/*
	Original implementation by Axel Isaksson in 2015
	as part of the Autokorg project.
	Modified by Axel Isaksson in 2015
	
	Original license was GPLv3, relicensed with the permission of the
	original author as 3-clause BSD.
*/

#include "system/LPC11xx.h"
#include "spi.h"
#include <stdint.h>


void spi_init() {
	LPC_IOCON->SCK_LOC  = 0x0;
	//LPC_IOCON->PIO0_6 &= ~0x7;
	LPC_IOCON->SWCLK_PIO0_10 = 0x2;
	LPC_IOCON->PIO0_8 &= ~0x7;
	LPC_IOCON->PIO0_8 |= 0x1;
	LPC_IOCON->PIO0_9 &= ~0x7;
	LPC_IOCON->PIO0_9 |= 0x1;
	//LPC_IOCON->SCK_LOC  = 2;
	//LPC_SYSCON->PRESETCTRL |= 0x5;
	//LPC_SYSCON->SYSAHBCLKCTRL &= ~(1 << 11);
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 11);
	LPC_SYSCON->SSP0CLKDIV = 4;
	LPC_SYSCON->PRESETCTRL |= 0x1;
	
	LPC_SSP0->CPSR = 0x2;
	LPC_SSP0->CR0 = 0x107;
	LPC_SSP0->CR1 = 0x2;

	return;
}


uint8_t spi_send_recv(uint8_t data) {
	while(!(LPC_SSP0->SR & 0x2));
	LPC_SSP0->DR = data;
	while(!(LPC_SSP0->SR & 0x4));
	return LPC_SSP0->DR;
}
