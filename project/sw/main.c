#include <stdint.h>
#include "system/LPC11xx.h"
#include "util.h"
#include "uart.h"
#include "main.h"
#include "spi.h"
#include "radiolink.h"
#include "i2c.h"
#include "imu.h"
#include "protocol.h"

#define RANGE IMU_RANGE_HIGHEST

#ifdef DEBUG
#define SAMPLE_FREQ 1
#else
#define SAMPLE_FREQ 50
#endif

uint32_t global_timer;
volatile uint32_t sampleflag = 1;

void initialize(void) {
	/* TODO: Set CPU clock etc. */
	
	LPC_SYSCON->SYSOSCCTRL = 0x3; //Use external oscillator
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);
	LPC_SYSCON->SYSAHBCLKDIV = 0x1;
	LPC_SYSCON->PDRUNCFG &= ~0x80;
	LPC_SYSCON->SYSPLLCTRL = 0x01;
	LPC_SYSCON->SYSPLLCLKSEL = 0x1; //Use system oscillator and not RC oscillator
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
	radiolink_init(PROTOCOL_PACKET_SIZE);
	//uart_printf("radiolink_init() done\n");
}

void systick_irq() {
	global_timer++;
	sampleflag = 1;
}

void systick_enable(int frequency) {
	SysTick->LOAD = SYSTEM_CLOCK / frequency;
	SysTick->VAL = 0;
	SysTick->CTRL = 0x1 | 0x2 | 0x4;
}

void setup_pins() {
	LPC_GPIO0->DIR |= (1 << 3); //IMUA0
	LPC_GPIO3->DIR |= (1 << 5); //IMUA1
	LPC_GPIO0->DIR |= (1 << 6); //IMUA2
	LPC_GPIO0->DIR |= (1 << 7); //IMUA3
	LPC_GPIO2->DIR |= (1 << 9); //IMUA4
	LPC_GPIO2->DIR |= (1 << 1); //IMUA5
}

int main(int ram, char **argv) {
	Imu imu[IMU_MAX];
	ProtocolPacket packet[IMU_MAX/2];
	
	uint32_t imus, tmp;
	int i, j;
	
	initialize();
	systick_enable(SAMPLE_FREQ);
	setup_pins();
	
	imus = imu_enumerate();
	
	for(i = 0, tmp = imus; tmp; i++, tmp >>= 1) {
		if(!(tmp & 0x1))
			continue;
		
		debug_printf("Found IMU %i\r\n", i);
		imu_setup(i, RANGE);
	}
	
	debug_printf("IMU setup done\r\n");
	
	unsigned char sync[26] = {
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 
	};
	uart_send_raw(sync, PROTOCOL_PACKET_SIZE);
	
	for(;;) {
		protocol_packet_new(packet, RANGE);
		
		if(sampleflag)
			debug_printf("Warning! Missed deadline\r\n");
		while(!sampleflag);
		sampleflag = 0;
		
		debug_printf("-----------------------------------------\r\n");
		for(i = 0, tmp = imus, j = 0; tmp; i++, tmp >>= 1) {
			if(!(tmp & 0x1))
				continue;
			
			imu_sample(i, imu + i);
		
			debug_printf("IMU %i:\r\n\tGyro: x=%hi, y=%hi, z=%hi\r\n\tAccel: x=%hi, y=%hi, z=%hi\r\n",
				i,
				imu[i].gyro.x,
				imu[i].gyro.y,
				imu[i].gyro.z,
				imu[i].accel.x,
				imu[i].accel.y,
				imu[i].accel.z
			);
			
			if(protocol_pack_imu(packet + j, imu + i, i) < 0) {
				j++;
				if(j < IMU_MAX/2)
					protocol_packet_new(packet + j, RANGE);
				protocol_pack_imu(packet + j, imu + i, i);
			}
		}
		
		j = (i + 1)/2;
		
		for(i = 0; i < j; i++) {
			if(!protocol_packet_isempty(packet + i)) {
				debug_printf("Sending packet %i\r\n", i);
				//radiolink_send_stubborn(PROTOCOL_PACKET_SIZE, (void *) (packet + i), 1000);
				uart_send_raw((void *) (packet + i), PROTOCOL_PACKET_SIZE);
			}
		}
	}
	
	return 0;
}
