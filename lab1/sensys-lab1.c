#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <asm/termios.h>

#define	BAUDRATE		115200

int ioctl(int d, unsigned long req, ...);
int tcflush(int fd, int q);

int initiate_serial_port(const char *file){
	struct termios2 config;
	int serial_port;
	
	serial_port = open(file, O_RDWR | O_NOCTTY /*| O_NDELAY*/);
	//Open serial port as ReadWrite.
	if(serial_port < 0) {
		return -1;
	}
	
	ioctl(serial_port, TCGETS2, &config);
	
	config.c_cflag &= ~CBAUD;
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= BOTHER;
	config.c_cflag |= CS8;
	//These options will let us set our own baudrate,
	//, no parity and a bytesize of 8 bits.
	
	config.c_iflag = 0;
	//No input processing.
	
	config.c_oflag = 0;
	//No output processing.
	
	config.c_lflag = 0;
	//No echoing and stuff.
	
	config.c_ispeed = BAUDRATE;
	config.c_ospeed = BAUDRATE;
	//Set our baudrates.
	
	ioctl(serial_port, TCSETS2, &config);
	//Sets the serial_port to work as we want it to,
	//with the custom baudrate BAUDRATE. 
	
	tcflush(serial_port, TCOFLUSH);
	tcflush(serial_port, TCIFLUSH);
	
	return serial_port;
}

int main(int argc, char **argv) {
	char buff[256];
	int serial_fd;
	int i;
	int light;
	
	enum {
		STATE_HANDSET,
		STATE_SPEAKER
	} state = STATE_HANDSET;
	
	if(argc < 2) {
		fprintf(stderr, "Usage: sensys-lab1 /dev/serial-port\n");
		return 1;
	}
	
	if((serial_fd = initiate_serial_port(argv[1])) < 0) {
		fprintf(stderr, "Error: could not open serial port\n");
		return 1;
	}
	
	write(serial_fd, "\r\n", 2);
	read(serial_fd, buff, 2);
	do {
		read(serial_fd, buff, 1);
	} while(*buff != '\n');
	
	write(serial_fd, "AT ", 3);
	while(1) {
		write(serial_fd, "S203?", 5);
		read(serial_fd, buff, 2);
		i = 0;
		do {
			read(serial_fd, &buff[i], 1);
			i++;
		} while(buff[i - 1] != '\n');
		buff[i] = 0;
		
		light = atoi(buff);
		
		switch(state) {
			case STATE_HANDSET:
				if(light < 3000) {
					state = STATE_SPEAKER;
					printf("Switched to speaker phone!\n");
				}
				break;
			case STATE_SPEAKER:
				if(light >= 3000) {
					state = STATE_HANDSET;
					printf("Switched to handset!\n");
				}
				break;
		}
		
		usleep(100000);
	}


	return 0;
}
