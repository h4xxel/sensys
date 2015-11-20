#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>

#define STR(s) XSTR(s)
#define XSTR(s) #s

#define SAMPRATE 120
#define BAUDRATE B115200
#define SCALE 21.0 //1G in accel raw data
#define G 9.81
#define DT (1.0/SAMPRATE)


//Calibrate for when board is stationary on surface
#define BIASX 1.3873873874
#define BIASY -0.4459459459

#define STARTTHRESHOLD -1.6
#define STOPTHRESHOLD 3.0

int open_serial(const char *file, speed_t baudrate){
	struct termios config;
	int fd;
	
	if((fd = open(file, O_RDWR | O_NOCTTY /*| O_NDELAY*/)) < 0) {
		return -1;
	}
	
	if(!isatty(fd))
		goto fail;

	if(tcgetattr(fd, &config) < 0)
		goto fail;
	
	//No input, output or line processing
	config.c_iflag = 0;
	config.c_oflag = 0;
	config.c_lflag = 0;
	
	//8N1
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
	
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 0;
	
	if(cfsetispeed(&config, baudrate) < 0 || cfsetospeed(&config, baudrate) < 0)
		goto fail;
	
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
		goto fail;

	return fd;
	fail:
	close(fd);
	return -1;
}

ssize_t serial_puts(int fd, const char *str) {
	size_t len;
	
	if(!str)
		return -1;
	
	len = strlen(str);
	return write(fd, str, len);
}

ssize_t serial_gets(int fd, char *buf, size_t len) {
	ssize_t bytes_read = 0;
	
	if(!buf || len < 1)
		return -1;
	
	do {
		if(bytes_read >= len - 1)
			break;
		read(fd, buf, 1);
		bytes_read++;
	} while(*buf++ != '\n');
	
	*buf = 0;
	return bytes_read;
}

int main(int argc, char **argv) {
	char buf[256];
	int serial;
	double rawx, rawy;
	double velx = 0, vely = 0;
	double posx = 0, posy = 0;
	double accx, accy;
	
	enum {
		STATE_IDLE,
		STATE_MEASURING,
	} state = STATE_IDLE;
	
	if(argc < 2) {
		fprintf(stderr, "Usage: sensys-lab2 /dev/serial-port\n");
		return 1;
	}
	
	if((serial = open_serial(argv[1], BAUDRATE)) < 0) {
		fprintf(stderr, "Error: could not open serial port\n");
		return 1;
	}
	
	serial_puts(serial, "AT+OSX=3\r\n");
	serial_puts(serial, "AT+OSR=" STR(SAMPRATE) "\r\n"); //120 Hz sample rate
	
	
	buf[255] = 0;
	for(;;) {
		serial_gets(serial, buf, 255);
		if(sscanf(buf, "DATA AXL: %lf,%lf", &rawx, &rawy) < 2)
			continue;
		
		accx = ((rawx - BIASX)/SCALE)*G;
		accy = ((rawy - BIASY)/SCALE)*G;
		
		switch(state) {
			case STATE_IDLE:
				velx = vely = posx = posy = 0;

				if(accy < STARTTHRESHOLD) {
					printf("Starting measurement!\n");
					state = STATE_MEASURING;
				}
				break;
			case STATE_MEASURING:
				velx += accx*DT;
				vely += accy*DT;
				posx += velx*DT;
				posy += vely*DT;
				
				if(accy > STOPTHRESHOLD) {
					printf("Ending measurement!\n");
					printf("Distance: %lf m\n", -posy);
					state = STATE_IDLE;
				}
				break;
		}
	}
	
	return 0;
}
