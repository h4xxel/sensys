#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
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

static int serial_fd;

int open_serial(const char *file){
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
	
	if(cfsetispeed(&config, BAUDRATE) < 0 || cfsetospeed(&config, BAUDRATE) < 0)
		goto fail;
	
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
		goto fail;

	serial_fd = fd;
	return fd;
	fail:
	close(fd);
	return -1;
}


void wait_for_sync() {
	uint8_t data;
	int ff;

	for (ff = 0;;) {
		read(serial_fd, &data, 1);
		if (data == 0xFF)
			ff++;
		if (!data && ff > 2)
			break;
		if (data != 0xFF)
			ff = 0;
	}
}


void serial_get_package(uint8_t *data) {
	int i;

	for (i = 0; i < 26;) {
		i += read(serial_fd, &data[i], 26 - i);
	}
	return;
}


#if 0
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
#endif
