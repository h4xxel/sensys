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

typedef struct Vector3 Vector3;
struct Vector3 {
	double x;
	double y;
	double z;
};

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

void get_gyro(int fd, Vector3 *vec) {
	char buf[256];
	buf[255] = 0;
	serial_gets(fd, buf, 255);
	sscanf(buf, "%lf %lf %lf\r\n", &vec->x, &vec->y, &vec->z);
}

int main(int argc, char **argv) {
	char buf[256];
	int serial;
	Vector3 gyro;
	
	if(argc < 2) {
		fprintf(stderr, "Usage: sensys-lab2 /dev/serial-port\n");
		return 1;
	}
	
	if((serial = open_serial(argv[1], BAUDRATE)) < 0) {
		fprintf(stderr, "Error: could not open serial port\n");
		return 1;
	}
	
	do {
		serial_gets(serial, buf, 255);
	} while(!(buf[0] == 'O' && buf[1] == 'K'));
	
	buf[255] = 0;
	for(;;) {
		get_gyro(serial, &gyro);
		printf("%lf, %lf, %lf\n", gyro.x, gyro.y, gyro.z);
	}
	
	return 0;
}

