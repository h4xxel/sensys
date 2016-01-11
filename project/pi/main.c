#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include "vector.h"

#define STR(s) XSTR(s)
#define XSTR(s) #s

#define SAMPRATE 50.0
#define BAUDRATE B115200

struct termios ttysave;

void atexit_restore_terminal() {
	tcsetattr(STDIN_FILENO, TCSANOW, &ttysave);
	return;
}


void init_unbuffered_input() {
	struct termios ttystate;
	atexit(atexit_restore_terminal);

	tcgetattr(STDIN_FILENO, &ttystate);
	ttysave = ttystate;
	ttystate.c_lflag &= ~(ICANON | ECHO);
	ttystate.c_cc[VMIN] = 1;
	tcsetattr(STDIN_FILENO, TCSANOW, &ttysave);
	fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
	
	return;
}


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
	double x, y, z;
	
	char buf[256];
	buf[255] = 0;
	serial_gets(fd, buf, 255);
	sscanf(buf, "%lf %lf %lf\r\n", &x, &y, &z);
	vec->x = x/32768.0*125.0*(1.0/SAMPRATE);
	vec->y = y/32768.0*125.0*(1.0/SAMPRATE);
	vec->z = z/32768.0*125.0*(1.0/SAMPRATE);
}


int main(int argc, char **argv) {
	char buf[256];
	int serial;
	Vector3 gyro;
	
	/*if(argc < 2) {
		fprintf(stderr, "Usage: sensys-lab2 /dev/serial-port\n");
		return 1;
	}
	
	if((serial = open_serial(argv[1], BAUDRATE)) < 0) {
		fprintf(stderr, "Error: could not open serial port\n");
		return 1;
	}*/
	
	protocol_init();
	init_unbuffered_input();

	#if 0
	printf("Waiting for motherboard reset...\n");
	do {
		serial_gets(serial, buf, 255);
	} while(!(buf[0] == 'O' && buf[1] == 'K'));
	printf("OK\n");
	
	buf[255] = 0;
	#endif
	launch_worker(serial);
	run_triangle();
	#if 0
	for(;;) {
//		get_gyro(serial, &gyro);
//		printf("%lf, %lf, %lf\n", gyro.x, gyro.y, gyro.z);
	}
	#endif
	
	return 0;
}

