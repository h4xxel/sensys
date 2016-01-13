#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <signal.h>
#include "vector.h"
#include "radiolink/protocol.h"
#include "gl.h"
#include "bone.h"

#define STR(s) XSTR(s)
#define XSTR(s) #s

#define SAMPRATE 50.0
#define BAUDRATE B115200

struct termios ttysave;

void atexit_restore_terminal() {
	tcsetattr(STDIN_FILENO, TCSANOW, &ttysave);
	printf("Terminal reset to normal\n");
	return;
}


void init_unbuffered_input() {
	struct termios ttystate;
	atexit(atexit_restore_terminal);

	tcgetattr(STDIN_FILENO, &ttystate);
	ttysave = ttystate;
	ttystate.c_lflag &= ~(ICANON | ECHO);
	ttystate.c_cc[VMIN] = 1;
	tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
	fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
	
	return;
}

void sighandler(int sig) {
	exit(0);
}


int main(int argc, char **argv) {
	signal(SIGINT, sighandler);
	init_grid();
	protocol_init();
	init_unbuffered_input();
	
	bone_parse("testbone.txt");
	launch_worker();
	run_triangle();
	
	return 0;
}

