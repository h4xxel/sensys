BIN=sensys
OBJS=triangle.o thread.o main.o bone.o

ifeq ($(shell uname -m),armv6l)
include Makefile.pi
else
include Makefile.pc
endif
