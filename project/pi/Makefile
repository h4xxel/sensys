BIN=sensys
OBJS=triangle.o thread.o main.o bone.o linalg.o

ifeq ($(shell uname -m),armv6l)
include Makefile.pi
else
include Makefile.pc
endif

all: $(BIN) $(LIB)

%.o: %.c
	@echo "[ CC ] $@"
	@rm -f $@ 
	@$(CC) $(CFLAGS) $(INCLUDES) -g -c $< -o $@ -Wno-deprecated-declarations

%.o: %.cpp
	@echo "[ CPP] $@"
	@rm -f $@ 
	@$(CXX) $(CFLAGS) $(INCLUDES) -g -c $< -o $@ -Wno-deprecated-declarations

$(BIN): $(OBJS)
	@echo "[ CC ] $@"
	@$(CC) -o $@ -Wl,--whole-archive $(OBJS) $(LDFLAGS) -Wl,--no-whole-archive -rdynamic

%.a: $(OBJS)
	$(AR) r $@ $^

clean:
	for i in $(OBJS); do (if test -e "$$i"; then ( rm $$i ); fi ); done
	@rm -f $(BIN) $(LIB)

