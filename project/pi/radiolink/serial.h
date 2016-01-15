#ifndef __SERIAL_H__
#define	__SERIAL_H__

#include <stdint.h>

int open_serial(const char *file);
void wait_for_sync();
void serial_get_package(uint8_t *data);


#endif
