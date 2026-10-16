#ifndef SHIM_ZEPHYR_HWINFO_H
#define SHIM_ZEPHYR_HWINFO_H
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
ssize_t hwinfo_get_device_id(uint8_t *buffer, size_t length);
#endif
