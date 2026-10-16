/* Host-test shim: the few kernel services src/apos_gw.c uses, on a fake clock
 * the test drives. NOT part of the firmware build. */
#ifndef SHIM_ZEPHYR_KERNEL_H
#define SHIM_ZEPHYR_KERNEL_H

#include <stdint.h>

typedef struct { int64_t ms; } k_timeout_t;
#define K_MSEC(x) ((k_timeout_t){(int64_t)(x)})

int64_t k_uptime_get(void);
int32_t k_sleep(k_timeout_t t);

#endif
