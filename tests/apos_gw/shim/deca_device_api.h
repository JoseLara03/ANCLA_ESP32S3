/* Host-test shim for the handful of DW3000 driver calls apos_gw.c makes. The
 * test's fake radio captures every transmitted frame. */
#ifndef SHIM_DECA_DEVICE_API_H
#define SHIM_DECA_DEVICE_API_H

#include <stdint.h>

#define DWT_SUCCESS 0
#define DWT_ERROR (-1)
#define DWT_START_TX_IMMEDIATE 0
#define DWT_INT_TXFRS_BIT_MASK 0x80u
#define FCS_LEN 2UL

void dwt_forcetrxoff(void);
void dwt_writesysstatuslo(uint32_t mask);
int dwt_writetxdata(uint16_t len, uint8_t *buf, uint16_t off);
void dwt_writetxfctrl(uint16_t len, uint16_t off, int ranging);
int dwt_starttx(int mode);
uint32_t dwt_readsystimestamphi32(void);

#endif
