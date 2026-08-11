/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The handful of helpers the Qorvo examples provide in
 * examples/shared_data/shared_functions.c, which the vendored br101 module does
 * not carry -- it ships only the driver and the platform layer. Kept in one
 * place so that dependency is visible rather than smeared across the
 * responders.
 */

#ifndef UWB_DWTIME_H
#define UWB_DWTIME_H

#include <stdbool.h>
#include <stdint.h>
#include <deca_device_api.h>

/* DWT time units per UWB microsecond.
 *
 * fw-cre's shared_defines.h says 63898; this project uses 65536, which is what
 * both peers on this network use -- tag_testting/src/uwb_ss_initiator.c and
 * ESP32S3UWB/src/ranging.c, the latter having ranged against that tag on
 * hardware. The constant scales the delayed-TX turnaround both ends must agree
 * on, so the source's value is not merely a different convention here. */
#define UUS_TO_DWT_TIME 65536UL

/* Width of a timestamp field in the legacy VEWA response. */
#define RESP_MSG_TS_LEN 4u

/* Frame check sequence appended by the radio. dwt_getframelength() and
 * cb_data->datalength both report payload + FCS. The vendored driver header
 * (deca_device_api.h) also defines this constant as 2UL; the #ifndef guard
 * avoids a redefinition conflict when both headers are included. */
#ifndef FCS_LEN
#define FCS_LEN 2u
#endif

/* The 40-bit RX timestamp of the last received frame, as a 64-bit value.
 * Valid until the next reception. */
uint64_t uwb_get_rx_timestamp_u64(void);

/* Write the low 4 bytes of ts into a response timestamp field, least
 * significant byte first. */
void uwb_resp_msg_set_ts(uint8_t *ts_field, uint64_t ts);

/* Spin until every bit in lo_mask is set in the low system status register,
 * or timeout_ms elapses. Returns true if the bits set in time, false on
 * timeout -- the caller must treat a timeout as a failed operation and
 * recover (e.g. dwt_forcetrxoff()).
 *
 * The timeout is not defensive padding: dwt_starttx()'s own HPDWARN deadline
 * check can race a borderline-late delayed TX and still report DWT_SUCCESS
 * for a transmission that never actually completes, leaving TXFRS unset
 * forever. An earlier unbounded version of this spin hung the whole console
 * (main thread runs at priority 0) the first time that race actually landed
 * on real hardware, confirmed at DISC_BASE_UUS=2400 in anchor_respond.c.
 *
 * Only safe for events the ISR does not clear first. Never call this with
 * DWT_INT_CIADONE_BIT_MASK after an RX event: dwt_isr() clears
 * SYS_STATUS_ALL_RX_GOOD -- which includes CIADONE -- before invoking the RX
 * callback, so the bit will not set again until the next frame and this would
 * spin until timeout every time. The same applies to TXFRS if TX interrupts
 * are ever enabled. */
bool uwb_wait_for_sysstatus_lo(uint32_t lo_mask, uint32_t timeout_ms);

#endif /* UWB_DWTIME_H */
