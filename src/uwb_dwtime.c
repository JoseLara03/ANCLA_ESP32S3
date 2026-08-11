/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "uwb_dwtime.h"

#include <zephyr/kernel.h>

#include <deca_device_api.h>

uint64_t uwb_get_rx_timestamp_u64(void)
{
	uint8_t ts_tab[5];
	uint64_t ts = 0;

	dwt_readrxtimestamp(ts_tab, DWT_COMPAT_NONE);
	for (int8_t i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

uint64_t uwb_get_tx_timestamp_u64(void)
{
	uint8_t ts_tab[5];
	uint64_t ts = 0;

	dwt_readtxtimestamp(ts_tab);
	for (int8_t i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

void uwb_resp_msg_set_ts(uint8_t *ts_field, uint64_t ts)
{
	for (uint8_t i = 0; i < RESP_MSG_TS_LEN; i++) {
		ts_field[i] = (uint8_t)(ts >> (i * 8));
	}
}

bool uwb_wait_for_sysstatus_lo(uint32_t lo_mask, uint32_t timeout_ms)
{
	int64_t deadline = k_uptime_get() + (int64_t)timeout_ms;

	while (!(dwt_readsysstatuslo() & lo_mask)) {
		if (k_uptime_get() >= deadline) {
			return false;
		}
	}
	return true;
}
