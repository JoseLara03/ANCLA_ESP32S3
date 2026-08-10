/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Anchor boot: bring the DW3220 up with the active configuration.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "uwb_config.h"
#include "uwb_radio.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
	const uwb_config_t *cfg = uwb_config_get();
	int ret;

	ret = uwb_radio_init(cfg);
	if (ret) {
		LOG_ERR("radio bring-up failed (%d)", ret);
		return ret;
	}

	LOG_INF("DW3220 ready, IRQ armed");
	return 0;
}
