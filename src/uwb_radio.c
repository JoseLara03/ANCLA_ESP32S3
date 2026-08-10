/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * DW3220 bring-up: reset, probe, initialise, configure the fixed PHY, enable
 * CIA diagnostics, apply antenna calibration and arm the IRQ.
 */

#include "uwb_radio.h"
#include "uwb_phy.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <deca_device_api.h>
#include <deca_probe_interface.h>
#include <dw3000_hw.h>

LOG_MODULE_REGISTER(uwb_radio, LOG_LEVEL_INF);

static dwt_config_t   phy_cfg   = UWB_PHY_CONFIG_INITIALIZER;
static dwt_txconfig_t tx_cfg    = UWB_PHY_TXCONFIG_INITIALIZER;

int uwb_radio_init(const uwb_config_t *cfg)
{
	int ret;

	ret = dw3000_hw_init();
	if (ret) {
		LOG_ERR("dw3000_hw_init failed (%d)", ret);
		return ret;
	}

	dw3000_hw_reset();

	/* The DW3220 needs a moment after RSTn is released before it answers SPI. */
	k_msleep(2);

	/* dwt_probe() reads DEV_ID over raw SPI, bypassing the register-access
	 * layer, so success here already proves the bus, CS and reset polarity
	 * are good. */
	ret = dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);
	if (ret) {
		LOG_ERR("dwt_probe failed (%d) — check SPI wiring, CS and reset polarity",
			ret);
		return ret;
	}

	/* dwt_initialise() MUST be the first register access after dwt_probe().
	 * It is what assigns the driver's local-data pointer (dw->priv), and
	 * every register access dereferences that pointer for the SPI-CRC mode.
	 * Calling anything else first — dwt_readdevid(), dwt_checkidlerc(),
	 * dwt_configure() — faults on a NULL dereference. The DW3220 is already
	 * in IDLE_RC by now: dw3000_hw_reset() plus the delay above cover the
	 * INIT_RC -> IDLE_RC time. */
	if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS) {
		LOG_ERR("dwt_initialise failed");
		return -EIO;
	}

	if (!dwt_checkidlerc()) {
		LOG_ERR("device not in IDLE_RC after init");
		return -EIO;
	}

	LOG_INF("DW3220 device ID: 0x%08x", dwt_readdevid());

	if (dwt_configure(&phy_cfg)) {
		LOG_ERR("dwt_configure failed — PHY rejected");
		return -EIO;
	}

	/* Must follow dwt_configure(); without it every diagnostic register
	 * reads zero, and the DISCOVERY response carries CIR power/quality. */
	dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);

	dwt_configuretxrf(&tx_cfg);
	dwt_settxantennadelay(cfg->ant_delay_tx);
	dwt_setrxantennadelay(cfg->ant_delay_rx);
	dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

	ret = dw3000_hw_init_interrupt();
	if (ret) {
		LOG_ERR("dw3000_hw_init_interrupt failed (%d)", ret);
		return ret;
	}

	LOG_INF("radio ready (ant_tx=%u ant_rx=%u)", cfg->ant_delay_tx,
		cfg->ant_delay_rx);
	return 0;
}
