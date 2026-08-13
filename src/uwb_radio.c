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
#include <zephyr/devicetree.h>

#include <deca_device_api.h>
#include <deca_probe_interface.h>
#include <dw3000_hw.h>
#include <dw3000_spi.h>

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

	/* Up to fast rate now that the part is out of INIT_RC. Until this
	 * point the DW3000 requires <= 7 MHz (deca_device_api.h:2381, :2601),
	 * which is why dw3000_spi_init() starts on spi_cfgs[0] at 2 MHz.
	 *
	 * Nothing in the driver does this for us on this build: .setfastrate
	 * is wired into the vtable (platform/deca_port.c:37) but its only
	 * call sites are inside a static init() wrapper, and dwt_initialise()
	 * dispatches to ull_initialise instead (dw3000_device.c:9371). Without
	 * this call the whole session runs at 2 MHz -- which is what it did
	 * until now, while the module's own boot log printed "max 8MHz" from
	 * the never-selected spi_cfgs[1]. */
	dw3000_spi_speed_fast();

	LOG_INF("SPI at fast rate (%u Hz requested)",
		(unsigned)DT_PROP(DT_INST(0, decawave_dw3000), spi_max_frequency));

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
	/* Must precede dwt_setlnapamode(): driving the external PA requires fine
	 * grain TX sequencing to be off (Qorvo API note on dwt_setlnapamode() in
	 * deca_device_api.h), and it is ON by default. Left on, EXTTXE does not
	 * hold the PA asserted across the frame and the board transmits far below
	 * its rated power -- measured on the bench as this board's frames landing
	 * at -90..-92 dBm where a DWM3001C anchor at comparable range landed at
	 * -74..-76 dBm, i.e. within 1-3 dB of the DW3000's ~-93 dBm sensitivity
	 * floor, which cost ~40 % of DISCOVERY responses to packet loss. The
	 * symptom is TX-only and looks like an RF range problem, not a firmware
	 * one: RX is unaffected, so the anchor still hears every poll it fails to
	 * answer audibly. */
	dwt_setfinegraintxseq(0);
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
