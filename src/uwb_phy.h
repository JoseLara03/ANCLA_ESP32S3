/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The fixed PHY contract. Every node on this network — anchors and tags —
 * must match these values exactly, so they are compile-time constants rather
 * than settings. Changing anything here changes the wire and invalidates
 * every unit's antenna calibration.
 */

#ifndef UWB_PHY_H
#define UWB_PHY_H

#include <deca_device_api.h>

/* Channel 5, PLEN-1024, PAC32, code 9, 850 kbps, SFD_IEEE_4Z, STS off. */
#define UWB_PHY_CONFIG_INITIALIZER                                             \
	{                                                                      \
		5,                  /* channel */                              \
		DWT_PLEN_1024,      /* TX preamble length */                   \
		DWT_PAC32,          /* RX preamble acquisition chunk */        \
		9,                  /* TX preamble code */                     \
		9,                  /* RX preamble code */                     \
		3,                  /* SFD type: 4z 8-symbol */                \
		DWT_BR_850K,        /* data rate */                            \
		DWT_PHRMODE_STD,                                               \
		DWT_PHRRATE_STD,                                               \
		(1025 + 8 - 32),    /* SFD timeout: PLEN + 1 + SFD - PAC */    \
		DWT_STS_MODE_OFF,                                              \
		DWT_STS_LEN_64,     /* ignored while STS is off */             \
		DWT_PDOA_M0                                                    \
	}

/* PG delay and TX power for channel 5. */
#define UWB_PHY_TXCONFIG_INITIALIZER                                           \
	{                                                                      \
		0x34,       /* PG delay */                                     \
		0xfafafafa, /* TX power */                                     \
		0x0         /* PG count */                                     \
	}

#endif /* UWB_PHY_H */
