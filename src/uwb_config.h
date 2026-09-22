/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Per-anchor configuration: identity, mode, antenna calibration and position.
 * Pure C with no Zephyr dependency so the validation is host-testable; the
 * persistence layer lives in uwb_store.c and the console in anchor_shell.c.
 *
 * The PHY is deliberately absent: it is a fixed network contract (see
 * uwb_phy.h), not a runtime setting.
 */

#ifndef UWB_CONFIG_H
#define UWB_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

/* Anchors in the deployment; valid ids are 0..UWB_MAX_ANCHORS-1. */
#define UWB_MAX_ANCHORS 4

/* Factory-reference seed for an uncalibrated unit, in DWT units. */
#define UWB_ANT_DELAY_DEFAULT 16385u

/* Anchor short addresses start at 1: the MAC contract reserves 0x0000 for the
 * gateway (spec/2026-06-17-uwb-mac-protocol-contract.md, Topology). The console
 * id stays 0-based; the wire address is derived. */
#define UWB_ANCHOR_ADDR_BASE 0x0001u

/* Reserved by the contract; declared here so the host test can assert no valid
 * id ever collides with it. */
#define UWB_ADDR_GATEWAY_RESERVED 0x0000u

enum uwb_mode {
	UWB_MODE_SLAVE   = 0, /* SS-TWR responder; observes the gateway beacon */
	UWB_MODE_GATEWAY = 1, /* emits the TDMA beacon and grants CFP seats    */
	UWB_MODE_COUNT
};

/* A GATEWAY's CFP partition. Meaningless on a SLAVE, same as anchor_id on a
 * GATEWAY -- defaults to TWR and is never read outside GATEWAY mode. See
 * docs/superpowers/specs/2026-08-30-blink-slotted-mac-design.md section 1.1:
 * a cell runs ranging OR blink, never both, so this is a per-cell build/boot
 * choice, not a per-superframe one. */
enum uwb_cell_mode {
	UWB_CELL_TWR   = 0, /* CFP repartitioned into GW_N_CFP ranging slots  */
	UWB_CELL_BLINK = 1, /* CFP repartitioned into BLINK_N_SLOTS blink slots */
	UWB_CELL_MODE_COUNT
};

/* B14 flight-compensation source. See sync_model.h's sync_model_set_tau() for
 * what tau_dtu corrects and docs/superpowers/plans/2026-09-22-b14-flight-compensation.md
 * for the full derivation.
 *
 * TWR and AUTO are Fase B stubs: this board has no on-air tau measurement
 * yet, so uwb_config_sync_tau_effective() resolves both of them to the same
 * stored survey value SURVEY uses. That is not a placeholder bug -- it is
 * FR-9's documented degrade-on-no-source behaviour, just permanently true
 * until Fase B ships a TWR source to degrade FROM. */
enum sync_tau_mode {
	SYNC_TAU_MODE_LEGACY = 0, /* tau forced to 0 -- FR-11 rollback, bit-exact
				   * pre-B14-fix behaviour regardless of the
				   * stored survey value */
	SYNC_TAU_MODE_SURVEY = 1, /* use sync_tau_dtu, from a one-time survey
				   * computation pushed over the console */
	SYNC_TAU_MODE_TWR    = 2, /* Fase B, not yet implemented */
	SYNC_TAU_MODE_AUTO   = 3, /* Fase B, not yet implemented */
	SYNC_TAU_MODE_COUNT
};

typedef struct {
	uint8_t  mode;           /* enum uwb_mode */
	uint8_t  anchor_id;      /* 0..UWB_MAX_ANCHORS-1 */
	uint8_t  cell_mode;      /* enum uwb_cell_mode; GATEWAY only */
	uint16_t ant_delay_tx;
	uint16_t ant_delay_rx;
	float    x;
	float    y;
	float    z;
	bool     position_valid; /* false until coordinates are set */
	int32_t  sync_tau_dtu;   /* SURVEY source value, signed DTU */
	uint8_t  sync_tau_mode;  /* enum sync_tau_mode */
} uwb_config_t;

/* Overwrite *c with the documented defaults: SLAVE, id 0, 16385/16385,
 * origin, position invalid. */
void uwb_config_set_defaults(uwb_config_t *c);

/* The single active instance, at defaults until uwb_store_load() runs. */
uwb_config_t *uwb_config_get(void);

/* Validating setters. Each returns true and mutates on success, or returns
 * false and leaves *c completely untouched on a rejected value. */
bool uwb_config_set_mode(uwb_config_t *c, uint8_t mode);
bool uwb_config_set_id(uwb_config_t *c, uint8_t id);
bool uwb_config_set_cell_mode(uwb_config_t *c, uint8_t cell_mode);
bool uwb_config_set_ant(uwb_config_t *c, uint32_t tx, uint32_t rx);

/* Always succeeds; any coordinate triple is legitimate. Sets position_valid. */
void uwb_config_set_pos(uwb_config_t *c, float x, float y, float z);

/* Always succeeds; any int32 is a legitimate (if perhaps absurd) flight-time
 * correction -- validating a physically-plausible range is the survey tool's
 * job, not this layer's, same as uwb_config_set_pos(). */
void uwb_config_set_sync_tau(uwb_config_t *c, int32_t tau_dtu);

bool uwb_config_set_sync_tau_mode(uwb_config_t *c, uint8_t mode);

/* "legacy" / "survey" / "twr" / "auto", matched exactly. Returns false and
 * leaves *out untouched on an unknown name. */
bool uwb_config_sync_tau_mode_from_name(const char *name, uint8_t *out);

/* One of the names above, or "unknown" for an out-of-range value. Never
 * NULL. */
const char *uwb_config_sync_tau_mode_name(uint8_t mode);

/* Resolves FR-3's source priority into the DTU value the sync model should
 * actually apply. TWR and AUTO currently resolve identically to SURVEY --
 * see enum sync_tau_mode's comment for why that is not a bug. */
int32_t uwb_config_sync_tau_effective(const uwb_config_t *c);

/* "slave" / "gateway", matched exactly. Returns false and leaves *out
 * untouched on an unknown name. */
bool uwb_config_mode_from_name(const char *name, uint8_t *out);

/* "slave", "gateway", or "unknown" for an out-of-range value. Never NULL. */
const char *uwb_config_mode_name(uint8_t mode);

/* "twr" / "blink", matched exactly. Returns false and leaves *out untouched
 * on an unknown name. */
bool uwb_config_cell_mode_from_name(const char *name, uint8_t *out);

/* "twr", "blink", or "unknown" for an out-of-range value. Never NULL. */
const char *uwb_config_cell_mode_name(uint8_t cell_mode);

/* This board's 16-bit short address. In SLAVE mode: UWB_ANCHOR_ADDR_BASE +
 * anchor_id, the single place the console id maps to the on-air address (its
 * low byte is also the id the tag echoes in the legacy WAVE poll). In GATEWAY
 * mode: always UWB_ADDR_GATEWAY_RESERVED, regardless of anchor_id -- that
 * field is meaningless on a gateway and defaults to 0, which used to alias
 * onto anchor id 0's real address. */
uint16_t uwb_config_short_addr(const uwb_config_t *c);

#endif /* UWB_CONFIG_H */
