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

enum uwb_mode {
	UWB_MODE_SLAVE   = 0, /* SS-TWR responder; observes the gateway beacon */
	UWB_MODE_GATEWAY = 1, /* emits the TDMA beacon and grants CFP seats    */
	UWB_MODE_COUNT
};

typedef struct {
	uint8_t  mode;           /* enum uwb_mode */
	uint8_t  anchor_id;      /* 0..UWB_MAX_ANCHORS-1 */
	uint16_t ant_delay_tx;
	uint16_t ant_delay_rx;
	float    x;
	float    y;
	float    z;
	bool     position_valid; /* false until coordinates are set */
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
bool uwb_config_set_ant(uwb_config_t *c, uint32_t tx, uint32_t rx);

/* Always succeeds; any coordinate triple is legitimate. Sets position_valid. */
void uwb_config_set_pos(uwb_config_t *c, float x, float y, float z);

/* "slave" / "gateway", matched exactly. Returns false and leaves *out
 * untouched on an unknown name. */
bool uwb_config_mode_from_name(const char *name, uint8_t *out);

/* "slave", "gateway", or "unknown" for an out-of-range value. Never NULL. */
const char *uwb_config_mode_name(uint8_t mode);

#endif /* UWB_CONFIG_H */
