/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MQTT payload formatting. Pure C with no Zephyr dependency so the wire
 * contract is host-testable; the transport lives in net_uplink.c.
 *
 * The position payload is a fixed contract with an existing consumer. Do not
 * add, remove or rename fields without changing that consumer too.
 */

#ifndef POS_JSON_H
#define POS_JSON_H

#include <stddef.h>

#include "pos_sink.h"

/* Zone identifier, published as "name" on the anchors topic. The position
 * topic no longer carries a zone field -- the consumer looks it up via the
 * anchors topic instead. */
#define POS_JSON_ZONE_NAME "852541"

/* Buffer size that fits either document plus its NUL. The anchors stub is the
 * larger of the two (four named anchors plus one real lat/long pair);
 * tests/pos_json/ asserts it still fits. */
#define POS_JSON_MAX_LEN 640

/* Format one fix as the position payload:
 *   {"Tid":4660,"x":1.23,"y":4.56,"z":0}
 *
 * Tid is fix->src_addr as a plain decimal number (NOT hex, NOT a string) --
 * 0x1234 formats as 4660. z is the integer literal 0: the solver is 2D and
 * there is no z measurement yet.
 *
 * Returns the number of bytes written excluding the NUL, or -1 if the buffer
 * was too small. On -1 the caller MUST drop the message: publishing a
 * truncated JSON document is worse than publishing nothing. */
int pos_json_fix(char *buf, size_t len, const struct pos_fix *fix);

/* Format the stubbed zone/anchor map. Same return contract.
 * Placeholder coordinates until the anchor survey work lands. */
int pos_json_anchors(char *buf, size_t len);

#endif /* POS_JSON_H */
