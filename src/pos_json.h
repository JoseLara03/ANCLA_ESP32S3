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

#include "apos_store.h"
#include "pos_sink.h"

/* Zone identifier, published as "name" on the anchors topic. The position
 * topic no longer carries a zone field -- the consumer looks it up via the
 * anchors topic instead. */
#define POS_JSON_ZONE_NAME "852541"

/* Topic the survey trigger will arrive on. Declared here with the other topics,
 * composed from POS_JSON_ZONE_NAME, so a topic can never disagree with the zone
 * in its payload -- the same rule the position and anchors topics follow.
 *
 * NOT SUBSCRIBED YET: net_uplink.c has no subscribe path. Reserved so the name
 * is settled and visible next to its siblings rather than being invented later
 * in whichever file happens to add the subscription. */
#define POS_JSON_TOPIC_SURVEY "uwb/anchor/survey/" POS_JSON_ZONE_NAME

/* Buffer size that fits either document plus its NUL. Sized on the LARGER of the
 * two: a full APOS_MAX_NODES surveyed document (~150 bytes per anchor at the
 * widest coordinate and lat/long values), not the four-anchor stub it used to be
 * sized on. tests/pos_json/ asserts the worst case still fits. */
#define POS_JSON_MAX_LEN 1536

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

/* Format the zone/anchor map for the retained anchors topic.
 *
 * With a valid survey, emits the surveyed geometry: one entry per surveyed
 * anchor, named ANC-<zone>-<NNN> from its short address, with s->node[0] as the
 * axis/reference anchor carrying s->ref_lat/ref_lon. Every other anchor is
 * local-only (latitude/longitude 0) and positioned relative to the reference in
 * metres, which is what the schema already meant.
 *
 * With s == NULL or !s->valid, emits the original four-anchor stub unchanged, so
 * a gateway that has never been surveyed still publishes a schema-valid document
 * rather than an empty one. The stub's coordinates are placeholders; its schema
 * is the contract.
 *
 * Same return contract as pos_json_fix(): bytes written excluding the NUL, or -1
 * if the buffer was too small. On -1 the caller MUST drop the message --
 * publishing truncated JSON is worse than publishing nothing. */
int pos_json_anchors(char *buf, size_t len, const struct apos_survey *s);

#endif /* POS_JSON_H */
