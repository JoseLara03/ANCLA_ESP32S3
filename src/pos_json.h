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

/* Buffer size that fits either document plus its NUL. Sized on the LARGER of
 * the two: a full APOS_MAX_NODES (32) surveyed document, not the four-anchor
 * stub. tests/pos_json/ asserts the worst case still fits.
 *
 * Per-anchor derivation, at the widest values each field can legitimately hold:
 *   92  literal characters of the entry template, comma included
 *    5  "%03u" of a uint16_t short address
 *   10  "false" twice, for isAxis and isReferenceAxis
 *   25  "%.8f" latitude and longitude -- "-90.00000000" and
 *       "-180.00000000", both bounded because `apos ref` range-checks them
 *   48  "%.2f" x, y and z at 16 characters each, i.e. metre coordinates up to
 *       +/-99999999.99, which is orders of magnitude past any real site
 *  ---
 *  180 bytes per anchor, x32 = 5760, plus 28 for the zone wrapper, 2 for the
 *  closing "]}" and 1 for the NUL = 5791. Rounded up to 5888.
 *
 * That is an upper BOUND, not the observed length: tests/pos_json/ builds
 * exactly that worst case and measures 4928 bytes, so the constant carries
 * ~16 % real headroom and the test asserts at least 5 %.
 *
 * NOT chunked, and that is a decision rather than an omission. The anchors
 * topic is a RETAINED single document that a downstream consumer parses whole;
 * splitting it across publishes would mean the consumer seeing a partial map
 * between them, and there is no sequencing field in the schema to reassemble
 * one. Nor is truncation an option -- pos_json_anchors() returns -1 rather than
 * emitting a short document, precisely because a truncated retained publish
 * poisons the topic until the next connect.
 *
 * The MQTT transport does NOT need to grow with this. net_uplink.c's
 * MQTT_TX_BUF_SIZE (256) holds only the encoded fixed header, topic and message
 * id: Zephyr's mqtt_publish() sends the payload as a SEPARATE iovec entry
 * straight from the caller's buffer (subsys/net/lib/mqtt/mqtt.c), so a 5.7 kB
 * payload never passes through client.tx_buf. */
#define POS_JSON_MAX_LEN 5888

/* Format one fix as the position payload:
 *   {"Tid":4660,"x":1.23,"y":4.56,"z":0}
 *
 * Tid is fix->tag_id (a stable per-physical-tag id derived from the tag's
 * EUI -- see src/tag_id.c) as a plain decimal number (NOT hex, NOT a
 * string). It is NOT fix->src_addr, which is only the tag's current MAC
 * short address and is reallocated across a rejoin. z is the integer
 * literal 0: the solver is 2D and there is no z measurement yet.
 *
 * Returns the number of bytes written excluding the NUL, or -1 if the buffer
 * was too small. On -1 the caller MUST drop the message: publishing a
 * truncated JSON document is worse than publishing nothing. */
int pos_json_fix(char *buf, size_t len, const struct pos_fix *fix);

/* Format the zone/anchor map for the retained anchors topic.
 *
 * With a valid survey, emits the surveyed geometry: one entry per surveyed
 * anchor, named ANC-LOBBY-<NNN> from its short address, with s->node[0] as the
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
