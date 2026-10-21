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
#include <stdint.h>

#include "apos_store.h"
#include "pos_sink.h"

/* Zone identifier, published as "name" on the anchors topic. The position
 * topic no longer carries a zone field -- the consumer looks it up via the
 * anchors topic instead. */
#define POS_JSON_ZONE_NAME "852541"

/* Short address of the gateway in a stored survey (the CCP root). Restated
 * here, not included from uwb_config.h, so this file stays host-buildable on
 * its own; it is UWB_ADDR_GATEWAY_RESERVED and tests/pos_json/ pins that. */
#define POS_JSON_GATEWAY_ADDR 0x0000u

/* Topic the survey trigger will arrive on. Declared here with the other topics,
 * composed from POS_JSON_ZONE_NAME, so a topic can never disagree with the zone
 * in its payload -- the same rule the position and anchors topics follow.
 *
 * NOT SUBSCRIBED YET: net_uplink.c has no subscribe path. Reserved so the name
 * is settled and visible next to its siblings rather than being invented later
 * in whichever file happens to add the subscription. */
#define POS_JSON_TOPIC_SURVEY "uwb/anchor/survey/" POS_JSON_ZONE_NAME

/* TDoA observation topic: every anchor publishes one per BLINK it hears, and
 * the gateway subscribes. Zone-scoped and composed from POS_JSON_ZONE_NAME like
 * its siblings, so the topic can never disagree with the zone of the payload
 * published on it.
 *
 * UNLIKE POS_JSON_TOPIC_SURVEY, this one IS subscribed -- net_uplink.c gained a
 * subscribe path in Phase 3 Task 5. */
#define POS_JSON_TOPIC_BLINK "uwb/anchor/blink/" POS_JSON_ZONE_NAME

/* Periodic node status (src/node_status.c): one RETAINED topic per node,
 * POS_JSON_TOPIC_STATUS "<EUI16>" (an anchor's EUI, node_eui.h) or "gw" -- never the anchor
 * id, which two boards can share. Anchors cannot be reached once mounted,
 * so this is how `sync stats`, tau and the reset cause are read in the field. */
#define POS_JSON_TOPIC_STATUS "uwb/anchor/status/" POS_JSON_ZONE_NAME "/"

/* Remote commands (src/node_cmd.c, wire format in cmd_proto.h): each node subscribes to
 * POS_JSON_TOPIC_CMD "<EUI16>|gw" and answers on POS_JSON_TOPIC_ACK "<same>". Commands are
 * HMAC-signed against the challenge the node publishes in its status report, with that same
 * name as the signed node. */
#define POS_JSON_TOPIC_CMD "uwb/anchor/cmd/" POS_JSON_ZONE_NAME "/"
#define POS_JSON_TOPIC_ACK "uwb/anchor/ack/" POS_JSON_ZONE_NAME "/"

/* One observation: which anchor heard which BLINK and when, already in the
 * master's time base. Produced by blink_rx.c on the anchor and consumed by the
 * gateway (Task 6). */
struct pos_blink_obs {
	/* The anchor's id as it was when it published: 0..UWB_MAX_ANCHORS-1, or
	 * 255 for an anchor no gateway has assigned (UWB_ANCHOR_ID_UNASSIGNED).
	 * Only a fallback since 2026-10-21 -- `eui` below is the identity. */
	uint8_t  anchor_id;
	uint8_t  blink_seq;   /* from the frame; with tag_addr it is the group key */
	uint8_t  batt_soc;    /* 0-100, or UWB_FRAME_POS_SOC_CONNECTED */
	uint16_t tag_addr;    /* short address of the emitting tag */
	uint16_t quality;     /* CIR quality, diagnostics -- the solver ignores it */
	/* The BLINK's flags byte, forwarded WHOLE rather than as a decoded
	 * `moving` boolean. Same size, same code, and BLINK_FLAG_ALERT (which
	 * already exists and which the gateway has never seen) comes along for
	 * free -- as does the next flag, with no new field and no new parser
	 * branch. Decode with the BLINK_FLAG_* masks in blink_frame.h.
	 *
	 * Zero when the publishing anchor is older than proto 5 and did not
	 * send the field: the parser leaves it at 0, which reads as "not
	 * moving, no alert". That is the safe direction for the consumer --
	 * see pos_json_blink_parse()'s note. */
	uint8_t  flags;
	/* The publishing anchor's own 1-sigma TIMEBASE-CONVERSION error over a
	 * full CCP interval, in DTU (sync_model_sigma_dtu()). Lets the gateway
	 * weight each anchor's equations instead of assuming every link is
	 * equally good.
	 *
	 * Since 2026-09-18 this is the conversion error, not the raw link
	 * jitter (it used to be sync_model_jitter_est_dtu()). The wire meaning
	 * is compatible -- both are a 1-sigma DTU figure and both are larger
	 * for a worse anchor -- but the new one additionally RISES while that
	 * anchor is mid-re-baseline and falls as its baseline reconverges,
	 * which the long-run jitter statistic could not do at all. That was
	 * B13: an anchor coming out of a gateway reboot published its usual
	 * confidence while converting ~35x worse.
	 *
	 * 0 means UNKNOWN -- an anchor older than proto 5 that never sent the
	 * field. A consumer must treat 0 as "fall back to a flat sigma" and
	 * never as zero uncertainty, which would read as infinite confidence.
	 * A proto-5 anchor cannot publish 0: an anchor that cannot convert
	 * publishes no observation at all, and every convertible state clamps
	 * to at least 1.
	 *
	 * Mostly per-ANCHOR and slowly varying rather than per-blink: its
	 * jitter input is a long-run statistic for that anchor's CCP link, and
	 * only the baseline-state term moves faster. Per-blink discrimination
	 * is what `quality` above would be for, and whether ipatovAccumCount
	 * carries any at fixed PLEN has NOT been measured -- until it has,
	 * `quality` stays diagnostic and nothing weights by it. */
	uint16_t sigma_dtu;
	int64_t  t_dtu;       /* RX in the master's base, sync_model_to_master() */
	/* The publishing anchor's EUI-64, on the wire as "u" (16 hex digits).
	 * This is what the gateway and the edge resolve the anchor by; the id is
	 * a radio handle `apos assign` may move. has_eui is false for an anchor
	 * older than this field, which is then resolved by `anchor_id` as
	 * before. */
	bool     has_eui;
	uint8_t  eui[8];
};

/* Buffer big enough for pos_json_blink()'s worst case plus its NUL, and ALSO
 * the parser's hard rejection ceiling: pos_json_blink_parse() refuses any
 * payload of this size or more outright.
 *
 * ---- This constant is a VERSIONING CONSTRAINT, not just a buffer size ----
 *
 * Because it is a hard rejection, the "tolerates unknown fields" promise on
 * the parser holds only while the whole document stays under it. A newer
 * publisher whose maximal payload crosses the line makes an OLDER gateway
 * reject EVERY observation rather than ignore the extras -- a silent, total
 * loss of the TDoA input, not a graceful degradation. So adding a field means
 * measuring the new worst case first, and raising this (on GATEWAYS BEFORE
 * ANCHORS) if it no longer fits with room to spare.
 *
 * Raised 96 -> 128 on 2026-09-03, when proto 5 added "f" (flags) and "e"
 * (the anchor's measured sigma). Measured worst case went 66 -> 84 bytes,
 * which still fit under the old 95-byte limit but left only 11 bytes of
 * headroom -- enough to make the NEXT field an emergency rather than an
 * edit. tests/pos_json_blink/ prints the measured figure on every run
 * precisely so this is never estimated again. */
#define POS_JSON_BLINK_MAX_LEN 128

/* Upper bound on t_dtu: the DW3220's system counter is 40 bits, so anything
 * above this is not a timestamp this network can have produced. Named here
 * rather than written inline in the parser so the wire contract and the check
 * that enforces it cannot drift apart. */
#define POS_JSON_BLINK_TS_MAX 0xFFFFFFFFFFLL

/* Format one observation:
 *   {"a":2,"t":257,"s":90,"ts":"123456789012","q":1234,"b":77,"f":0,"e":22,
 *    "u":"7CDFA1B2C3D40000"}
 *
 * `ts` goes out as a QUOTED decimal string, not as a JSON number, and that is
 * deliberate: it is a 40-bit value (up to 1099511627775) and the Tid lesson (the
 * platform's int32 column, which SILENTLY dropped everything above INT32_MAX --
 * see CLAUDE.md) is that a large unquoted integer invites some consumer to
 * narrow it without saying so. The consumer of THIS topic is our own gateway, so
 * the encoding is ours to choose.
 *
 * Returns bytes written excluding the NUL, or -1 if it does not fit. On -1 the
 * caller MUST drop: publishing truncated JSON is worse than publishing
 * nothing. */
int pos_json_blink(char *buf, size_t len, const struct pos_blink_obs *o);

/* Parse what pos_json_blink() produces. `buf` need NOT be NUL-terminated (it
 * arrives that way from MQTT) and `len` is its real length. Tolerates unknown
 * fields, so a newer publisher cannot break an older gateway -- subject to the
 * length ceiling documented on POS_JSON_BLINK_MAX_LEN above, which is a real
 * limit on that promise and not a formality.
 *
 * EVERY field is range-checked, `ts` included: it is rejected below 0, above
 * the DW3220's 40-bit domain (0xFFFFFFFFFF), and on strtoll() overflow. That
 * bound is the last line of defence for the datum this whole path exists to
 * carry -- an out-of-domain value reaching tdoa_solve() would be differenced
 * against a real timestamp and produce a confidently wrong position (or a NaN)
 * from a call that reported success.
 *
 * Returns 0 and fills `*out`, or -1 if a field is missing, one is out of range,
 * or `len` does not fit the internal buffer (POS_JSON_BLINK_MAX_LEN). */
int pos_json_blink_parse(const char *buf, size_t len, struct pos_blink_obs *out);

/* Buffer size that fits either document plus its NUL. Sized on the LARGER of the
 * two: a full APOS_MAX_NODES surveyed document (~150 bytes per anchor at the
 * widest coordinate and lat/long values), not the four-anchor stub it used to be
 * sized on. tests/pos_json/ asserts the worst case still fits. */
#define POS_JSON_MAX_LEN 1536

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
