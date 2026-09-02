#ifndef UWB_FRAME_802_15_4Z_H
#define UWB_FRAME_802_15_4Z_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ---- Message-type tags (byte 9), continuing the existing 0xE0/0xE1 series ---- */
#define UWB_FRAME_TYPE_DISC   0xE2
#define UWB_FRAME_TYPE_MPOL   0xE3
#define UWB_FRAME_TYPE_RESP   0xE4

/* ---- TDMA MAC message types (contract v1) ---- */
#define UWB_FRAME_TYPE_BEACON    0xE5
#define UWB_FRAME_TYPE_JOIN      0xE6
#define UWB_FRAME_TYPE_GRANT     0xE7
#define UWB_FRAME_TYPE_KEEPALIVE 0xE8
#define UWB_FRAME_TYPE_RELEASE   0xE9
#define UWB_FRAME_TYPE_POS       0xEA
/* Moved from 0xEB on 2026-08-25. 0xEB was DOUBLE-ALLOCATED: the anchor project
 * had been using it as APOS_FRAME_TYPE for the auto-positioning survey since
 * before ALERT existed, and the collision was exact rather than approximate --
 * APOS_LEN_ENUM_RSP is 34 bytes, the same as UWB_FRAME_LEN_ALERT, with the
 * discriminating byte at the same offset 10 (ALERT's `state`, apos's subtype).
 *
 * It had not bitten yet, but only via two single-point saves: an ENUM_RSP
 * reaching a tag was rejected because subtype 0x02 trips
 * UWB_ALERT_STATE_RESERVED_MASK by one bit, and a HELP alert reaching an anchor
 * survived only because state 0x01 is exactly APOS_SUB_SURVEY_BEGIN and the two
 * frames differ in length. Any new 34-byte apos subtype with bit 1 clear, or any
 * change to UWB_FRAME_LEN_ALERT, turns that into silent cross-talk between two
 * unrelated functions.
 *
 * ALERT moved rather than apos because apos already has seven subtypes, its own
 * codec and its own host tests, whereas ALERT is a single type. 0xEC and 0xED
 * are reserved for CONFIG_SET / CONFIG_ACK (design spec section 4.4). */
#define UWB_FRAME_TYPE_ALERT     0xEE

#define UWB_ADDR_GATEWAY  0x0000u
#define UWB_ADDR_UNASSOC  0xFFFEu   /* tag src before it is granted a short addr */

/* Bumped 2 -> 3 on 2026-08-25. Two changes ride on it, neither of which alters
 * a frame LAYOUT -- which is why the version, not the length, is what protects
 * them:
 *
 *   1. The beacon's slot map is now a SCHEDULE ("who transmits this
 *      superframe") rather than an ownership table. A v2 tag reads absence from
 *      the map as a reclaimed seat and tears down, so it would thrash against a
 *      v3 gateway that time-multiplexes slots.
 *   2. GRANT byte 20 carries a SEAT id rather than a CFP slot index, and seat
 *      ids exceed N_CFP.
 *
 * A v2 tag against a v3 gateway therefore stays deaf in SCAN, which is the
 * correct loud failure rather than a silent misbehaviour -- but it does make
 * this a FLAG DAY: both firmwares are ours and both must be reflashed together.
 * ALERT's move to 0xEE is grouped here for the same reason.
 *
 * Bumped 3 -> 4 on 2026-08-31 for Task 4B, the BLINK slotted MAC (see
 * docs/superpowers/specs/2026-08-30-blink-slotted-mac-design.md section 1.3).
 * Again, the frame LAYOUT is unchanged -- UWB_FRAME_LEN_BEACON is still
 * `15 + 2 * UWB_FRAME_N_CFP` and `sched[]` still carries UWB_FRAME_N_CFP
 * entries -- but a v4 gateway running its cell in BLINK mode transmits
 * `sched[]` reserved/zero and assigns airtime by `blink_sched_slot_index()`
 * (identity on seat_id) instead. Same class of change as 2 -> 3: the
 * MEANING of an unchanged wire format changes, so the version has to move
 * even though nothing byte-visible does. A v3 tag against a v4 gateway
 * stays deaf in SCAN -- same flag day, both firmwares reflashed together. */
#define UWB_PROTO_VER     5
#define UWB_FRAME_N_CFP   11        /* ranging slots per superframe */
#define UWB_FRAME_N_CAP   4         /* CAP Aloha mini-slots (v1) */

#define UWB_FRAME_PANID       0xCADE  /* written as literal bytes 0xCA,0xDE */
#define UWB_FRAME_ADDR_BCAST  0xFFFF
#define UWB_FRAME_MAX_ANCHORS 4

#define UWB_FRAME_HDR_LEN     10  /* bytes 0-9 common header */
#define UWB_FRAME_LEN_DISC    14
#define UWB_FRAME_LEN_RESP    20
#define UWB_FRAME_MAX_LEN     37  /* BEACON with 11 slots, excl. FCS */

/* Multi-poll is variable length: header(10) + num(1) + n*(addr2+delay2) + ts(4). */
#define UWB_FRAME_LEN_MPOL(n)  (15 + 4 * (n))

#define UWB_FRAME_LEN_BEACON     (15 + 2 * UWB_FRAME_N_CFP)
#define UWB_FRAME_LEN_JOIN       19
#define UWB_FRAME_LEN_GRANT      24
#define UWB_FRAME_LEN_KEEPALIVE  12
#define UWB_FRAME_LEN_RELEASE    10
#define UWB_FRAME_EUI_LEN        8

/* batt_soc sentinel: NO PERCENTAGE READING IS AVAILABLE. Distinct from a real
 * 0 % reading, which is what the value exists to keep separable.
 *
 * The name says "connected" because the charger being attached is the case
 * that produces it in normal use -- terminal voltage says nothing about state
 * of charge while charging, so the gauge reports nothing. But the value is
 * ALSO what a failed gauge yields, and the tag does not distinguish the two.
 * So "connected" is the dominant cause, not a guarantee: do not build logic
 * that treats this value as proof a charger is present.
 *
 * Renamed from UWB_FRAME_POS_SOC_UNKNOWN on 2026-09-03. */
#define UWB_FRAME_POS_SOC_CONNECTED 0xFFu
#define UWB_FRAME_LEN_POS        24

/* ---- ALERT (0xEE): HELP/CANCEL, tag/anchor -> gateway --------------------
 * See spec/2026-08-16-uwb-help-alert-design.md §2/§3 for the field
 * semantics (epoch/repeat_seq/ttl/sender_hop ordering and the reserved
 * `state` bits). UWB_FRAME_LEN_ALERT (34) <= UWB_FRAME_MAX_LEN (37). */
#define UWB_ALERT_STATE_CANCEL 0x00u
#define UWB_ALERT_STATE_HELP   0x01u
#define UWB_ALERT_STATE_RESERVED_MASK 0xFEu   /* bits1-7, must be 0 */
#define UWB_ALERT_HOP_UNKNOWN  0xFFu
#define UWB_ALERT_TTL_INIT     6
#define UWB_FRAME_LEN_ALERT    34

struct uwb_alert {
    uint8_t  state;        /* bit0 HELP/CANCEL; bits1-7 reserved, must be 0 */
    uint8_t  epoch;
    uint8_t  repeat_seq;
    uint8_t  sender_hop;
    uint8_t  ttl;
    uint8_t  orig_eui[8];
    uint16_t orig_addr;
    uint8_t  batt_soc;
    float    last_x, last_y;   /* NaN when the tag has no fix */
};

struct uwb_anchor_slot {
    uint16_t addr;      /* anchor short address */
    uint16_t delay_us;  /* staggered response delay, microseconds */
};

/* ---- Builders: return bytes written (excl. FCS), or negative errno ---- */
int uwb_frame_discovery_build(uint8_t *buf, size_t buf_len,
                              uint16_t src_addr, uint32_t tx_ts);
int uwb_frame_multipoll_build(uint8_t *buf, size_t buf_len,
                              uint16_t src_addr,
                              const struct uwb_anchor_slot *slots,
                              uint8_t num_slots, uint32_t tx_ts);
int uwb_frame_response_build(uint8_t *buf, size_t buf_len,
                             uint16_t src_addr, uint16_t dest_addr,
                             uint32_t tx_ts, int32_t cir_power,
                             uint16_t cir_quality);

/* ---- Parsers: return 0 on success, negative errno on bad frame ---- */
int uwb_frame_parse_discovery_response(const uint8_t *buf, size_t len,
                                       uint16_t *src_addr,
                                       int32_t *cir_power,
                                       uint16_t *cir_quality);
int uwb_frame_parse_multipoll(const uint8_t *buf, size_t len,
                              struct uwb_anchor_slot *slots_out,
                              uint8_t *num_slots, uint32_t *tx_ts);

/* ---- BEACON frame builders and parsers ---- */
int  uwb_frame_beacon_build(uint8_t *buf, size_t buf_len, uint32_t frame_counter,
                            const uint16_t *slot_map, uint8_t n_slots);
int  uwb_frame_parse_beacon(const uint8_t *buf, size_t len, uint8_t *proto_ver,
                            uint32_t *frame_counter, uint16_t *slot_map_out,
                            uint8_t *n_slots);
bool uwb_frame_is_beacon(const uint8_t *buf, size_t len);
int  uwb_frame_beacon_find_addr(const uint16_t *slot_map, uint8_t n_slots, uint16_t addr);

/* ---- JOIN_REQ frame builders and parsers ---- */
int uwb_frame_join_build(uint8_t *buf, size_t buf_len, const uint8_t eui[8], uint8_t req_tier);
int uwb_frame_parse_join(const uint8_t *buf, size_t len, uint8_t eui_out[8], uint8_t *req_tier);
bool uwb_frame_is_join(const uint8_t *buf, size_t len);

/* ---- GRANT frame builders and parsers ---- */
int uwb_frame_grant_build(uint8_t *buf, size_t buf_len, const uint8_t eui[8],
                          uint16_t short_addr, uint8_t slot_index, uint8_t rate_tier, uint16_t lease);
int uwb_frame_parse_grant(const uint8_t *buf, size_t len, uint8_t eui_out[8],
                          uint16_t *short_addr, uint8_t *slot_index, uint8_t *rate_tier, uint16_t *lease);
bool uwb_frame_is_grant(const uint8_t *buf, size_t len);

/* ---- KEEPALIVE and RELEASE frame builders and parsers ---- */
int uwb_frame_keepalive_build(uint8_t *buf, size_t buf_len, uint16_t src_addr,
                              uint8_t req_tier, uint8_t slot_index);
int uwb_frame_parse_keepalive(const uint8_t *buf, size_t len, uint16_t *src_addr,
                              uint8_t *req_tier, uint8_t *slot_index);
bool uwb_frame_is_keepalive(const uint8_t *buf, size_t len);
int uwb_frame_release_build(uint8_t *buf, size_t buf_len, uint16_t src_addr);
bool uwb_frame_is_release(const uint8_t *buf, size_t len);

/* ---- POS (0xEA): tag position report, tag -> gateway ---- */
int  uwb_frame_pos_build(uint8_t *buf, size_t buf_len, uint16_t src_addr,
                         float x, float y, float residual_m,
                         uint8_t n_anchors, uint8_t batt_soc);
bool uwb_frame_is_pos(const uint8_t *buf, size_t len);
int  uwb_frame_parse_pos(const uint8_t *buf, size_t len, uint16_t *src_addr,
                         float *x, float *y, float *residual_m,
                         uint8_t *n_anchors, uint8_t *batt_soc);

/* ---- ALERT (0xEE) builder / parser / validator ---- */
int  uwb_frame_alert_build(uint8_t *buf, size_t buf_len, uint16_t src_addr,
                           const struct uwb_alert *a);
bool uwb_frame_is_alert(const uint8_t *buf, size_t len);
int  uwb_frame_parse_alert(const uint8_t *buf, size_t len, struct uwb_alert *a);

/* ---- Validators ---- */
bool uwb_frame_is_valid(const uint8_t *buf, size_t len);
bool uwb_frame_is_discovery(const uint8_t *buf, size_t len);
bool uwb_frame_is_multipoll(const uint8_t *buf, size_t len);
bool uwb_frame_is_response(const uint8_t *buf, size_t len);

/* ---- Utilities ---- */
uint16_t uwb_frame_get_src_addr(const uint8_t *buf);
uint16_t uwb_frame_get_dest_addr(const uint8_t *buf);
uint8_t  uwb_frame_get_seq_num(const uint8_t *buf);
void     uwb_frame_set_seq_num(uint8_t *buf, uint8_t seq);

#endif /* UWB_FRAME_802_15_4Z_H */
