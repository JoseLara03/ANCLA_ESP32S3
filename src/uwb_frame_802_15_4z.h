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
#define UWB_FRAME_TYPE_ANNOUNCE  0xEC
#define UWB_FRAME_TYPE_MPOL_RESP 0xED
/* Moved from 0xEB. 0xEB was DOUBLE-ALLOCATED: the anchor project has used it as
 * APOS_FRAME_TYPE for the auto-positioning survey since before ALERT existed,
 * and the collision was exact rather than approximate -- APOS_LEN_ENUM_RSP is
 * 34 bytes, the same as UWB_FRAME_LEN_ALERT, with the discriminating byte at
 * the same offset 10 (ALERT's `state`, apos's subtype).
 *
 * It never bit, but only via two single-point saves: an apos ENUM_RSP reaching
 * a tag was rejected because subtype 0x02 trips UWB_ALERT_STATE_RESERVED_MASK
 * by one bit, and a HELP alert reaching an anchor survived only because state
 * 0x01 is exactly APOS_SUB_SURVEY_BEGIN and the two frames differ in length.
 * Any new 34-byte apos subtype with bit 1 clear, or any change to
 * UWB_FRAME_LEN_ALERT, turns that into silent cross-talk between two unrelated
 * functions -- and the gateway's dispatch() tests apos FIRST, so an ALERT would
 * have been swallowed by the survey handler.
 *
 * ALERT moved rather than apos because apos already has seven subtypes, its own
 * codec and its own host tests, whereas ALERT is a single type whose relay is
 * not implemented on the anchor side at all.
 *
 * UWB_PROTO_VER is deliberately NOT bumped for this: it is already 3 on this
 * branch, and no ALERT has ever been transmitted by either firmware (bench
 * captures show no 0xEB from any node), so no peer can be relying on the old
 * code. Both repos still have to be reflashed together, as always for a shared
 * wire constant. */
#define UWB_FRAME_TYPE_ALERT     0xEE

#define UWB_ADDR_GATEWAY  0x0000u
#define UWB_ADDR_UNASSOC  0xFFFEu   /* tag src before it is granted a short addr */

#define UWB_PROTO_VER     3  /* v3: network-scaling (grouped discovery, ANNOUNCE,
                               * MULTI-POLL, phase-mask GRANT). Bumped once every
                               * frame change has landed on both tag and anchor. */
#define UWB_FRAME_N_CFP   11        /* ranging slots per superframe (v2) */
#define UWB_FRAME_N_CAP   4         /* CAP Aloha mini-slots (v1) */

#define UWB_FRAME_PANID       0xCADE  /* written as literal bytes 0xCA,0xDE */
#define UWB_FRAME_ADDR_BCAST  0xFFFF
#define UWB_FRAME_MAX_ANCHORS 4

#define UWB_FRAME_HDR_LEN     10  /* bytes 0-9 common header */
#define UWB_FRAME_LEN_DISC    16
#define UWB_FRAME_LEN_RESP    20

/* Grouped DISCOVERY: a round covers one group of the anchor population so the
 * collection window stays bounded as the anchor count grows past 4. Value
 * tied to ceil(UWB_MAX_ANCHORS / 4) on the anchor side -- 32 anchors, 4 per
 * group, 8 groups. */
#define UWB_FRAME_DISC_N_GROUPS_MAX 8
#define UWB_FRAME_MAX_LEN     37  /* BEACON with 11 slots, excl. FCS */

/* Multi-poll is variable length: header(10) + num(1) + n*(addr2+delay2) + ts(4). */
#define UWB_FRAME_LEN_MPOL(n)  (15 + 4 * (n))

#define UWB_FRAME_LEN_BEACON     (15 + 2 * UWB_FRAME_N_CFP)
#define UWB_FRAME_LEN_JOIN       19
#define UWB_FRAME_LEN_GRANT      26  /* v3: +2 bytes, phase_mask (network-scaling Phase 3) */
#define UWB_FRAME_LEN_KEEPALIVE  12
#define UWB_FRAME_LEN_RELEASE    10
#define UWB_FRAME_EUI_LEN        8

/* batt_soc value meaning "no reading" — the charger is connected (terminal
 * voltage says nothing about charge then) or the gauge failed. Distinct from a
 * real 0 % reading. */
#define UWB_FRAME_POS_SOC_UNKNOWN 0xFFu
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

/* ---- ANNOUNCE (0xEC): passive anchor beacon-followup, anchor -> tag -----
 * Broadcast in the T_ANNOUNCE_MS window right after a beacon; lets a tag
 * learn anchor position/quality without running discovery. UWB_FRAME_LEN_ANNOUNCE
 * (30) <= UWB_FRAME_MAX_LEN (37). z may be NaN (anchor has no height set). */
#define UWB_FRAME_LEN_ANNOUNCE 30

struct uwb_announce {
    uint16_t addr;
    float    x, y, z;      /* z may be NaN: anchor did not report a height */
    int32_t  cir_power;
    uint16_t cir_quality;
};

/* ---- MPOL_RESP (0xED): addressed multi-poll ranging response, anchor -> tag
 * See spec/2026-09-07-network-scaling-design.md §3.D for the byte layout.
 * dest = the polling tag's short address, src = the anchor's -- a real
 * header, unlike the legacy WAVE frames. z may be NaN (anchor has no height
 * configured); pos_solver falls back to pos_cfg's single dz in that case. */
#define UWB_FRAME_LEN_MPOL_RESP 31

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
                              uint16_t src_addr, uint32_t tx_ts,
                              uint8_t group, uint8_t n_groups);
int uwb_frame_multipoll_build(uint8_t *buf, size_t buf_len,
                              uint16_t src_addr,
                              const struct uwb_anchor_slot *slots,
                              uint8_t num_slots, uint32_t tx_ts);
int uwb_frame_response_build(uint8_t *buf, size_t buf_len,
                             uint16_t src_addr, uint16_t dest_addr,
                             uint32_t tx_ts, int32_t cir_power,
                             uint16_t cir_quality);

/* ---- Parsers: return 0 on success, negative errno on bad frame ---- */
int uwb_frame_parse_discovery(const uint8_t *buf, size_t len,
                              uint16_t *src_addr, uint32_t *tx_ts,
                              uint8_t *group, uint8_t *n_groups);
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

/* ---- GRANT frame builders and parsers ----
 * phase_mask (v3): bits over a UWB_NET_CYCLE_C-superframe cycle (uwb_net.h)
 * naming which superframes this tag actually participates in -- a grant is
 * (slot, phase_mask), not a permanent every-superframe seat. */
int uwb_frame_grant_build(uint8_t *buf, size_t buf_len, const uint8_t eui[8],
                          uint16_t short_addr, uint8_t slot_index, uint8_t rate_tier,
                          uint16_t lease, uint16_t phase_mask);
int uwb_frame_parse_grant(const uint8_t *buf, size_t len, uint8_t eui_out[8],
                          uint16_t *short_addr, uint8_t *slot_index, uint8_t *rate_tier,
                          uint16_t *lease, uint16_t *phase_mask);
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

/* ---- ANNOUNCE (0xEC) builder (anchor) / parser (tag) / validator ---- */
int  uwb_frame_announce_build(uint8_t *buf, size_t buf_len, uint16_t src_addr,
                              const struct uwb_announce *a);
bool uwb_frame_is_announce(const uint8_t *buf, size_t len);
int  uwb_frame_parse_announce(const uint8_t *buf, size_t len, struct uwb_announce *a);

/* ---- MPOL_RESP (0xED) builder (anchor) / parser (tag) / validator ---- */
int  uwb_frame_mpol_resp_build(uint8_t *buf, size_t buf_len, uint16_t src_addr,
                               uint16_t dest_addr, uint8_t anchor_id,
                               uint32_t poll_rx_ts, uint32_t resp_tx_ts,
                               float x, float y, float z);
bool uwb_frame_is_mpol_resp(const uint8_t *buf, size_t len);
/* dest_addr: the tag's own short address, checked against the frame's dest
 * field -- rejects a response that was actually addressed to another tag. */
int  uwb_frame_parse_mpol_resp(const uint8_t *buf, size_t len, uint16_t dest_addr,
                               uint16_t *src_addr, uint8_t *anchor_id,
                               uint32_t *poll_rx_ts, uint32_t *resp_tx_ts,
                               float *x, float *y, float *z);

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
