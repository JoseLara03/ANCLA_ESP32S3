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

#define UWB_ADDR_GATEWAY  0x0000u
#define UWB_ADDR_UNASSOC  0xFFFEu   /* tag src before it is granted a short addr */

#define UWB_PROTO_VER     2
#define UWB_FRAME_N_CFP   11        /* ranging slots per superframe (v2) */
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

/* batt_soc value meaning "no reading" — the charger is connected (terminal
 * voltage says nothing about charge then) or the gauge failed. Distinct from a
 * real 0 % reading. */
#define UWB_FRAME_POS_SOC_UNKNOWN 0xFFu
#define UWB_FRAME_LEN_POS        24

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
