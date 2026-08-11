#include "uwb_frame_802_15_4z.h"
#include <errno.h>
#include <string.h>

/* NOTE: the DISC/MPOL/RESP frames use 2-byte little-endian short addresses at
 * bytes 5-6 (dest) and 7-8 (src). This is intentionally distinct from the legacy
 * SS-TWR E0/E1 frames in uwb_ss_initiator.c, whose bytes 5-8 are a 4-char ASCII
 * tag ('WAVE'/'VEWA'); the two frame families are not wire-compatible by design. */

/* ---- Field offsets ---- */
#define OFF_FC0        0
#define OFF_FC1        1
#define OFF_SEQ        2
#define OFF_PAN        3   /* bytes 3-4 */
#define OFF_DEST       5   /* bytes 5-6 */
#define OFF_SRC        7   /* bytes 7-8 */
#define OFF_TYPE       9
#define OFF_DISC_TS    10  /* bytes 10-13 */
#define OFF_RESP_TS    10  /* bytes 10-13 */
#define OFF_RESP_CIRP  14  /* bytes 14-17 */
#define OFF_RESP_CIRQ  18  /* bytes 18-19 */
#define OFF_MPOL_NUM   10
#define OFF_MPOL_SLOTS 11
#define OFF_POS_X      10  /* bytes 10-13 */
#define OFF_POS_Y      14  /* bytes 14-17 */
#define OFF_POS_RES    18  /* bytes 18-21 */
#define OFF_POS_NANCH  22
#define OFF_POS_SOC    23

/* ---- Little-endian field helpers ---- */
static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;  p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static uint32_t get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void put_f32(uint8_t *p, float v) {
    uint32_t u;
    memcpy(&u, &v, sizeof(u));
    put_u32(p, u);
}
static float get_f32(const uint8_t *p) {
    uint32_t u = get_u32(p);
    float v;
    memcpy(&v, &u, sizeof(v));
    return v;
}

/* Write the common 10-byte header. PANID is written as literal bytes 0xCA,0xDE
 * (matching existing tag frames), NOT little-endian-encoded. */
static int write_hdr(uint8_t *buf, size_t buf_len, size_t need,
                     uint16_t dest, uint16_t src, uint8_t type)
{
    if (!buf) return -EINVAL;
    if (buf_len < need) return -EMSGSIZE;
    buf[OFF_FC0] = 0x41;
    buf[OFF_FC1] = 0x88;
    buf[OFF_SEQ] = 0;           /* caller sets via uwb_frame_set_seq_num */
    buf[OFF_PAN] = 0xCA;
    buf[OFF_PAN + 1] = 0xDE;
    put_u16(&buf[OFF_DEST], dest);
    put_u16(&buf[OFF_SRC], src);
    buf[OFF_TYPE] = type;
    return 0;
}

/* ---- Builders ---- */
int uwb_frame_discovery_build(uint8_t *buf, size_t buf_len,
                              uint16_t src_addr, uint32_t tx_ts)
{
    int rc = write_hdr(buf, buf_len, UWB_FRAME_LEN_DISC,
                       UWB_FRAME_ADDR_BCAST, src_addr, UWB_FRAME_TYPE_DISC);
    if (rc) return rc;
    put_u32(&buf[OFF_DISC_TS], tx_ts);
    return UWB_FRAME_LEN_DISC;
}

int uwb_frame_multipoll_build(uint8_t *buf, size_t buf_len, uint16_t src_addr,
                              const struct uwb_anchor_slot *slots,
                              uint8_t num_slots, uint32_t tx_ts)
{
    if (!buf || !slots) {
        return -EINVAL;
    }
    if (num_slots == 0 || num_slots > UWB_FRAME_MAX_ANCHORS) {
        return -EINVAL;
    }
    size_t need = 15u + 4u * (size_t)num_slots;
    int rc = write_hdr(buf, buf_len, need,
                       UWB_FRAME_ADDR_BCAST, src_addr, UWB_FRAME_TYPE_MPOL);
    if (rc) return rc;
    buf[OFF_MPOL_NUM] = num_slots;
    size_t o = OFF_MPOL_SLOTS;
    for (uint8_t i = 0; i < num_slots; i++) {
        put_u16(&buf[o], slots[i].addr);
        put_u16(&buf[o + 2], slots[i].delay_us);
        o += 4;
    }
    put_u32(&buf[o], tx_ts);
    return (int)need;
}

int uwb_frame_response_build(uint8_t *buf, size_t buf_len, uint16_t src_addr,
                             uint16_t dest_addr, uint32_t tx_ts,
                             int32_t cir_power, uint16_t cir_quality)
{
    int rc = write_hdr(buf, buf_len, UWB_FRAME_LEN_RESP,
                       dest_addr, src_addr, UWB_FRAME_TYPE_RESP);
    if (rc) return rc;
    put_u32(&buf[OFF_RESP_TS], tx_ts);
    put_u32(&buf[OFF_RESP_CIRP], (uint32_t)cir_power);
    put_u16(&buf[OFF_RESP_CIRQ], cir_quality);
    return UWB_FRAME_LEN_RESP;
}

/* ---- Parsers ---- */
int uwb_frame_parse_discovery_response(const uint8_t *buf, size_t len,
                                       uint16_t *src_addr, int32_t *cir_power,
                                       uint16_t *cir_quality)
{
    if (!buf || !src_addr || !cir_power || !cir_quality) {
        return -EINVAL;
    }
    if (!uwb_frame_is_response(buf, len)) {
        return -EBADMSG;
    }
    *src_addr    = get_u16(&buf[OFF_SRC]);
    *cir_power   = (int32_t)get_u32(&buf[OFF_RESP_CIRP]);
    *cir_quality = get_u16(&buf[OFF_RESP_CIRQ]);
    return 0;
}

int uwb_frame_parse_multipoll(const uint8_t *buf, size_t len,
                              struct uwb_anchor_slot *slots_out,
                              uint8_t *num_slots, uint32_t *tx_ts)
{
    if (!buf || !slots_out || !num_slots || !tx_ts) {
        return -EINVAL;
    }
    if (!uwb_frame_is_multipoll(buf, len)) {
        return -EBADMSG;
    }
    uint8_t n = buf[OFF_MPOL_NUM];
    size_t o = OFF_MPOL_SLOTS;
    for (uint8_t i = 0; i < n; i++) {
        slots_out[i].addr     = get_u16(&buf[o]);
        slots_out[i].delay_us = get_u16(&buf[o + 2]);
        o += 4;
    }
    *tx_ts = get_u32(&buf[o]);
    *num_slots = n;
    return 0;
}

/* ---- Validators ---- */
bool uwb_frame_is_valid(const uint8_t *buf, size_t len)
{
    if (!buf) {
        return false;
    }
    if (len < UWB_FRAME_HDR_LEN || len > UWB_FRAME_MAX_LEN) {
        return false;
    }
    if (buf[OFF_FC0] != 0x41 || buf[OFF_FC1] != 0x88) {
        return false;
    }
    if (buf[OFF_PAN] != 0xCA || buf[OFF_PAN + 1] != 0xDE) {
        return false;
    }
    return true;
}

bool uwb_frame_is_discovery(const uint8_t *buf, size_t len)
{
    return uwb_frame_is_valid(buf, len) &&
           buf[OFF_TYPE] == UWB_FRAME_TYPE_DISC &&
           len >= UWB_FRAME_LEN_DISC;
}

bool uwb_frame_is_multipoll(const uint8_t *buf, size_t len)
{
    if (!uwb_frame_is_valid(buf, len)) {
        return false;
    }
    if (buf[OFF_TYPE] != UWB_FRAME_TYPE_MPOL) {
        return false;
    }
    if (len < OFF_MPOL_SLOTS + 1) {   /* must be able to read num_anchors */
        return false;
    }
    uint8_t n = buf[OFF_MPOL_NUM];
    if (n == 0 || n > UWB_FRAME_MAX_ANCHORS) {
        return false;
    }
    return len >= (size_t)(15u + 4u * n);
}

bool uwb_frame_is_response(const uint8_t *buf, size_t len)
{
    return uwb_frame_is_valid(buf, len) &&
           buf[OFF_TYPE] == UWB_FRAME_TYPE_RESP &&
           len >= UWB_FRAME_LEN_RESP;
}

/* ---- Utilities ---- */
uint16_t uwb_frame_get_src_addr(const uint8_t *buf)  { return get_u16(&buf[OFF_SRC]); }
uint16_t uwb_frame_get_dest_addr(const uint8_t *buf) { return get_u16(&buf[OFF_DEST]); }
uint8_t  uwb_frame_get_seq_num(const uint8_t *buf)   { return buf[OFF_SEQ]; }
void     uwb_frame_set_seq_num(uint8_t *buf, uint8_t seq) { buf[OFF_SEQ] = seq; }

/* ---- BEACON frame support ---- */

int uwb_frame_beacon_build(uint8_t *buf, size_t buf_len, uint32_t frame_counter,
                           const uint16_t *slot_map, uint8_t n_slots)
{
    if (!slot_map) return -EINVAL;
    size_t need = 15u + 2u * (size_t)n_slots;
    int rc = write_hdr(buf, buf_len, need, UWB_FRAME_ADDR_BCAST,
                       UWB_ADDR_GATEWAY, UWB_FRAME_TYPE_BEACON);
    if (rc) return rc;
    buf[10] = UWB_PROTO_VER;
    put_u32(&buf[11], frame_counter);
    for (uint8_t i = 0; i < n_slots; i++) put_u16(&buf[15 + 2 * i], slot_map[i]);
    return (int)need;
}

bool uwb_frame_is_beacon(const uint8_t *buf, size_t len)
{
    return len >= 15 && uwb_frame_is_valid(buf, len) && buf[OFF_TYPE] == UWB_FRAME_TYPE_BEACON;
}

int uwb_frame_parse_beacon(const uint8_t *buf, size_t len, uint8_t *proto_ver,
                           uint32_t *frame_counter, uint16_t *slot_map_out, uint8_t *n_slots)
{
    if (!uwb_frame_is_beacon(buf, len)) return -EINVAL;
    if (((len - 15) % 2) != 0) return -EINVAL;
    uint8_t ns = (uint8_t)((len - 15) / 2);
    if (proto_ver) *proto_ver = buf[10];
    if (frame_counter) *frame_counter = get_u32(&buf[11]);
    if (slot_map_out) for (uint8_t i = 0; i < ns; i++) slot_map_out[i] = get_u16(&buf[15 + 2 * i]);
    if (n_slots) *n_slots = ns;
    return 0;
}

int uwb_frame_beacon_find_addr(const uint16_t *slot_map, uint8_t n_slots, uint16_t addr)
{
    if (!slot_map) return -1;
    for (uint8_t i = 0; i < n_slots; i++) if (slot_map[i] == addr) return (int)i;
    return -1;
}

/* ---- JOIN_REQ frame support ---- */

int uwb_frame_join_build(uint8_t *buf, size_t buf_len, const uint8_t eui[8], uint8_t req_tier)
{
    int rc = write_hdr(buf, buf_len, UWB_FRAME_LEN_JOIN, UWB_ADDR_GATEWAY,
                       UWB_ADDR_UNASSOC, UWB_FRAME_TYPE_JOIN);
    if (rc) return rc;
    if (!eui) return -EINVAL;
    memcpy(&buf[10], eui, UWB_FRAME_EUI_LEN);
    buf[18] = req_tier;
    return UWB_FRAME_LEN_JOIN;
}

bool uwb_frame_is_join(const uint8_t *buf, size_t len)
{
    return len == UWB_FRAME_LEN_JOIN && uwb_frame_is_valid(buf, len) &&
           buf[OFF_TYPE] == UWB_FRAME_TYPE_JOIN;
}

int uwb_frame_parse_join(const uint8_t *buf, size_t len, uint8_t eui_out[8], uint8_t *req_tier)
{
    if (!uwb_frame_is_join(buf, len)) return -EINVAL;
    if (eui_out) memcpy(eui_out, &buf[10], UWB_FRAME_EUI_LEN);
    if (req_tier) *req_tier = buf[18];
    return 0;
}

/* ---- GRANT frame support ---- */

int uwb_frame_grant_build(uint8_t *buf, size_t buf_len, const uint8_t eui[8],
                          uint16_t short_addr, uint8_t slot_index, uint8_t rate_tier, uint16_t lease)
{
    int rc = write_hdr(buf, buf_len, UWB_FRAME_LEN_GRANT, UWB_FRAME_ADDR_BCAST,
                       UWB_ADDR_GATEWAY, UWB_FRAME_TYPE_GRANT);
    if (rc) return rc;
    if (!eui) return -EINVAL;
    memcpy(&buf[10], eui, UWB_FRAME_EUI_LEN);
    put_u16(&buf[18], short_addr);
    buf[20] = slot_index;
    buf[21] = rate_tier;
    put_u16(&buf[22], lease);
    return UWB_FRAME_LEN_GRANT;
}

bool uwb_frame_is_grant(const uint8_t *buf, size_t len)
{
    return len == UWB_FRAME_LEN_GRANT && uwb_frame_is_valid(buf, len) &&
           buf[OFF_TYPE] == UWB_FRAME_TYPE_GRANT;
}

int uwb_frame_parse_grant(const uint8_t *buf, size_t len, uint8_t eui_out[8],
                          uint16_t *short_addr, uint8_t *slot_index, uint8_t *rate_tier, uint16_t *lease)
{
    if (!uwb_frame_is_grant(buf, len)) return -EINVAL;
    if (eui_out)   memcpy(eui_out, &buf[10], UWB_FRAME_EUI_LEN);
    if (short_addr) *short_addr = get_u16(&buf[18]);
    if (slot_index) *slot_index = buf[20];
    if (rate_tier)  *rate_tier = buf[21];
    if (lease)      *lease = get_u16(&buf[22]);
    return 0;
}

/* ---- KEEPALIVE and RELEASE frame support ---- */

int uwb_frame_keepalive_build(uint8_t *buf, size_t buf_len, uint16_t src_addr,
                              uint8_t req_tier, uint8_t slot_index)
{
    int rc = write_hdr(buf, buf_len, UWB_FRAME_LEN_KEEPALIVE, UWB_ADDR_GATEWAY,
                       src_addr, UWB_FRAME_TYPE_KEEPALIVE);
    if (rc) return rc;
    buf[10] = req_tier;
    buf[11] = slot_index;
    return UWB_FRAME_LEN_KEEPALIVE;
}

bool uwb_frame_is_keepalive(const uint8_t *buf, size_t len)
{
    return len == UWB_FRAME_LEN_KEEPALIVE && uwb_frame_is_valid(buf, len) &&
           buf[OFF_TYPE] == UWB_FRAME_TYPE_KEEPALIVE;
}

int uwb_frame_parse_keepalive(const uint8_t *buf, size_t len, uint16_t *src_addr,
                              uint8_t *req_tier, uint8_t *slot_index)
{
    if (!uwb_frame_is_keepalive(buf, len)) return -EINVAL;
    if (src_addr)   *src_addr = uwb_frame_get_src_addr(buf);
    if (req_tier)   *req_tier = buf[10];
    if (slot_index) *slot_index = buf[11];
    return 0;
}

int uwb_frame_release_build(uint8_t *buf, size_t buf_len, uint16_t src_addr)
{
    int rc = write_hdr(buf, buf_len, UWB_FRAME_LEN_RELEASE, UWB_ADDR_GATEWAY,
                       src_addr, UWB_FRAME_TYPE_RELEASE);
    if (rc) return rc;
    return UWB_FRAME_LEN_RELEASE;
}

bool uwb_frame_is_release(const uint8_t *buf, size_t len)
{
    return len == UWB_FRAME_LEN_RELEASE && uwb_frame_is_valid(buf, len) &&
           buf[OFF_TYPE] == UWB_FRAME_TYPE_RELEASE;
}

/* ---- POS frame support ---- */

int uwb_frame_pos_build(uint8_t *buf, size_t buf_len, uint16_t src_addr,
                        float x, float y, float residual_m,
                        uint8_t n_anchors, uint8_t batt_soc)
{
    int rc = write_hdr(buf, buf_len, UWB_FRAME_LEN_POS, UWB_ADDR_GATEWAY,
                       src_addr, UWB_FRAME_TYPE_POS);
    if (rc) return rc;
    put_f32(&buf[OFF_POS_X],   x);
    put_f32(&buf[OFF_POS_Y],   y);
    put_f32(&buf[OFF_POS_RES], residual_m);
    buf[OFF_POS_NANCH] = n_anchors;
    buf[OFF_POS_SOC]   = batt_soc;
    return UWB_FRAME_LEN_POS;
}

bool uwb_frame_is_pos(const uint8_t *buf, size_t len)
{
    return len == UWB_FRAME_LEN_POS && uwb_frame_is_valid(buf, len) &&
           buf[OFF_TYPE] == UWB_FRAME_TYPE_POS;
}

int uwb_frame_parse_pos(const uint8_t *buf, size_t len, uint16_t *src_addr,
                        float *x, float *y, float *residual_m,
                        uint8_t *n_anchors, uint8_t *batt_soc)
{
    if (!uwb_frame_is_pos(buf, len)) return -EINVAL;
    if (src_addr)   *src_addr   = uwb_frame_get_src_addr(buf);
    if (x)          *x          = get_f32(&buf[OFF_POS_X]);
    if (y)          *y          = get_f32(&buf[OFF_POS_Y]);
    if (residual_m) *residual_m = get_f32(&buf[OFF_POS_RES]);
    if (n_anchors)  *n_anchors  = buf[OFF_POS_NANCH];
    if (batt_soc)   *batt_soc   = buf[OFF_POS_SOC];
    return 0;
}
