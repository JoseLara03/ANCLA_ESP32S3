#ifndef POS_SINK_H
#define POS_SINK_H

#include <stdint.h>

/* One decoded tag position report. */
struct pos_fix {
    uint16_t src_addr;    /* tag short address, from the frame header -- MAC
                           * layer only; reallocated across a rejoin, so this
                           * must never be published to the platform as Tid.
                           * See tag_id below. */
    uint32_t tag_id;      /* stable per-physical-tag id: tag_id_from_eui() of
                           * the seat's EUI, resolved by the caller (the
                           * gateway's dispatch path) before publish. This is
                           * what pos_json_fix() emits as Tid. */
    float    x;           /* metres */
    float    y;           /* metres */
    float    residual_m;  /* RMS range residual; larger means less trustworthy */
    uint8_t  n_anchors;   /* 3 or 4 */
    uint8_t  batt_soc;    /* 0-100, or UWB_FRAME_POS_SOC_UNKNOWN */
};

/* Consume one fix. In E1 this logs a JSON line on the console; E2 kept that
 * log line and added an MQTT publish (via net_uplink_submit()) to
 * testtopic/1/position. Called from the gateway's dispatch path, so it must
 * not block. */
void pos_sink_publish(const struct pos_fix *fix);

#endif /* POS_SINK_H */
