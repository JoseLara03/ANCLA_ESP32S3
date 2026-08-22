#ifndef POS_SINK_H
#define POS_SINK_H

#include <stdbool.h>
#include <stdint.h>

/* One decoded tag position report. */
struct pos_fix {
    uint16_t src_addr;    /* tag short address, from the frame header */
    /* Stable tag identity: gw_core_tag_id() of the EUI-64 the gateway holds for
     * src_addr. This -- NOT src_addr -- is what goes out as "Tid". A short
     * address is a MAC lease that changes every time a tag's seat expires and
     * it re-JOINs; the platform keys its records on Tid and a changing one
     * makes the tag disappear from the map. See gw_core.h.
     *
     * tag_id_valid is false when the gateway could not resolve an EUI for
     * src_addr (no live seat -- a tag cannot range without one, so this means
     * the seat expired between the tag's last exchange and this frame, or the
     * gateway rebooted under a tag still using its old address). The fix is
     * still logged on the console; it is NOT published, because publishing a
     * fallback identity would put exactly the unstable id back on the topic
     * this field exists to keep stable. A separate flag rather than
     * `tag_id != 0` because 0 is a legal (if improbable) tag id. */
    uint32_t tag_id;
    bool     tag_id_valid;
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
