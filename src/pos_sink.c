#include "pos_sink.h"
#include "net_uplink.h"
#include "uwb_frame_802_15_4z.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pos_sink, LOG_LEVEL_INF);

void pos_sink_publish(const struct pos_fix *fix)
{
    /* Defensive: n_anchors is documented (pos_sink.h) as 3 or 4. A corrupt or
     * malicious frame could carry anything in that byte, and publishing it
     * unchecked would emit a bogus "n" into the JSON telemetry stream.
     * Rejected here so a bad frame never occupies a queue slot either. */
    if (fix->n_anchors < 3 || fix->n_anchors > UWB_FRAME_MAX_ANCHORS) {
        LOG_WRN("POS from 0x%04X: n_anchors=%u out of range, dropping",
                fix->src_addr, fix->n_anchors);
        return;
    }

    /* The console line stays. It is the ONLY place residual, n_anchors and
     * batt_soc remain visible -- the MQTT payload carries none of them (see
     * pos_json.c) -- and it is what makes the gateway debuggable over USB with
     * no broker present.
     *
     * An unknown battery is reported as JSON null, not as 0 or 255: a consumer
     * must be able to tell "no reading" from "flat battery". */
    if (fix->batt_soc == UWB_FRAME_POS_SOC_UNKNOWN) {
        LOG_INF("{\"tag\":\"0x%04X\",\"tid\":%u,\"x\":%.2f,\"y\":%.2f,"
                "\"residual\":%.3f,\"n\":%u,\"batt\":null}",
                fix->src_addr, fix->tag_id, (double)fix->x, (double)fix->y,
                (double)fix->residual_m, fix->n_anchors);
    } else {
        LOG_INF("{\"tag\":\"0x%04X\",\"tid\":%u,\"x\":%.2f,\"y\":%.2f,"
                "\"residual\":%.3f,\"n\":%u,\"batt\":%u}",
                fix->src_addr, fix->tag_id, (double)fix->x, (double)fix->y,
                (double)fix->residual_m, fix->n_anchors, fix->batt_soc);
    }

    /* Non-blocking hand-off to the uplink thread. Safe on the dispatch path. */
    net_uplink_submit(fix);
}
