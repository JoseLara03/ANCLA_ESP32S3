#include "pos_sink.h"
#include "uwb_frame_802_15_4z.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pos_sink, LOG_LEVEL_INF);

void pos_sink_publish(const struct pos_fix *fix)
{
    /* An unknown battery is reported as JSON null, not as 0 or 255: a consumer
     * must be able to tell "no reading" from "flat battery". */
    if (fix->batt_soc == UWB_FRAME_POS_SOC_UNKNOWN) {
        LOG_INF("{\"tag\":\"0x%04X\",\"x\":%.2f,\"y\":%.2f,"
                "\"residual\":%.3f,\"n\":%u,\"batt\":null}",
                fix->src_addr, (double)fix->x, (double)fix->y,
                (double)fix->residual_m, fix->n_anchors);
    } else {
        LOG_INF("{\"tag\":\"0x%04X\",\"x\":%.2f,\"y\":%.2f,"
                "\"residual\":%.3f,\"n\":%u,\"batt\":%u}",
                fix->src_addr, (double)fix->x, (double)fix->y,
                (double)fix->residual_m, fix->n_anchors, fix->batt_soc);
    }
}
