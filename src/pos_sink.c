#include "pos_sink.h"
#include "net_uplink.h"
#include "uwb_frame_802_15_4z.h"

#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pos_sink, LOG_LEVEL_INF);

/* Minimum gap between two "unresolved tag" lines, same idiom and reasoning as
 * anchor_respond.c's UNPOSITIONED_LOG_GAP_MS: a tag emits a fix per superframe,
 * so if this condition fires at all it fires continuously, and deferred logging
 * shares one buffer pool with every diagnostic an operator actually needs. */
#define UNRESOLVED_LOG_GAP_MS 10000

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
     * must be able to tell "no reading" from "flat battery".
     *
     * "tid" is here as well as on MQTT, and it is the only place the mapping
     * between the short address on the sniffer and the Tid on the platform is
     * visible. It is null when the address could not be resolved to an EUI-64
     * -- the same fixes the block at the end of this function refuses to
     * uplink. */
    char tid[12];

    if (fix->tag_id_valid) {
        snprintf(tid, sizeof(tid), "%u", fix->tag_id);
    } else {
        strcpy(tid, "null");
    }

    if (fix->batt_soc == UWB_FRAME_POS_SOC_UNKNOWN) {
        LOG_INF("{\"tag\":\"0x%04X\",\"tid\":%s,\"x\":%.2f,\"y\":%.2f,"
                "\"residual\":%.3f,\"n\":%u,\"batt\":null}",
                fix->src_addr, tid, (double)fix->x, (double)fix->y,
                (double)fix->residual_m, fix->n_anchors);
    } else {
        LOG_INF("{\"tag\":\"0x%04X\",\"tid\":%s,\"x\":%.2f,\"y\":%.2f,"
                "\"residual\":%.3f,\"n\":%u,\"batt\":%u}",
                fix->src_addr, tid, (double)fix->x, (double)fix->y,
                (double)fix->residual_m, fix->n_anchors, fix->batt_soc);
    }

    /* A fix the gateway cannot tie to an EUI-64 is logged above but never
     * published. "Tid" is the platform's primary key for a tag, and the only
     * value available here as a fallback is the short address -- i.e. precisely
     * the unstable, lease-scoped id that made the platform create a new record
     * every time a tag's seat expired. Publishing one bad fix would resurrect
     * that bug for one sample; dropping it costs one sample and the tag's next
     * JOIN restores the mapping.
     *
     * This should be rare: a tag cannot range without a seat, so the seat is
     * live whenever a fix is produced. It is reachable when the lease expires
     * in the window between the tag's last exchange and this frame, or when the
     * gateway reboots while tags keep transmitting under already-granted
     * addresses. */
    if (!fix->tag_id_valid) {
        static int64_t last_ms;
        static uint32_t suppressed;
        int64_t now = k_uptime_get();

        if (last_ms == 0 || now - last_ms >= UNRESOLVED_LOG_GAP_MS) {
            if (suppressed) {
                LOG_WRN("POS from 0x%04X: no seat holds this address, "
                        "so no EUI-64 to publish as Tid — fix not "
                        "uplinked (%u more since the last line)",
                        fix->src_addr, suppressed);
            } else {
                LOG_WRN("POS from 0x%04X: no seat holds this address, "
                        "so no EUI-64 to publish as Tid — fix not "
                        "uplinked",
                        fix->src_addr);
            }
            last_ms = now;
            suppressed = 0;
        } else {
            suppressed++;
        }
        return;
    }

    /* Non-blocking hand-off to the uplink thread. Safe on the dispatch path. */
    net_uplink_submit(fix);
}
