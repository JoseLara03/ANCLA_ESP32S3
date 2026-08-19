/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Ranging-path diagnostics, gated on CONFIG_ANCLA_RANGING_DEBUG (debug.conf).
 *
 * Why a build flag rather than just raising CONFIG_LOG_DEFAULT_LEVEL: every
 * module in the ranging path registers itself at LOG_LEVEL_INF explicitly, and
 * a per-module level HARD-CAPS the module regardless of the Kconfig default --
 * so the LOG_DBG lines in anchor_respond.c and uwb_slave.c (the beacon-guard
 * suppression verdict, the per-beacon lock state) cannot be turned on from a
 * .conf overlay at all. They are compiled out. Routing those registrations
 * through ANCLA_LOG_LEVEL is what makes them reachable.
 *
 * The production image is unaffected: with the flag off this resolves to
 * LOG_LEVEL_INF, exactly what each module hard-coded before, and every
 * diagnostic guarded by the flag compiles to nothing.
 *
 * Do not deploy the debug image. It is not unsafe on air -- it changes no
 * frame, no timing constant and no gate -- but a deaf-heartbeat line per
 * second plus a line per received frame is a lot of console traffic, and the
 * heartbeat's dwt_readsysstatuslo() is an extra SPI transfer on the SLAVE loop.
 */

#ifndef UWB_DEBUG_H
#define UWB_DEBUG_H

#include <zephyr/logging/log.h>

#ifdef CONFIG_ANCLA_RANGING_DEBUG
#define ANCLA_LOG_LEVEL LOG_LEVEL_DBG
#else
#define ANCLA_LOG_LEVEL LOG_LEVEL_INF
#endif

#endif /* UWB_DEBUG_H */
