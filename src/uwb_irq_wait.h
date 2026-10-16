/*
 * Copyright (c) 2026 Innovaforce
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef UWB_IRQ_WAIT_H_
#define UWB_IRQ_WAIT_H_

#include <zephyr/kernel.h>

/* Wait for an RX callback to give `sem`, the way the radio loops always have.
 *
 * With the DW3000 IRQ line in use this is exactly k_sem_take(): br101 runs
 * dwt_isr() from the system workqueue and the callback gives the semaphore.
 *
 * With CONFIG_ANCLA_DW_IRQ_POLL (boards whose IRQ line does not reach the
 * ESP32) nothing ever runs dwt_isr() on its own, so this loop does it from
 * the calling thread: dwt_checkirq() reads SYS_STATUS.IRQS -- the same bit
 * that drives the IRQ pin -- and dwt_isr() then dispatches the callbacks,
 * which give `sem` as before. Between reads it SLEEPS
 * CONFIG_ANCLA_DW_IRQ_POLL_PERIOD_US, never busy-waits, so lower-priority
 * threads (WiFi, MQTT uplink, shell) keep running and the K_PRIO_COOP(0)
 * gateway loop still yields where it used to block in k_sem_take().
 *
 * Returns 0 when `sem` was taken, -EAGAIN when `timeout` expired. */
int uwb_irq_wait(struct k_sem *sem, k_timeout_t timeout);

/* Poll-mode only: service one pending DW3000 event, if any. A no-op that
 * returns false when the IRQ line is in use. For loops that wait on more than
 * one object (cal_run.c's k_poll) and so cannot use uwb_irq_wait(). */
bool uwb_irq_pump(void);

#endif /* UWB_IRQ_WAIT_H_ */
