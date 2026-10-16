/*
 * Copyright (c) 2026 Innovaforce
 * SPDX-License-Identifier: Apache-2.0
 */

#include "uwb_irq_wait.h"

#include <errno.h>
#include <zephyr/sys/time_units.h>
#include <deca_device_api.h>

bool uwb_irq_pump(void)
{
#ifdef CONFIG_ANCLA_DW_IRQ_POLL
	if (dwt_checkirq()) {
		dwt_isr();
		return true;
	}
#endif
	return false;
}

int uwb_irq_wait(struct k_sem *sem, k_timeout_t timeout)
{
#ifndef CONFIG_ANCLA_DW_IRQ_POLL
	return k_sem_take(sem, timeout);
#else
	k_timepoint_t end = sys_timepoint_calc(timeout);

	while (1) {
		/* Service first, then look: a callback dwt_isr() runs here gives
		 * the semaphore synchronously, so the take below sees it without
		 * paying a sleep. */
		(void)uwb_irq_pump();

		if (k_sem_take(sem, K_NO_WAIT) == 0) {
			return 0;
		}
		if (sys_timepoint_expired(end)) {
			return -EAGAIN;
		}
		k_usleep(CONFIG_ANCLA_DW_IRQ_POLL_PERIOD_US);
	}
#endif
}
