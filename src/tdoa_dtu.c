/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tdoa_dtu.h"

#define TDOA_DTU_HALF  (TDOA_DTU_MODULO / 2LL)

void tdoa_dtu_rebase(struct tdoa_meas *m, size_t n)
{
	int64_t base;
	size_t i;

	if (m == NULL || n == 0u) {
		return;
	}

	base = m[0].t_dtu;

	for (i = 0u; i < n; i++) {
		/* % en C trunca hacia cero, así que el resto puede salir
		 * negativo; se normaliza a [0, 2^40) y sólo entonces se pliega
		 * a [-2^39, 2^39). Hacerlo en un paso con % sobre un valor
		 * posiblemente negativo es exactamente el error que hace que
		 * esto funcione en un sentido de la vuelta y no en el otro. */
		int64_t d = (m[i].t_dtu - base) % TDOA_DTU_MODULO;

		if (d < 0) {
			d += TDOA_DTU_MODULO;
		}
		if (d >= TDOA_DTU_HALF) {
			d -= TDOA_DTU_MODULO;
		}
		m[i].t_dtu = d;
	}
}

bool tdoa_dtu_plausible(const struct tdoa_meas *m, size_t n)
{
	size_t i;

	if (m == NULL || n == 0u) {
		return false;
	}

	for (i = 0u; i < n; i++) {
		int64_t t = m[i].t_dtu;

		if (t > TDOA_DTU_MAX_SPREAD || t < -TDOA_DTU_MAX_SPREAD) {
			return false;
		}
	}
	return true;
}
