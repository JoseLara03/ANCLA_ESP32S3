# Fase 2 — CCP entre anclas y la medición del gate de sincronía (A7)

> **Para trabajadores agénticos:** SUB-SKILL REQUERIDA: usa
> `superpowers:subagent-driven-development` (recomendado) o
> `superpowers:executing-plans` para implementar este plan tarea por tarea. Los
> pasos usan sintaxis de casilla (`- [ ]`) para seguimiento.

**Meta:** hacer legible desde una consola el único número del que depende toda
la migración a TDoA — el jitter de timestamp por observación entre dos anclas —
implementando el pegamento de radio que `sync_model` y `ccp_frame` esperan y que
hoy no existe.

**Arquitectura:** el **gateway** es el master CCP (hop 0), porque ya posee la
base de tiempo de la red y ya hace una TX retardada por superframe. El CCP se
transmite en la **ventana de guarda posterior al beacon**, aire que los slaves ya
tienen prohibido usar, así que cuesta cero airtime del CAP o del CFP. Un ancla
SLAVE lo recibe, alimenta `sync_model` y expone el resultado con un comando
`sync`. Todo el trabajo nuevo de radio son dos funciones acotadas colgadas de
lazos existentes; no hay hilos nuevos.

**Stack:** Zephyr 4.4.x sobre ESP32-S3, driver Qorvo `dwt_uwb_driver` vendorizado
(br101), C puro host-testeado con gcc para todo lo que no toca radio.

**Spec:** `docs/superpowers/specs/2026-08-25-rtls-scale-tdoa-design.md` (tarea A7,
fase 2). El procedimiento de operador y la tabla de decisión del gate están en
`docs/anchor-sync-measurement.md`, que este plan también actualiza.

---

## Restricciones globales

Copiadas del spec, de `CLAUDE.md` y de los encabezados de los módulos. Los
requisitos de **cada** tarea las incluyen implícitamente.

- **El PHY está congelado:** canal 5, PLEN_1024, PAC32, código 9, 850 kbps,
  SFD_IEEE_4Z, STS/PDoA apagados. Ninguna tarea lo toca.
- **`ccp_frame.{c,h}` y `sync_model.{c,h}` ya existen, están host-testeados y NO
  se modifican.** Este plan solo los conecta. Si algo parece requerir cambiarlos,
  para y repórtalo — su contrato es lo que la Fase 3 va a consumir.
- **El código de función CCP es `0xEF` y es el ÚLTIMO libre del rango `0xEx`.**
  No inventes códigos nuevos. La tabla de asignación cruzada entre repos está en
  `src/ccp_frame.h`.
- **Toda espera en el lazo del gateway o del slave debe estar acotada.** Ambos
  corren cooperativos; el del gateway a `K_PRIO_COOP(0)`. Una espera sin cota ya
  congeló esta placa una vez. Usa `uwb_wait_for_sysstatus_lo(mask, timeout_ms)`,
  nunca un spin propio.
- **`TXFRS` se queda fuera de la máscara de interrupciones habilitada.**
  `dwt_setinterrupt(DWT_INT_RX, 0, DWT_ENABLE_INT)` es deliberado: las rutas de
  TX sondean la finalización, y habilitar la interrupción de TX dejaría que la
  ISR limpie TXFRS primero y cuelgue ese sondeo.
- **Nada grande en la pila.** `CONFIG_MAIN_STACK_SIZE` es 4096 y `struct
  gw_core_ctx` (2588 B) ya la desbordó una vez, en silencio. Todo estado de
  módulo va `static`. `sizeof(struct sync_model)` es 72 B — pequeño, pero la
  regla no admite excepciones por tamaño.
- **Cada módulo de la ruta de ranging registra su nivel de log a través de
  `ANCLA_LOG_LEVEL`** (`src/uwb_debug.h`), no de `LOG_LEVEL_INF` literal. Un
  nivel por módulo es un tope de **tiempo de compilación** que ningún `.conf` ni
  comando de consola puede levantar.
- **No agregues `CONFIG_LOG_MODE_IMMEDIATE`.** Está prohibido en `debug.conf` con
  la razón escrita: rompió el timing del beacon y tiró la placa.
- Los tests de host son gcc plano, sin Zephyr, y se listan en `CLAUDE.md` §
  "Host tests" con su línea de compilación exacta. Toda suite nueva se agrega
  ahí.
- El repo es git local, **sin remoto**. Nada se empuja a ningún lado. Los mensajes
  de commit terminan con
  `Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>`.

---

## Decisiones de diseño, resueltas antes de las tareas

Las tres estaban abiertas en el spec. Van resueltas aquí con su razón, para que
ningún implementador las re-litigue a mitad de una tarea.

### D1 — El master CCP vive en el GATEWAY, no en un slave

El gateway ya agenda una TX retardada por superframe (`tx_beacon()`), y el reloj
desde el que la agenda **es** la base de tiempo de la red. Poner el master ahí es
coherente, no un atajo.

La alternativa —rol master en un ancla SLAVE— se rechaza porque introduciría una
ruta de **TX no solicitada** en la imagen de producción de un slave. `CLAUDE.md`
registra que esa propiedad de seguridad ya se movió una vez (de "el set de build
lo impide" a "los dos gates de `apos_node.c` lo impiden") y que debilitarla
devuelve el peligro de colisión que la imagen de calibración se mantiene separada
para evitar. Un slave transmitiendo cada 200 ms sin que nadie se lo pida es
exactamente eso.

**Consecuencia para la prueba de roles invertidos** que pide
`docs/anchor-sync-measurement.md` §3: no se invierten los roles, se **invierten
las placas**. `anchor mode gateway` en la otra y `kernel reboot cold`. Es la misma
técnica que `CLAUDE.md` ya bendice para separar una falla de firmware de una de
placa, y no cuesta código.

### D2 — El CCP va en la ventana de guarda POSTERIOR al beacon

Los slaves suprimen su propia TX en `[T_b − guard, T_b + occupancy + guard]`,
donde `T_b` es el RMARKER del beacon (es lo que `beacon_guard_beacon()` recibe:
el timestamp de RX). Ese aire ya está reservado y vacío. Meter el CCP ahí cuesta
**cero** airtime del CAP o del CFP.

Aritmética, toda derivada con `mac_budget.h` y verificada:

| | ns relativos a `T_b` |
|---|---|
| el beacon (37 B) **termina** en | 389 411 |
| SHR del CCP (preámbulo + SFD) | 1 050 194 |
| RMARKER del CCP más temprano legal | 1 439 605 |
| **`CCP_OFFSET_UUS` elegido = `BEACON_OCCUPANCY_UUS` = 1500 uus** | **1 538 461** |
| el CCP **termina** en | 1 777 284 |
| la guarda **cierra** en | 3 076 922 |

Márgenes: **+98 856 ns** contra el fin del beacon y **+1 299 638 ns** contra el
cierre de la guarda. El offset no es un número mágico: es
`BEACON_OCCUPANCY_UUS`, es decir *"inmediatamente después de la ocupación que el
beacon ya declara para sí"*.

El CCP cuesta **1.289 ms de airtime, 0.645 % de un superframe de 200 ms**.

### D3 — Dos TX retardadas en el lazo `K_PRIO_COOP(0)`

Es una segunda TX retardada por superframe en el lazo donde esta semana se pagaron
un desbordamiento de pila silencioso y una espera sin cota. Mitigaciones, todas
obligatorias:

1. La TX del CCP reusa **exactamente** el patrón de `tx_beacon()`, incluida la
   espera acotada de TXFRS.
2. `ccp_master_after_beacon()` se llama **después** de que el beacon confirmó su
   TXFRS y **solo si** confirmó — nunca compite con el beacon por el arma.
3. Todo el estado del módulo es `static`.
4. La corrida de verificación se hace con la imagen debug, que lleva
   `CONFIG_THREAD_ANALYZER`. Se lee `main` y se compara contra el 1748 / 4096
   registrado en `CLAUDE.md`.

---

## Estructura de archivos

| Archivo | Responsabilidad |
|---|---|
| **Crear** `src/ccp_sched.h` | *Solo header, sin `.c`* — mismo patrón que `uwb_mac.h`. Dónde cae el CCP en el superframe (`CCP_OFFSET_UUS`) y los dos `BUILD_ASSERT` que prueban que cabe. |
| **Crear** `tests/ccp_sched/test_ccp_sched.c` | **Incluir el header ES el test**: bajo el shim de gcc los `BUILD_ASSERT` son `_Static_assert`, así que un presupuesto que deja de cumplirse falla al **compilar**. Mismo patrón que `tests/mac_budget/test_uwb_mac_asserts.c`. |
| **Crear** `src/ccp_master.{c,h}` | La TX del CCP en el gateway. Una función, acotada, llamada tras el beacon. |
| **Crear** `src/ccp_slave.{c,h}` | La RX del CCP en el slave: dueño del `struct sync_model` estático, detección de huecos por `ccp_seq`, contadores. |
| **Crear** `src/sync_shell.c` | El árbol `sync`. Convierte el gate en una línea leíble, con veredicto. |
| **Modificar** `src/uwb_gateway.c` | Llamar `ccp_master_init()` y `ccp_master_after_beacon()`. |
| **Modificar** `src/uwb_slave.c` | Llamar `ccp_slave_init()` y meter `ccp_slave_on_rx()` en la cadena de despacho. |
| **Modificar** `CMakeLists.txt` | Tres `.c` nuevos. |
| **Modificar** `docs/anchor-sync-measurement.md` | §2 deja de decir "no implementado". |
| **Modificar** `CLAUDE.md` | Entradas de layout, host tests y consola. |

`ccp_sched.h` se separa de `ccp_master.c` a propósito: el presupuesto de airtime
tiene que ser verificable **sin** un toolchain de Zephyr, y es lo único de esta
fase que puede estar mal de forma silenciosa y arruinar la medición entera.

---

## Task 1: `ccp_sched.h` — dónde cae el CCP, probado en compilación

**Files:**
- Crear: `src/ccp_sched.h`
- Crear: `tests/ccp_sched/test_ccp_sched.c`
- Modificar: `CLAUDE.md` (sección "Host tests" y "Layout")

**Interfaces:**
- Consume: `BEACON_OCCUPANCY_UUS`, `BEACON_GUARD_UUS` de `src/uwb_mac.h`;
  `MAC_PS_TO_NS`, `MAC_UUS_TO_NS`, `MAC_SHR_PS`, `MAC_BITS_PS`, `MAC_PHR_BITS`,
  `MAC_FCS_BYTES`, `MAC_PHY_PLEN_SYM`, `MAC_PHY_SFD_SYM`, `MAC_PHY_BITRATE` de
  `src/mac_budget.h`; `CCP_FRAME_LEN` de `src/ccp_frame.h`;
  `UWB_FRAME_MAX_LEN` de `src/uwb_frame_802_15_4z.h`.
- Produce: `CCP_OFFSET_UUS` (macro entera, valor 1500), y los macros auxiliares
  `CCP_SCHED_SHR_NS`, `CCP_SCHED_POST_RMARKER_NS(bytes)`, `CCP_SCHED_AT_NS`,
  `CCP_SCHED_BEACON_END_NS`, `CCP_SCHED_GUARD_END_NS`. La Task 2 usa
  **`CCP_OFFSET_UUS`**; los demás existen para los asserts y los tests.

- [ ] **Step 1: escribir el header**

Crear `src/ccp_sched.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Where the CCP sits inside the superframe.
 *
 * Header only, no .c file, same as uwb_mac.h: everything here is compile-time
 * arithmetic, and the BUILD_ASSERTs at the bottom ARE the deliverable. There is
 * no runtime behaviour to test, only a budget that must hold.
 *
 * The placement decision, and why it costs nothing:
 *
 * A slave suppresses its own TX inside [T_b - BEACON_GUARD_UUS,
 * T_b + BEACON_OCCUPANCY_UUS + BEACON_GUARD_UUS], where T_b is the beacon's
 * RMARKER -- that is the value beacon_guard_beacon() is fed, since it comes
 * from the beacon's RX timestamp. That window is already reserved and already
 * empty, so a CCP transmitted inside it takes NO airtime away from the CAP or
 * the CFP. The alternative -- a slot of its own -- would have cost 0.645% of
 * every superframe and forced the capacity model in the design spec section 3.2
 * to be re-run.
 *
 * The offset is BEACON_OCCUPANCY_UUS rather than a tuned number, and that is
 * the point: it reads as "immediately after the occupancy the beacon already
 * declares for itself". The two asserts below prove both edges hold.
 *
 * Everything is measured from the beacon's RMARKER, because that is what a
 * delayed TX is programmed against. Note the RMARKER sits at the END of its own
 * SHR, so a frame's preamble PRECEDES its scheduled time -- this is the same
 * trap that made an earlier SS-TWR span estimate double-count both SHRs; see
 * MAC_SSTWR_EXCHANGE_PS in mac_budget.h.
 */

#ifndef CCP_SCHED_H
#define CCP_SCHED_H

#include "ccp_frame.h"
#include "mac_budget.h"
#include "uwb_frame_802_15_4z.h"
#include "uwb_mac.h"

#include <zephyr/sys/util.h>

/* Offset from the beacon's RMARKER to the CCP's RMARKER, in UUS. The caller
 * multiplies by UUS_TO_DWT_TIME; this header stays free of radio headers so the
 * budget is checkable under plain gcc. */
#define CCP_OFFSET_UUS  BEACON_OCCUPANCY_UUS

/* ---- Derived quantities, for the asserts and for tests/ccp_sched ---- */

/* Preamble + SFD. A frame's RMARKER is at the end of this, so the SHR occupies
 * the air BEFORE the scheduled time. */
#define CCP_SCHED_SHR_NS                                                       \
	MAC_PS_TO_NS(MAC_SHR_PS(MAC_PHY_PLEN_SYM, MAC_PHY_SFD_SYM))

/* Airtime AFTER the RMARKER: PHR + payload + FCS. `bytes` excludes the FCS,
 * matching the UWB_FRAME_LEN_* convention used everywhere in this project. */
#define CCP_SCHED_POST_RMARKER_NS(bytes)                                       \
	MAC_PS_TO_NS(MAC_BITS_PS(MAC_PHR_BITS, MAC_PHY_BITRATE) +               \
		     MAC_BITS_PS(((uint64_t)(bytes) + MAC_FCS_BYTES) * 8ULL,    \
				 MAC_PHY_BITRATE))

/* The CCP's scheduled RMARKER, relative to the beacon's. */
#define CCP_SCHED_AT_NS         MAC_UUS_TO_NS(CCP_OFFSET_UUS)

/* Where the beacon's own frame stops occupying the air, relative to its
 * RMARKER. UWB_FRAME_MAX_LEN is the beacon length: it is the longest frame the
 * contract defines, and tx_beacon() builds exactly that. */
#define CCP_SCHED_BEACON_END_NS                                                \
	CCP_SCHED_POST_RMARKER_NS(UWB_FRAME_MAX_LEN)

/* Where the slaves' suppression window closes, relative to the beacon's
 * RMARKER. */
#define CCP_SCHED_GUARD_END_NS                                                 \
	(MAC_UUS_TO_NS(BEACON_OCCUPANCY_UUS) + MAC_UUS_TO_NS(BEACON_GUARD_UUS))

/* (a) The CCP's PREAMBLE must not start before the beacon's frame has finished.
 * Checking the RMARKER alone would pass while the preamble sat on top of the
 * beacon's payload -- 1050 us of collision that a sniffer would show as a
 * corrupt beacon and nothing would attribute to the CCP. */
BUILD_ASSERT(CCP_SCHED_AT_NS >= CCP_SCHED_BEACON_END_NS + CCP_SCHED_SHR_NS,
	     "CCP preamble would start before the beacon frame ends");

/* (b) The CCP must be off the air before the suppression window closes, or it
 * collides with the first legitimate slave transmit of the CAP. */
BUILD_ASSERT(CCP_SCHED_AT_NS + CCP_SCHED_POST_RMARKER_NS(CCP_FRAME_LEN) <=
		     CCP_SCHED_GUARD_END_NS,
	     "CCP still transmitting when the beacon guard closes");

#endif /* CCP_SCHED_H */
```

- [ ] **Step 2: escribir el test que falla**

Crear `tests/ccp_sched/test_ccp_sched.c`. **Incluir el header es el test** — si
un presupuesto deja de cumplirse, esto no compila:

```c
/* Including ccp_sched.h IS the test: under tests/mac_budget/shim its
 * BUILD_ASSERTs are _Static_assert, so a budget that stops holding fails to
 * COMPILE under plain gcc instead of waiting for a Zephyr build. Same pattern
 * as tests/mac_budget/test_uwb_mac_asserts.c.
 *
 * The printed figures are not decoration: they are the numbers the design
 * decision in docs/superpowers/plans/2026-08-26-fase2-ccp-sync.md D2 was made
 * from, so a reader can confirm the table rather than trust it. */
#include "ccp_sched.h"

#include <stdio.h>

static int failures;

#define CHECK(cond)                                                            \
	do {                                                                   \
		if (!(cond)) {                                                 \
			printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond);  \
			failures++;                                            \
		}                                                              \
	} while (0)

/* The offset is BEACON_OCCUPANCY_UUS by DERIVATION, not by coincidence. If
 * someone retunes it to a literal, this fails and says why. */
static void test_offset_is_the_beacon_occupancy(void)
{
	CHECK(CCP_OFFSET_UUS == BEACON_OCCUPANCY_UUS);
	CHECK(CCP_OFFSET_UUS == 1500u);
}

/* Both edges, with their margins pinned. A change that leaves the asserts
 * holding but eats the margin is worth seeing. */
static void test_ccp_fits_the_post_beacon_guard(void)
{
	uint32_t earliest = CCP_SCHED_BEACON_END_NS + CCP_SCHED_SHR_NS;
	uint32_t ends_at = CCP_SCHED_AT_NS +
			   CCP_SCHED_POST_RMARKER_NS(CCP_FRAME_LEN);

	CHECK(CCP_SCHED_AT_NS >= earliest);
	CHECK(ends_at <= CCP_SCHED_GUARD_END_NS);

	/* Measured 2026-08-26. Exact values, not bounds: these are constant
	 * arithmetic over frozen PHY parameters, so anything that moves them
	 * moves the airtime model and should be read, not silently absorbed. */
	CHECK(CCP_SCHED_SHR_NS == 1050194u);
	CHECK(CCP_SCHED_BEACON_END_NS == 389411u);
	CHECK(earliest == 1439605u);
	CHECK(CCP_SCHED_AT_NS == 1538461u);
	CHECK(ends_at == 1777284u);
	CHECK(CCP_SCHED_GUARD_END_NS == 3076922u);

	CHECK(CCP_SCHED_AT_NS - earliest == 98856u);
	CHECK(CCP_SCHED_GUARD_END_NS - ends_at == 1299638u);
}

/* The CCP's total airtime and its share of a superframe. Quoted in
 * docs/anchor-sync-measurement.md section 2, so pin it here rather than letting
 * the doc drift. */
static void test_airtime_share_is_recorded(void)
{
	uint32_t full = CCP_SCHED_SHR_NS +
			CCP_SCHED_POST_RMARKER_NS(CCP_FRAME_LEN);

	CHECK(full == 1289017u);
	/* 0.645% of a 200 ms superframe, to a tenth of a per mille. */
	CHECK((uint32_t)((uint64_t)full * 10000u /
			 MAC_UUS_TO_NS(T_SUPERFRAME_UUS)) == 64u);
}

int main(void)
{
	test_offset_is_the_beacon_occupancy();
	test_ccp_fits_the_post_beacon_guard();
	test_airtime_share_is_recorded();

	if (failures) {
		printf("FAILED %d\n", failures);
		return 1;
	}
	printf("ALL TESTS PASSED\n");
	return 0;
}
```

- [ ] **Step 3: correr el test y verificar que FALLA**

Antes de crear el header, el test no compila. Correr:

```powershell
gcc -Wall -Wextra -Isrc -Itests/mac_budget/shim -o tests/ccp_sched/test_ccp_sched.exe tests/ccp_sched/test_ccp_sched.c src/mac_budget.c
```

Esperado: `fatal error: ccp_sched.h: No such file or directory`.

Si ya creaste el header en el Step 1, **neutraliza un assert a propósito** para
ver que el test tiene dientes: cambia `CCP_OFFSET_UUS` a `1400u` y recompila.
Esperado: falla la compilación con *"CCP preamble would start before the beacon
frame ends"*. Restaura `BEACON_OCCUPANCY_UUS` después. **No sigas sin haber visto
ese fallo** — un assert que nunca se vio fallar no es evidencia de nada.

- [ ] **Step 4: correr el test y verificar que PASA**

```powershell
gcc -Wall -Wextra -Isrc -Itests/mac_budget/shim -o tests/ccp_sched/test_ccp_sched.exe tests/ccp_sched/test_ccp_sched.c src/mac_budget.c
./tests/ccp_sched/test_ccp_sched.exe
```

Esperado: `ALL TESTS PASSED`, exit 0.

- [ ] **Step 5: registrar la suite en CLAUDE.md**

En la sección "Host tests", después del bloque de `test_uwb_mac_asserts.exe`,
agregar:

```powershell
gcc -Wall -Wextra -Isrc -Itests/mac_budget/shim -o tests/ccp_sched/test_ccp_sched.exe tests/ccp_sched/test_ccp_sched.c src/mac_budget.c
./tests/ccp_sched/test_ccp_sched.exe            # ALL TESTS PASSED, exits 0
```

Y en la nota que sigue sobre `-Itests/mac_budget/shim`, agregar que
`tests/ccp_sched/` lo necesita por la misma razón.

En la sección "Layout", después de la entrada de `src/ccp_frame.{c,h}`:

```
- `src/ccp_sched.h` — dónde cae el CCP en el superframe: `CCP_OFFSET_UUS`
  (= `BEACON_OCCUPANCY_UUS`, es decir inmediatamente después de la ocupación
  del beacon, dentro de la guarda que los slaves ya no pueden usar) y los dos
  `BUILD_ASSERT` que prueban que el preámbulo no pisa el beacon y que la trama
  termina antes de que cierre la guarda. Solo header, sin `.c`, mismo patrón
  que `uwb_mac.h`; host-testeado en `tests/ccp_sched/` donde **incluir el
  header es el test**.
```

- [ ] **Step 6: commit**

```bash
git add src/ccp_sched.h tests/ccp_sched/test_ccp_sched.c CLAUDE.md
git commit -F - <<'EOF'
feat(ccp): place the CCP in the post-beacon guard, and prove it fits

The slaves already suppress their own TX inside [T_b - BEACON_GUARD_UUS,
T_b + BEACON_OCCUPANCY_UUS + BEACON_GUARD_UUS], so a CCP transmitted there
takes no airtime from the CAP or the CFP at all. A slot of its own would have
cost 0.645% of every superframe and forced the capacity model in the design
spec section 3.2 to be re-run.

CCP_OFFSET_UUS is BEACON_OCCUPANCY_UUS rather than a tuned literal: it reads
as "immediately after the occupancy the beacon already declares for itself".

Two BUILD_ASSERTs, both measured from the beacon's RMARKER because that is
what a delayed TX is programmed against:

  (a) the CCP's PREAMBLE starts after the beacon's frame ends -- margin
      98 856 ns. Checking the RMARKER alone would pass while 1050 us of
      preamble sat on top of the beacon's payload, which a sniffer would show
      as a corrupt beacon and nothing would attribute to the CCP.
  (b) the CCP is off the air before the window closes -- margin 1 299 638 ns.

Header only, no .c, same as uwb_mac.h: it is all compile-time arithmetic and
the asserts are the deliverable. Host-tested where INCLUDING THE HEADER IS THE
TEST, so a budget that stops holding fails to compile under plain gcc instead
of waiting for a Zephyr build.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
EOF
```

---

## Task 2: `ccp_master` — el gateway transmite un CCP tras cada beacon

**Files:**
- Crear: `src/ccp_master.h`, `src/ccp_master.c`
- Modificar: `src/uwb_gateway.c`
- Modificar: `CMakeLists.txt`

**Interfaces:**
- Consume: `CCP_OFFSET_UUS` (Task 1); `ccp_frame_build()`, `struct ccp_frame`,
  `CCP_FRAME_LEN`, `CCP_HOP_ROOT` de `src/ccp_frame.h`; `SYNC_DTU_MASK` de
  `src/sync_model.h`; `UUS_TO_DWT_TIME`, `FCS_LEN`,
  `uwb_wait_for_sysstatus_lo()` de `src/uwb_dwtime.h`;
  `UWB_ADDR_GATEWAY_RESERVED` de `src/uwb_config.h`; `tag_id_from_eui()` de
  `src/tag_id.h`.
- Produce:
  - `void ccp_master_init(void);`
  - `void ccp_master_after_beacon(uint64_t beacon_tx_dtu, uint8_t *frame_seq);`
  - `void ccp_master_stats(uint32_t *sent, uint32_t *dropped, uint32_t *root_id);`

- [ ] **Step 1: escribir el header**

Crear `src/ccp_master.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The gateway's clock-calibration-packet transmitter: the root of the anchor
 * sync tree (hop 0).
 *
 * On the GATEWAY, deliberately. The gateway already schedules one delayed TX
 * per superframe and the clock it schedules from IS the network's time base, so
 * the CCP master and the beacon master being the same board is coherent rather
 * than convenient. A master role on a SLAVE was rejected: it would add an
 * UNSOLICITED transmit path to a production slave image, which is exactly the
 * collision hazard apos_node.c's two gates exist to prevent -- see CLAUDE.md on
 * why ss_initiator.c being in production is safe only because of them.
 *
 * To repeat the measurement with the roles swapped (docs/anchor-sync-
 * measurement.md section 3), swap the BOARDS: `anchor mode gateway` on the
 * other one and `kernel reboot cold`. Same technique CLAUDE.md already
 * prescribes for separating a firmware fault from a board fault.
 */

#ifndef CCP_MASTER_H
#define CCP_MASTER_H

#include <stdint.h>

/* Derives root_id from this board's EUI-64 and clears the counters. Call once
 * before the gateway loop starts. */
void ccp_master_init(void);

/* Transmit one CCP, scheduled CCP_OFFSET_UUS after the beacon's RMARKER.
 *
 * Call ONLY with a beacon_tx_dtu that tx_beacon() actually returned, i.e. only
 * after that beacon's TXFRS was confirmed. A CCP scheduled from a beacon that
 * never transmitted would be scheduled from a time that does not exist.
 *
 * `frame_seq` is the gateway's shared 802.15.4 sequence counter, consumed by
 * reference exactly as tx_beacon() and send_grant() consume it.
 *
 * Bounded: at most one delayed TX and one bounded TXFRS wait
 * (CCP_MASTER_TX_TIMEOUT_MS). Never blocks. Safe on the K_PRIO_COOP(0) loop. */
void ccp_master_after_beacon(uint64_t beacon_tx_dtu, uint8_t *frame_seq);

/* Diagnostics. `dropped` counts CCPs whose transmission was not CONFIRMED --
 * a failed dwt_starttx() or a TXFRS that never arrived. Each one leaves a gap
 * in ccp_seq on purpose, so a receiver sees the miss. */
void ccp_master_stats(uint32_t *sent, uint32_t *dropped, uint32_t *root_id);

#endif /* CCP_MASTER_H */
```

- [ ] **Step 2: escribir la implementación**

Crear `src/ccp_master.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ccp_master.h"

#include "ccp_frame.h"
#include "ccp_sched.h"
#include "sync_model.h"
#include "tag_id.h"
#include "uwb_config.h"
#include "uwb_debug.h"
#include "uwb_dwtime.h"

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <deca_device_api.h>

LOG_MODULE_REGISTER(ccp_master, ANCLA_LOG_LEVEL);

/* Bound for the post-dwt_starttx() TXFRS wait. The CCP is scheduled
 * CCP_OFFSET_UUS (1500 uus ~= 1.54 ms) after the beacon's RMARKER, and
 * ccp_master_after_beacon() is called once that beacon's TXFRS has already been
 * confirmed -- so by the time we arm, at most ~1.54 ms of the offset remains,
 * plus 1.29 ms of CCP airtime. Same derivation style as the gateway's own
 * TX_COMPLETE_TIMEOUT_MS: ceil(worst_ms) + 5 ms of margin.
 *
 * This is a SEPARATE constant from uwb_gateway.c's TX_COMPLETE_TIMEOUT_MS (11)
 * and anchor_respond.c's (18). All three cover different worst cases and
 * CLAUDE.md already records one bug caused by conflating two of them. */
#define CCP_MASTER_TX_TIMEOUT_MS 8

static uint8_t  tx_buf[CCP_FRAME_LEN];
static uint32_t root_id;
static uint8_t  ccp_seq;
static uint32_t n_sent;
static uint32_t n_dropped;

void ccp_master_init(void)
{
	uint8_t eui[8] = {0};
	ssize_t n = hwinfo_get_device_id(eui, sizeof(eui));

	if (n != (ssize_t)sizeof(eui)) {
		/* Not fatal for the measurement: root_id only has to be stable
		 * for this board's lifetime and distinct from other roots on
		 * the same air, and a single site has one root. Say so instead
		 * of pretending, because a receiver keys its baseline on it. */
		LOG_WRN("hwinfo_get_device_id gave %d bytes — root_id falls "
			"back to a constant, which is only safe with ONE "
			"gateway on this air", (int)n);
		root_id = 0xC0FFEE01u;
	} else {
		/* FNV-1a over the whole 8-byte EUI, reusing the tag's hash so
		 * this project has one derivation rather than two. Its
		 * 31-bit mask is irrelevant here -- root_id has no int32
		 * consumer -- but harmless, and sharing the function is worth
		 * more than shaving a bit. */
		root_id = tag_id_from_eui(eui, sizeof(eui));
	}

	ccp_seq = 0;
	n_sent = 0;
	n_dropped = 0;

	LOG_INF("{\"ccp_master\":{\"root_id\":%u,\"offset_uus\":%u}}",
		root_id, (unsigned int)CCP_OFFSET_UUS);
}

void ccp_master_after_beacon(uint64_t beacon_tx_dtu, uint8_t *frame_seq)
{
	/* The delayed-TX register is programmed in hi32 units, so the hardware
	 * rounds the RMARKER DOWN to a 256-DTU boundary. The payload MUST carry
	 * the rounded value.
	 *
	 * This is the single most destructive mistake available in this file.
	 * Carrying the unrounded value puts up to 255 DTU -- ~4 ns -- of error
	 * into every observation, and the gate this whole phase exists to
	 * measure has a threshold of 1 ns. The measurement would fail, and it
	 * would fail in the direction that looks like a hardware verdict. */
	uint64_t want = (beacon_tx_dtu +
			 (uint64_t)CCP_OFFSET_UUS * UUS_TO_DWT_TIME) &
			SYNC_DTU_MASK;
	uint32_t at_hi32 = (uint32_t)(want >> 8);
	uint64_t rmarker = ((uint64_t)at_hi32) << 8;

	struct ccp_frame f = {
		.seq = ccp_seq,
		.hop = CCP_HOP_ROOT,
		.tx_dtu = rmarker,
		.root_id = root_id,
	};

	/* Consumed at BUILD time, not on success, and that is deliberate. A CCP
	 * that is built and never transmitted must leave a GAP so the receiver
	 * calls sync_model_miss() for it. Incrementing only on success would
	 * hide the skipped superframe and silently corrupt the receiver's
	 * baseline -- it would fold two intervals in as one. Same convention as
	 * frame_seq_nb on the responder path; see CLAUDE.md on reading sniffer
	 * captures. */
	ccp_seq++;

	int n = ccp_frame_build(tx_buf, sizeof(tx_buf),
				UWB_ADDR_GATEWAY_RESERVED, (*frame_seq)++, &f);

	if (n < 0) {
		LOG_ERR("CCP build failed (%d)", n);
		n_dropped++;
		return;
	}

	dwt_setdelayedtrxtime(at_hi32);
	dwt_writetxdata((uint16_t)n, tx_buf, 0);
	dwt_writetxfctrl((uint16_t)(n + FCS_LEN), 0, 0);

	if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS) {
		dwt_forcetrxoff();
		n_dropped++;
		LOG_DBG("CCP missed its slot");
		return;
	}
	if (!uwb_wait_for_sysstatus_lo(DWT_INT_TXFRS_BIT_MASK,
				       CCP_MASTER_TX_TIMEOUT_MS)) {
		/* NOT optional, and not merely tidy. CLAUDE.md records that
		 * dwt_starttx() can report DWT_SUCCESS for a delayed TX that
		 * never happens. Counting such a CCP as sent would leave the
		 * receiver with no gap to notice, so it would fold a
		 * two-interval span in as one -- a fabricated observation in
		 * the one estimator the whole TDoA migration turns on. */
		dwt_forcetrxoff();
		n_dropped++;
		LOG_WRN("CCP started but TXFRS never completed — forced off");
		return;
	}
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
	n_sent++;
}

void ccp_master_stats(uint32_t *sent, uint32_t *dropped, uint32_t *root)
{
	if (sent) {
		*sent = n_sent;
	}
	if (dropped) {
		*dropped = n_dropped;
	}
	if (root) {
		*root = root_id;
	}
}
```

- [ ] **Step 3: colgarlo del lazo del gateway**

En `src/uwb_gateway.c`, agregar el include junto a los otros locales (orden
alfabético, después de `apos_gw.h`):

```c
#include "ccp_master.h"
```

Inicializar justo después de `gw_core_init(&ctx);`:

```c
	ccp_master_init();
```

Y en el cuerpo del lazo externo, reemplazar:

```c
		CRUMB(GW_CRUMB_TX_BEACON);
		uint64_t ts = tx_beacon(&ctx, true, next_beacon);
```

por:

```c
		CRUMB(GW_CRUMB_TX_BEACON);
		uint64_t ts = tx_beacon(&ctx, true, next_beacon);

		/* Only on a CONFIRMED beacon. ts == 0 means the beacon did not
		 * go out, and a CCP scheduled from a beacon that never
		 * transmitted would be scheduled from a time that does not
		 * exist. Bounded: one delayed TX, one bounded TXFRS wait. */
		if (ts != 0u) {
			CRUMB(GW_CRUMB_CCP_TX);
			ccp_master_after_beacon(ts, &gw_seq);
		}
```

Agregar el crumb nuevo al `enum gw_crumb`, **al final** para no renumerar los
existentes (su numeración es un contrato con la leyenda que imprime
`gw_stall_expiry`):

```c
	GW_CRUMB_CCP_TX,      /* 9: inside ccp_master_after_beacon() */
```

Y extender la leyenda en el `LOG_ERR` de `gw_stall_expiry()`:

```c
		"8=tx_beacon 9=ccp_tx\"}}",
```

- [ ] **Step 4: agregarlo al build**

En `CMakeLists.txt`, en la lista `src/` del target de producción, en orden
alfabético después de `src/ccp_frame.c`:

```cmake
	src/ccp_master.c
```

- [ ] **Step 5: compilar ambas imágenes**

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
west build -b ancla_esp32s3/esp32s3/procpu -d build_dbg -- "-DEXTRA_CONF_FILE=debug.conf"
```

Esperado: exit 0 en las dos, sin warnings. Cita `-D` — PowerShell parte un
`-DEXTRA_CONF_FILE=debug.conf` sin comillas en el punto.

- [ ] **Step 6: verificar en hardware con sniffer**

Flashear **producción** en el gateway. Con el sniffer DWM3001CDK:

1. Debe aparecer una trama `0xEF` de 21 bytes (23 en el aire con FCS) **después
   de cada beacon `0xE5`**, con `src = 0x0000`.
2. El byte en el offset 10 (`ccp_seq`) debe incrementar de 1 en 1 sin huecos.
3. El byte en el offset 11 (`hop`) debe ser `0x00`.
4. **El beacon debe seguir a tiempo.** Ningún `"beacon started but TXFRS never
   completed"` en la consola. Con la imagen debug, el heartbeat `gw_sf` debe
   seguir cada 200.0 ms y `main` en `thread_analyzer` no debe pasar de ~1800 /
   4096 (el pico registrado en `CLAUDE.md` es 1748).

Si aparecen `"CCP started but TXFRS never completed"` de forma sostenida, **no
subas `CCP_MASTER_TX_TIMEOUT_MS` sin derivarlo de nuevo** — mide primero cuánto
del offset queda al armar. `CLAUDE.md` registra un bug causado exactamente por
tratar uno de estos límites como perilla.

- [ ] **Step 7: commit**

```bash
git add src/ccp_master.c src/ccp_master.h src/uwb_gateway.c CMakeLists.txt
git commit -F - <<'EOF'
feat(ccp): the gateway transmits a CCP after every beacon

Root of the anchor sync tree (hop 0), on the gateway deliberately: it already
schedules one delayed TX per superframe and the clock it schedules from IS the
network's time base. A master role on a SLAVE was rejected because it would add
an unsolicited transmit path to a production slave image -- the exact collision
hazard apos_node.c's two gates exist to prevent.

Three things in here are load-bearing and none is obvious:

- The delayed-TX register is programmed in hi32, so the hardware rounds the
  RMARKER down to a 256-DTU boundary and the PAYLOAD must carry the rounded
  value. Carrying the unrounded one puts up to 255 DTU (~4 ns) into every
  observation against a gate threshold of 1 ns -- the measurement would fail,
  and it would fail looking like a hardware verdict.
- ccp_seq is consumed at BUILD time, not on success. A CCP built and never
  transmitted must leave a gap so the receiver calls sync_model_miss() for it;
  incrementing only on success would hide the skipped superframe and fold two
  intervals in as one. Same convention as frame_seq_nb on the responder path.
- The TXFRS wait is not tidiness. dwt_starttx() can report DWT_SUCCESS for a
  delayed TX that never happens, and counting that CCP as sent would inject a
  fabricated observation into the one estimator the TDoA migration turns on.

Called only with a beacon_tx_dtu that tx_beacon() actually returned, so a CCP
is never scheduled from a beacon that did not go out. Bounded throughout: one
delayed TX and one bounded TXFRS wait, safe on the K_PRIO_COOP(0) loop.

CCP_MASTER_TX_TIMEOUT_MS is a third constant of its kind, separate from
uwb_gateway.c's 11 and anchor_respond.c's 18. They cover different worst cases
and CLAUDE.md already records a bug from conflating two of them.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
EOF
```

---

## Task 3: `ccp_slave` + el comando `sync` — el gate se vuelve legible

**Files:**
- Crear: `src/ccp_slave.h`, `src/ccp_slave.c`, `src/sync_shell.c`
- Modificar: `src/uwb_slave.c`
- Modificar: `CMakeLists.txt`
- Modificar: `CLAUDE.md` (secciones "Consola" y "Layout")

**Interfaces:**
- Consume: `ccp_frame_is_ccp()`, `ccp_frame_parse()`, `struct ccp_frame` de
  `src/ccp_frame.h`; todo el API público de `src/sync_model.h`
  (`sync_model_init`, `sync_model_observe`, `sync_model_miss`,
  `sync_model_error_dtu`, `sync_model_drift_ppb`,
  `sync_model_residual_rms_dtu`, `sync_model_residual_max_dtu`,
  `sync_model_residual_count`, `sync_model_residual_reset`,
  `sync_model_jitter_est_dtu`, `SYNC_DTU_MASK`, `SYNC_MISS_MAX`).
- Produce:
  - `void ccp_slave_init(void);`
  - `bool ccp_slave_on_rx(const uint8_t *buf, uint16_t plen, uint64_t rx_ts);`
    — `true` si la trama era un CCP y quedó consumida.
  - `struct sync_model *ccp_slave_model(void);`
  - `void ccp_slave_stats(uint32_t *n_rx, uint32_t *n_gap, uint32_t *n_reject, uint32_t *root_id);`

- [ ] **Step 1: escribir el header del receptor**

Crear `src/ccp_slave.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The SLAVE side of anchor clock sync: receive CCPs, feed sync_model, and own
 * the one instance of it.
 *
 * This module holds no arithmetic of its own. Everything that could be wrong
 * about the estimator lives in sync_model.c, which is pure C and host-tested;
 * what lives here is the part a host test cannot reach -- a real RX timestamp
 * from a real DW3220 -- and the sequence bookkeeping that tells the model when
 * an expected CCP did not arrive.
 */

#ifndef CCP_SLAVE_H
#define CCP_SLAVE_H

#include "sync_model.h"

#include <stdbool.h>
#include <stdint.h>

/* Clear to the no-observations state. Call once before the SLAVE loop starts. */
void ccp_slave_init(void);

/* Offer one received frame. Returns true if it WAS a CCP -- consumed either
 * way, so the caller can stop its dispatch chain -- and false if the frame is
 * for someone else.
 *
 * `rx_ts` is the full 40-bit DW3220 RX timestamp, exactly as
 * uwb_get_rx_timestamp_u64() returns it. Do not shift it: sync_model works in
 * whole DTU, and handing it a hi32 value would throw away the bottom 8 bits --
 * 255 DTU, ~4 ns, against a 1 ns gate. */
bool ccp_slave_on_rx(const uint8_t *buf, uint16_t plen, uint64_t rx_ts);

/* The live model, for the `sync` shell command. Non-const because
 * sync_model_residual_reset() takes a mutable pointer.
 *
 * Unsynchronised, and safe today for one reason: the SLAVE loop is the only
 * writer and the shell is strictly lower priority, so a reader never observes a
 * half-updated model -- it sees the previous state or the next one. Same
 * reasoning, and the same fragility, as apos_gw_result(). */
struct sync_model *ccp_slave_model(void);

/* Counters. `n_gap` is expected CCPs that never arrived, `n_reject` is frames
 * that were CCPs but were not usable -- a parse failure, a hop this node may
 * not adopt, or a duplicate sequence number. */
void ccp_slave_stats(uint32_t *n_rx, uint32_t *n_gap, uint32_t *n_reject,
		     uint32_t *root_id);

#endif /* CCP_SLAVE_H */
```

- [ ] **Step 2: escribir la implementación del receptor**

Crear `src/ccp_slave.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ccp_slave.h"

#include "ccp_frame.h"
#include "uwb_debug.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ccp_slave, ANCLA_LOG_LEVEL);

/* static, not automatic. 72 bytes would fit anywhere, but CLAUDE.md records a
 * 2588-byte automatic silently overflowing the 4096-byte main stack, and the
 * rule that came out of it does not take exceptions for size. */
static struct sync_model model;

static uint32_t cur_root;
static uint8_t  last_seq;
static bool     have_seq;
static uint32_t n_rx;
static uint32_t n_gap;
static uint32_t n_reject;

void ccp_slave_init(void)
{
	sync_model_init(&model);
	cur_root = 0u;
	last_seq = 0u;
	have_seq = false;
	n_rx = 0u;
	n_gap = 0u;
	n_reject = 0u;
}

bool ccp_slave_on_rx(const uint8_t *buf, uint16_t plen, uint64_t rx_ts)
{
	struct ccp_frame f;

	if (!ccp_frame_is_ccp(buf, plen)) {
		return false;
	}

	/* ccp_frame_parse() also rejects a hop this node must not adopt, so the
	 * CCP_HOP_MAX check cannot be forgotten here. */
	if (ccp_frame_parse(buf, plen, &f) != 0) {
		n_reject++;
		return true;
	}

	if (!have_seq || f.root_id != cur_root) {
		/* A different root is a different time base, and the baseline
		 * built against the old one describes nothing. Start over
		 * rather than mixing two clocks into one rate estimate. */
		if (have_seq) {
			LOG_WRN("{\"ccp_slave\":{\"root_changed\":%u,"
				"\"was\":%u}}", f.root_id, cur_root);
		}
		sync_model_init(&model);
		cur_root = f.root_id;
		have_seq = true;
		last_seq = f.seq;
		sync_model_observe(&model, f.tx_dtu, rx_ts & SYNC_DTU_MASK);
		n_rx++;
		return true;
	}

	uint8_t gap = (uint8_t)(f.seq - last_seq); /* wraps at 256, as ccp_seq does */

	last_seq = f.seq;

	if (gap == 0u) {
		/* Same sequence number twice. Not a miss and not an
		 * observation: folding a duplicate in would count one interval
		 * as two and bias the rate. */
		n_reject++;
		return true;
	}

	if (gap > (uint8_t)(SYNC_MISS_MAX + 1u)) {
		/* Already coasted past the model's own limit, so its estimate
		 * is invalid whatever we do. A fresh baseline beats feeding it
		 * hundreds of misses one call at a time on the SLAVE loop. */
		n_gap += (uint32_t)gap - 1u;
		sync_model_init(&model);
	} else {
		for (uint8_t k = 1u; k < gap; k++) {
			sync_model_miss(&model);
			n_gap++;
		}
	}

	sync_model_observe(&model, f.tx_dtu, rx_ts & SYNC_DTU_MASK);
	n_rx++;
	return true;
}

struct sync_model *ccp_slave_model(void)
{
	return &model;
}

void ccp_slave_stats(uint32_t *rx, uint32_t *gap, uint32_t *reject,
		     uint32_t *root)
{
	if (rx) {
		*rx = n_rx;
	}
	if (gap) {
		*gap = n_gap;
	}
	if (reject) {
		*reject = n_reject;
	}
	if (root) {
		*root = cur_root;
	}
}
```

- [ ] **Step 3: colgarlo del despacho del slave**

En `src/uwb_slave.c`, agregar el include:

```c
#include "ccp_slave.h"
```

Inicializar antes del lazo, junto a `apos_node_init()`:

```c
	ccp_slave_init();
```

Y en la cadena de despacho (la que hoy empieza con `if
(!apos_node_on_rx(...))`), meter el CCP **primero**. Reemplazar:

```c
		if (!apos_node_on_rx(rx_buf, plen, &cfg_snapshot,
```

por:

```c
		/* First in the chain, and cheap to reject: ccp_frame_is_ccp()
		 * is a type-and-length test. A CCP arrives once per superframe
		 * and is for nobody else, so there is no reason to walk it past
		 * the survey and ranging responders. */
		if (ccp_slave_on_rx(rx_buf, plen, rx_ts)) {
			continue;
		}

		if (!apos_node_on_rx(rx_buf, plen, &cfg_snapshot,
```

**Verifica que `continue` sea correcto en ese contexto** — si el despacho no
está dentro de un lazo con `continue` disponible, usa la estructura `if/else if`
que ya tenga el archivo. No inventes control de flujo nuevo.

- [ ] **Step 4: escribir el comando `sync`**

Crear `src/sync_shell.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The `sync` command tree: the Phase 2 gate, read off a console.
 *
 * The whole point is that an operator reads ONE number and gets a verdict,
 * because the natural mistake here is expensive and silent. sync_model.h spells
 * it out: the residual RMS is NOT the jitter -- the prediction consumes two
 * noisy local timestamps, so the RMS sits at about 1.55x the real jitter. An
 * operator reading 64 DTU of RMS as 1 ns would be looking at ~0.65 ns and
 * REJECTING hardware that passes, killing the TDoA migration on a
 * misinterpretation. So this command prints jitter_est first, prints the
 * verdict itself, and prints rms only as a diagnostic beside it.
 */

#include "ccp_slave.h"
#include "sync_model.h"

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include <stdlib.h>

/* Thresholds from docs/anchor-sync-measurement.md section 4. 64 DTU = 1 ns
 * (SYNC_DTU_PER_NS), 32 DTU = 0.5 ns. */
#define SYNC_GATE_PASS_DTU      32u
#define SYNC_GATE_FAIL_DTU      64u

/* The statistic is an RMS and a short sample is a noisy one. The simulation in
 * tests/sync_model uses 400 observations and section 3 of the doc asks the
 * operator for the same, so refuse to render a verdict below it rather than
 * letting a 20-sample reading look authoritative. */
#define SYNC_GATE_MIN_COUNT     400u

static const char *verdict_of(uint32_t jitter_dtu, uint32_t count, bool valid)
{
	if (!valid) {
		return "no-lock";
	}
	if (count < SYNC_GATE_MIN_COUNT) {
		return "insufficient";
	}
	if (jitter_dtu < SYNC_GATE_PASS_DTU) {
		return "pass";
	}
	if (jitter_dtu <= SYNC_GATE_FAIL_DTU) {
		return "marginal";
	}
	return "fail";
}

static int cmd_stats(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	struct sync_model *m = ccp_slave_model();
	uint32_t rx = 0, gap = 0, reject = 0, root = 0;

	ccp_slave_stats(&rx, &gap, &reject, &root);

	/* sync_model_error_dtu() returns UINT32_MAX when the model has no rate
	 * estimate. That is the documented public way to ask, rather than
	 * reaching into the struct's `valid` field. */
	bool valid = sync_model_error_dtu(m, 0u) != UINT32_MAX;

	uint32_t jit = sync_model_jitter_est_dtu(m);
	uint32_t cnt = sync_model_residual_count(m);

	/* 1 DTU = 15.65 ps. Printed because nobody reasons in DTU. */
	uint32_t jit_ps = (uint32_t)(((uint64_t)jit * 1565u) / 100u);

	shell_print(sh,
		    "{\"sync\":{\"root\":%u,\"rx\":%u,\"gaps\":%u,"
		    "\"rejected\":%u,\"count\":%u,\"valid\":%u,"
		    "\"jitter_est_dtu\":%u,\"jitter_est_ps\":%u,"
		    "\"rms_dtu\":%u,\"max_dtu\":%u,\"drift_ppb\":%d,"
		    "\"verdict\":\"%s\"}}",
		    root, rx, gap, reject, cnt, valid ? 1u : 0u,
		    jit, jit_ps,
		    sync_model_residual_rms_dtu(m),
		    sync_model_residual_max_dtu(m),
		    sync_model_drift_ppb(m),
		    verdict_of(jit, cnt, valid));

	if (valid && cnt >= SYNC_GATE_MIN_COUNT) {
		uint32_t mx = sync_model_residual_max_dtu(m);
		uint32_t rms = sync_model_residual_rms_dtu(m);

		/* Section 4's cross-check, applied rather than left to the
		 * operator: a max far above the RMS means the distribution has
		 * a tail, which is multipath or an intermittent obstruction --
		 * not clock noise. Fix the setup and re-measure before
		 * believing the verdict. */
		if (rms > 0u && mx > 5u * rms) {
			shell_warn(sh,
				   "max is %ux the RMS — that is a TAIL, not "
				   "Gaussian clock noise. Almost certainly "
				   "multipath or an intermittent obstruction. "
				   "Fix the setup and re-measure before "
				   "trusting the verdict.",
				   mx / rms);
		}
	}

	shell_print(sh,
		    "read jitter_est, NOT rms: the residual differences two "
		    "noisy timestamps and its RMS is ~1.55x the real jitter. "
		    "Thresholds: <%u DTU pass, <=%u marginal, above that fail.",
		    SYNC_GATE_PASS_DTU, SYNC_GATE_FAIL_DTU);
	return 0;
}

static int cmd_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	sync_model_residual_reset(ccp_slave_model());
	shell_print(sh, "residual statistics cleared — the rate estimate and "
			"its baseline are NOT touched, so `count` restarts "
			"while the lock is kept");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_sync,
	SHELL_CMD_ARG(stats, NULL,
		      "stats — the Phase 2 gate as JSON, with a verdict",
		      cmd_stats, 1, 0),
	SHELL_CMD_ARG(reset, NULL,
		      "reset — clear the residual statistics, keeping the lock",
		      cmd_reset, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(sync, &sub_sync, "Anchor clock synchronisation", NULL);
```

- [ ] **Step 5: agregarlo al build**

En `CMakeLists.txt`, en orden alfabético:

```cmake
	src/ccp_slave.c
```

y

```cmake
	src/sync_shell.c
```

- [ ] **Step 6: compilar ambas imágenes**

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
west build -b ancla_esp32s3/esp32s3/procpu -d build_dbg -- "-DEXTRA_CONF_FILE=debug.conf"
```

Esperado: exit 0 en las dos, sin warnings.

- [ ] **Step 7: verificar en hardware**

Flashear producción en el gateway **y** en un ancla SLAVE. En la consola del
slave:

1. `sync stats` inmediatamente tras el boot: `rx:0`, `valid:0`,
   `verdict:"no-lock"`.
2. Tras unos segundos: `rx` subiendo ~5 por segundo (un CCP por superframe de
   200 ms), `root` distinto de cero e igual al `root_id` que el gateway imprimió
   en su `{"ccp_master":...}`, `valid:1`.
3. `gaps` debe quedarse en 0 o muy bajo. Un `gaps` que sube tan rápido como `rx`
   significa que se está perdiendo uno de cada dos CCPs — revisa el enlace, no
   el modelo.
4. `rejected` debe ser 0. Si sube, el CCP llega pero no es utilizable: parse
   fallido o hop no adoptable.
5. `drift_ppb` debe caer dentro de ±40000 (la tolerancia combinada de dos
   cristales). Muy fuera de eso significa que una placa no corre el cristal que
   debería.

- [ ] **Step 8: registrar en CLAUDE.md**

En la sección "Consola", después del árbol `apos`:

```
El receptor de CCP agrega un árbol `sync`, en la imagen de **producción**. Solo
reporta; no transmite nada y no tiene comandos de configuración.

```
sync stats                     el gate de Fase 2 como JSON, con veredicto.
                               Lee `jitter_est`, NO `rms` — el residual
                               diferencia dos timestamps ruidosos y su RMS
                               vale ~1.55x el jitter real
sync reset                     limpia las estadísticas de residual sin tocar
                               el lock ni la línea base
```
```

En la sección "Layout", después de `src/ccp_sched.h`:

```
- `src/ccp_master.{c,h}` — la TX del CCP en el GATEWAY, raíz del árbol de
  sincronía (hop 0). En el gateway a propósito: ya agenda una TX retardada por
  superframe y el reloj desde el que la agenda **es** la base de tiempo. Un
  master en un slave habría añadido una ruta de TX no solicitada a una imagen
  de producción.
- `src/ccp_slave.{c,h}` — la RX del CCP en el SLAVE: dueño del único
  `struct sync_model`, detecta CCPs perdidos por huecos en `ccp_seq` y llama
  `sync_model_miss()` por cada uno. Sin aritmética propia — todo lo que puede
  estar mal del estimador vive en `sync_model.c`, que es C puro host-testeado.
- `src/sync_shell.c` — el árbol `sync`. Imprime el veredicto del gate él mismo
  porque el error natural (leer `rms` como si fuera el jitter) rechaza hardware
  que pasa.
```

- [ ] **Step 9: commit**

```bash
git add src/ccp_slave.c src/ccp_slave.h src/sync_shell.c src/uwb_slave.c CMakeLists.txt CLAUDE.md
git commit -F - <<'EOF'
feat(sync): receive CCPs on the slave, and make the Phase 2 gate readable

ccp_slave.c holds no arithmetic of its own. Everything that could be wrong
about the estimator is in sync_model.c, which is pure C and host-tested; what
lives here is the part a host test cannot reach -- a real RX timestamp from a
real DW3220 -- plus the sequence bookkeeping that tells the model when an
expected CCP did not arrive.

Three cases the bookkeeping has to get right, none of them obvious:

- A gap in ccp_seq is N-1 misses, fed one at a time, because that is what the
  model's coasting logic counts. Past SYNC_MISS_MAX + 1 the estimate is invalid
  whatever we do, so a fresh baseline beats hundreds of miss calls on the SLAVE
  loop.
- A duplicate sequence number is neither a miss nor an observation. Folding it
  in would count one interval as two and bias the rate.
- A different root_id is a different time base, and the baseline built against
  the old one describes nothing. Start over rather than mixing two clocks.

rx_ts is passed as the full 40-bit value. Handing sync_model a hi32 would throw
away the bottom 8 bits -- 255 DTU, ~4 ns, against a 1 ns gate.

`sync stats` prints the verdict itself, and that is the point rather than
polish. sync_model.h records that the residual RMS is about 1.55x the real
jitter, because the prediction consumes two noisy timestamps. An operator
reading 64 DTU of RMS as 1 ns would be looking at ~0.65 ns and rejecting
hardware that passes -- killing the migration on a misinterpretation. So
jitter_est comes first, the verdict is computed here, and rms sits beside it as
a diagnostic. It also refuses a verdict below 400 observations and applies
section 4's max/rms tail check instead of leaving it to the reader.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
EOF
```

---

## Task 4: cerrar el lazo — el documento del gate deja de mentir, y se corre

**Files:**
- Modificar: `docs/anchor-sync-measurement.md` (§2 y §3)
- Modificar: `docs/superpowers/specs/2026-08-25-rtls-scale-tdoa-design.md` (fila A7)

**Interfaces:**
- Consume: todo lo de las tareas 1–3.
- Produce: nada de código. El entregable es un documento ejecutable y un número.

- [ ] **Step 1: corregir §2 de `docs/anchor-sync-measurement.md`**

La sección se titula "What is implemented, and what is not" y hoy lista cuatro
piezas como no implementadas. Reemplazar la tabla y la lista de faltantes por:

```markdown
**Implemented and host-tested:**

| Piece | Where |
|---|---|
| The sync arithmetic, error budget, residual statistics | `src/sync_model.{c,h}`, `tests/sync_model/` |
| The CCP wire format | `src/ccp_frame.{c,h}`, `tests/ccp_frame/` |
| Where the CCP sits in the superframe | `src/ccp_sched.h`, `tests/ccp_sched/` |

**Implemented, and verified on hardware:**

| Piece | Where |
|---|---|
| Master role (gateway, hop 0) | `src/ccp_master.{c,h}` |
| Slave role, gap detection, model ownership | `src/ccp_slave.{c,h}` |
| `sync stats` / `sync reset` | `src/sync_shell.c` |

The CCP goes in the **post-beacon guard window**, where slaves already may not
transmit, so it costs **no** airtime from the CAP or the CFP —
`CCP_OFFSET_UUS` is `BEACON_OCCUPANCY_UUS` and two `BUILD_ASSERT`s in
`src/ccp_sched.h` prove the preamble does not overlap the beacon and the frame
is off the air before the guard closes. Its airtime is **1.289 ms, 0.645 % of a
200 ms superframe**.

**Role selection is deliberately not runtime-configurable.** The master is the
gateway, because putting an unsolicited transmit path in a production SLAVE
image is the collision hazard `apos_node.c`'s gates exist to prevent. To repeat
the measurement with the roles swapped (§3), swap the **boards**: `anchor mode
gateway` on the other one, then `kernel reboot cold`.
```

- [ ] **Step 2: corregir §3 de `docs/anchor-sync-measurement.md`**

El encabezado dice "Procedure, once the glue above exists". Cambiarlo a
"Procedure" y reemplazar los pasos por:

```markdown
1. Flash **production** on both boards. One is `anchor mode gateway` (the CCP
   master, hop 0), the other `anchor mode slave` (the receiver).
2. On the gateway, confirm `{"ccp_master":{"root_id":N,...}}` at boot and note
   `root_id`.
3. On the slave, `sync stats`. Confirm `rx` climbing by ~5 per second, `root`
   equal to the gateway's `root_id`, and `valid:1` within a couple of seconds.
   `gaps` and `rejected` should both stay at or near 0.
4. `sync reset`, then leave it alone for **at least 90 seconds** — the verdict
   is withheld below 400 observations on purpose, and at 5 per second that is
   80 s.
5. `sync stats`. **Read `verdict` and `jitter_est_ps`.** Do not read `rms_dtu`
   as the jitter; §0.2 explains why that misleads in the confident direction.

Repeat with the boards swapped. A large asymmetry between the two directions
points at one board, not at the technology — the same logic as swapping
`anchor id` to separate a firmware fault from a board fault.
```

- [ ] **Step 3: correr el gate**

Seguir §1 (prerequisitos) y §3 (procedimiento) de
`docs/anchor-sync-measurement.md`. Anotar en §4 del documento la salida completa
de `sync stats` de las dos direcciones, con fecha.

**Este paso es la tarea.** Las tres anteriores solo existen para hacerlo
posible.

- [ ] **Step 4: registrar el resultado y decidir**

Según §4 del documento:

| `verdict` | Qué se hace |
|---|---|
| `pass` (< 32 DTU, < 0.5 ns) | La Fase 3 procede. El plan ya está escrito: `docs/superpowers/plans/2026-08-25-fase3-tdoa.md`. |
| `marginal` (32–64 DTU) | Re-medir a otra distancia y orientación antes de decidir. Si aguanta, procede **solo** después de costear los remedios de §5. |
| `fail` (> 64 DTU, > 1 ns) | La Fase 3 **no procede como está diseñada**. Trabajar §5 en orden de costo: primero el arreglo del setup, luego el intervalo de CCP, luego `SYNC_PHASE_EMA_SHIFT`. |

Actualizar la fila A7 del spec de `parcial` a `sí`, y agregar el número medido a
`CLAUDE.md` junto a la entrada de `src/sync_model.{c,h}`.

- [ ] **Step 5: commit**

```bash
git add docs/anchor-sync-measurement.md docs/superpowers/specs/2026-08-25-rtls-scale-tdoa-design.md CLAUDE.md
git commit -F - <<'EOF'
docs(sync): the Phase 2 gate is now executable, and here is the number

Section 2 no longer lists four pieces as unimplemented, and section 3 no longer
opens with "once the glue above exists". Records the measured jitter from both
board directions, and the decision it drives.

Also records what the placement bought: the CCP lives in the post-beacon guard
where slaves already may not transmit, so it costs no CAP or CFP airtime at
all -- 1.289 ms, 0.645% of a superframe, with two BUILD_ASSERTs holding the
budget.

And why role selection is not runtime-configurable: the master is the gateway
because an unsolicited transmit path in a production SLAVE image is the
collision hazard apos_node.c's gates exist to prevent. Swapping the roles for
the second measurement means swapping the BOARDS.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
EOF
```

---

## Auto-revisión

Corrida contra el spec con ojos frescos, tras escribir el plan.

**1. Cobertura del spec.** A7 dice *"Frame CCP + emisión/recepción entre anclas;
plan de medición en hardware para el gate de Fase 2"*. El frame ya existía
(`ccp_frame`, commit `e00edef`). Emisión → Task 2. Recepción → Task 3. Plan de
medición en hardware → ya existía como documento y la Task 4 lo vuelve
ejecutable y lo corre. Las cuatro piezas que
`docs/anchor-sync-measurement.md` §2 lista como faltantes quedan cubiertas: rol
master (T2), rol slave (T3), comando `sync` (T3), selección de rol (D1, resuelta
por diseño en vez de por código). **Sin huecos.**

**2. Marcadores de posición.** Ninguno. Todo bloque de código es completo y
compilable; los dos `BUILD_ASSERT` fueron verificados numéricamente antes de
escribirse (márgenes +98 856 ns y +1 299 638 ns); las constantes impresas en los
tests son valores medidos, no aproximaciones.

**3. Consistencia de tipos.** `ccp_master_after_beacon(uint64_t, uint8_t *)`
coincide con la llamada de la Task 3 en `uwb_gateway.c`.
`ccp_slave_on_rx(const uint8_t *, uint16_t, uint64_t)` coincide con el sitio de
despacho. `ccp_slave_model()` devuelve `struct sync_model *` **no const**,
porque `sync_model_residual_reset()` toma un puntero mutable — verificado contra
`sync_model.h:233`. `ccp_slave_stats()` y `ccp_master_stats()` tienen cuatro y
tres parámetros de salida respectivamente y las dos llamadas del shell coinciden.

**Tres cosas que la revisión cambió:**

- `ccp_seq` se incrementa al **construir**, no al confirmar. El primer borrador
  lo incrementaba al éxito, que oculta el superframe salteado y hace que el
  receptor pliegue dos intervalos como uno. Lo correcto es dejar el hueco
  visible, y es la misma convención que `frame_seq_nb` en la ruta del responder.
- El **redondeo a hi32** del RMARKER. La primera versión ponía en el payload el
  valor sin redondear. El registro de TX retardada se programa en hi32, así que
  el hardware redondea hacia abajo a un múltiplo de 256 DTU: el payload habría
  mentido por hasta 255 DTU (~4 ns) en **cada** observación, contra un umbral de
  gate de 1 ns. La medición habría fallado pareciendo un veredicto de hardware.
- El assert (a) compara contra el **fin del beacon más el SHR del CCP**, no solo
  contra el fin del beacon. Comparar el RMARKER habría pasado mientras 1050 µs
  de preámbulo se sentaban encima del payload del beacon — una colisión que un
  sniffer muestra como beacon corrupto y que nada atribuiría al CCP. Es el mismo
  error de contar el SHR que ya se pagó una vez en `MAC_SSTWR_EXCHANGE_PS`.

**Una cosa que este plan NO hace, dicha en vez de escondida:** no implementa el
árbol de sincronía de producción (raíz → masters en hop 1 → hojas en hop 2,
`CCP_HOP_MAX`). `ccp_frame` ya lleva el campo `hop` y `ccp_frame_parse()` ya
rechaza un hop no adoptable, así que el formato de aire está listo; lo que falta
es que un slave **retransmita** como master en hop 1. Eso es trabajo de Fase 3 o
posterior, requiere coordinación por el backhaul WiFi (que es lo que la decisión
del backhaul compró), y meterlo aquí retrasaría el gate que decide si algo de
esto vale la pena. El gate necesita dos placas y un hop.
