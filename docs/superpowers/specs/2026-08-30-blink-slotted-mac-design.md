# Tarea 4B — acceso ranurado para BLINK a escala — diseño

**Contexto:** `docs/superpowers/plans/2026-08-25-fase3-tdoa.md`, Tarea 4B (stub) y
su sección "Lo que T4B necesita antes de poder escribirse" (medido
2026-08-28, después de que la Tarea 7 puso el BLINK en el aire). Ese documento
decidió deliberadamente NO implementar T4B desde sí mismo: "Requiere su propio
ciclo de diseño (spec + plan), como el resto." Este documento es ese ciclo.

**Objetivo:** cobrar el ahorro de airtime que el BLINK ya demostró en
hardware (Tarea 7) pero que hoy no se traduce en capacidad de red: un BLINK de
1.223 ms ocupa la misma ranura de 13.32 ms que un barrido TWR completo, así
que la red sigue limitada a los mismos ~11 tags de siempre. Objetivo de
producto: 100 tags a 5 Hz (§1 de
`docs/superpowers/specs/2026-08-25-rtls-scale-tdoa-design.md`).

---

## 0. Lo que este documento NO reabre

Por la regla de este proyecto de no re-litigar decisiones ya tomadas:

- **El acceso es ranurado, no aleatorizado** (Decisión 1 del stub). Con acceso
  aleatorio la probabilidad de colisión por blink a 100 tags es 9-46% según el
  espacio elegido — inaceptable a cualquier tamaño razonable.
- **La asignación de slot es implícita; el beacon no crece un mapa explícito**
  (Decisión 2 del stub). Un mapa de ~110-114 entradas son 220-228 bytes,
  por encima del máximo de 802.15.4 (127 bytes) y muy por encima de
  `UWB_FRAME_LEN_BEACON` actual (37 bytes a 11 slots).
- **El BLINK se programa como TX diferido en DTU desde el RX del beacon**, no
  desde el reloj de milisegundos del kernel (Hallazgo 2 del stub, medido). El
  reloj de ms por sí solo limita la ganancia a ~4x en vez de ~10x.
- **`pos_solver`/`pos_ekf` del tag no se tocan aquí.** Esta tarea es MAC pura;
  no toca el solve.

---

## 1. Qué SÍ resuelve este documento

Las cinco preguntas abiertas que el stub dejó explícitas, en orden de cuánto
determinan a las demás.

### 1.1 La forma de `f(seat_id, sf_counter, tier)`, y su prueba de no-colisión

**Decisión: el CFP se REPARTICIONA, no se extiende.** Los mismos ~146.5 ms que
hoy ocupan los 11 slots de ranging TWR se re-subdividen en `BLINK_N_SLOTS`
slots de blink. `T_SUPERFRAME_UUS`, el beacon y el CAP **no cambian** — la
única superficie que cambia es la estructura interna del CFP. Esto evita la
alternativa (añadir una región de blink *además* de la CFP existente), que
habría alargado el superframe y arrastrado cada presupuesto de tiempo que
asume 200 ms: `SYNC_CCP_INTERVAL_DTU`, la predicción de `beacon_guard`, y
cada `BUILD_ASSERT` de `uwb_mac.h`/`mac_budget.h`. Repartir en vez de sumar
mantiene esa superficie intacta.

**Consecuencia estructural: una celda corre en modo TWR o en modo BLINK, no
los dos a la vez.** El CFP no puede estar simultáneamente subdividido en 11
slots de 13.32 ms y en ~110 slots de ~1.3 ms — son dos particiones distintas
del mismo intervalo de tiempo. Un tag que sigue en `blink off` (TWR) no puede
rangear en una celda cuyo CFP ya se reparticionó a slots de blink. Esto no es
una limitación nueva inventada aquí: es exactamente la misma forma que tomó
`proto_ver` 3 (día-bandera, ambos firmwares son nuestros, reflasheo
coordinado) y es consistente con §7 del spec de escala: TDoA **reemplaza** el
rol de ranging, no lo optimiza. La coexistencia real de TWR y BLINK en una
misma celda es exactamente lo que la Fase 5 ("anclas como relevo de downlink;
TWR conservado en el perímetro") ya reserva para más adelante — no se
resuelve aquí.

**La función de slot es la identidad sobre `seat_id`, sin depender de
`sf_counter` ni de qué otros asientos estén activos:**

```
slot_index(seat_id) = seat_id,   válido cuando seat_id < BLINK_N_SLOTS
```

Por qué esto es válido y suficiente:

- `seat_id` (0..`GW_MAX_SEATS`-1 = 0..127) ya es el índice del arreglo
  `seats[]` en `gw_core.c` — un entero pequeño, estable mientras el asiento
  vive, y ya viaja al tag en el GRANT (`struct gw_grant.seat_id`). No hace
  falta ningún dato nuevo en el aire.
- Si `BLINK_N_SLOTS >= GW_MAX_SEATS`, la asignación `slot = seat_id` es
  **inyectiva por construcción sobre TODO el espacio de asientos**, para
  cualquier mezcla de tiers, sin importar cuáles asientos estén vivos ni
  cuáles due este superframe. No hace falta ninguna coordinación entre
  asientos ni ningún cómputo global — cada tag calcula su propio slot leyendo
  su propio `seat_id`, punto. Ésa es la prueba de no-colisión que el stub
  pedía: es trivial precisamente porque el dominio de slots cubre el dominio
  de asientos.
- `sf_counter` sigue gobernando **si** un asiento transmite este superframe
  (el mecanismo de tier/lease existente, sin cambios — ver §1.5), pero no
  **dónde**. Separar "cuándo" de "dónde" es lo que hace que `f()` no necesite
  ver el estado de otros asientos.

**El caso `BLINK_N_SLOTS < GW_MAX_SEATS` (banda de desborde) — política v1:
NO ADMITIR asientos con `seat_id >= BLINK_N_SLOTS` en una celda de blink.**
Esta banda es ya HIPOTÉTICA en este build: con el guard medido (§1.2),
`BLINK_N_SLOTS = 144 > GW_MAX_SEATS = 128`, así que la política de rechazo
descrita abajo no se ejercita hoy -- queda documentada porque una guarda más
grande (o un `GW_MAX_SEATS` mayor) podría volver a cruzar ese umbral. En vez
de inventar un esquema de multiplexado por fase para
la banda de desborde (coloreo de intervalos sobre 3 periodos de tier
distintos — un problema real, no trivial), la política v1 es que
`gw_core_join()` en modo blink **rechaza** una solicitud de asiento cuyo único
`seat_id` libre cae en `>= BLINK_N_SLOTS`. Efectivamente:

```
GW_MAX_SEATS_BLINK = min(GW_MAX_SEATS, BLINK_N_SLOTS)
```

**El costo honesto, actualizado con el guard medido (§1.2):** con
`BLINK_N_SLOTS = 144 > GW_MAX_SEATS = 128`, `min()` arriba se resuelve a 128 --
el techo de presencia garantizada NO baja en este build; sigue siendo el mismo
128 que la Fase 1a garantiza en modo TWR, y `GW_MAX_SEATS` (el tamaño físico
de `seats[]`, no el guard) es ahora el término que limita, no
`BLINK_N_SLOTS`. Esto era una posibilidad real antes de medir -- con el
guard provisional de 200 us, `BLINK_N_SLOTS` caía en 96-114, por debajo de
128, y la banda de desborde de abajo sí se habría ejercitado. Se deja la
política v1 escrita porque una guarda más floja o un `GW_MAX_SEATS` mayor
podrían volver a cruzar ese umbral. Refinar el desborde con multiplexado por
fase queda
explícitamente diferido — no se hace aquí, igual que §7 del spec de escala
difiere otras optimizaciones — y solo vale la pena si un despliegue real
necesita más asientos simultáneos que `BLINK_N_SLOTS`.

### 1.2 El valor de `BLINK_SLOT_GUARD_UUS`

**Actualizado 2026-09-01 con datos de banco reales — ya no es el valor de
arranque provisional.** La Tarea 2 de este plan midió el jitter de armado del
TAG en hardware; este documento se actualiza junto con el código, por su
propia regla de no divergir.

Entradas conocidas (Decisión 1 del stub):
- Jitter de armado de TX del **gateway** (ANCLA, ESP32-S3): 64 us, medido en
  hardware.
- Deriva de cristal del tag sobre un superframe de 200 ms a 40 ppm: ~8 us,
  despreciable frente a lo anterior.
- **Jitter de armado de TX del TAG (nRF52833), ahora medido**: `tools/blink_jitter.py`
  contra `COM7_2026_09_01.12.43.19.533.txt`, tres tags reales forzados a
  seat_id 120/121/122 vía `CONFIG_ANCLA_DEBUG_FORCE_HIGH_SEAT` (para ejercer
  la cola de riesgo del redondeo a ms que gw_core.h documenta en la nota de
  la Tarea 5, no solo los seat_id bajos que una flota pequeña ocupa por
  defecto). >=730 muestras por tag, spread máximo entre los tres:
  **39.2 ns** — cuatro órdenes de magnitud por debajo de los dos términos ya
  conocidos, y por tanto no dominante.

Suma de términos conocidos/medidos: 64 + 8 + 0.04 us ≈ **72 us**.

Valor fijado: **`BLINK_SLOT_GUARD_UUS = 100`** (0.1 ms) — redondeado
hacia arriba desde la suma de ~72 us para dejar margen, no la cifra cruda, y
por debajo del punto medio provisional de 200 us que este documento fijaba
antes de que hubiera cualquier medición del lado del tag. Con el modelo de
`mac_budget.h` (ver §1.4) y la tabla PHY congelada, esto da
**`BLINK_N_SLOTS` = 144** (`tests/blink_sched/`, medido, no estimado).

El jitter del tag se midió solo en la cola de asientos (120-122 de 128); no
se ha medido en asientos bajos ni se espera que difiera — `blink_sched_slot_index()`
es la identidad, y nada en el armado TX depende de qué seat_id se trate — pero
se anota la limitación en vez de asumirla.

### 1.3 `proto_ver`: sí sube, y el mecanismo ya existe

**Decisión: nueva versión, `UWB_PROTO_VER = 4`.** El formato del beacon NO
cambia — `UWB_FRAME_LEN_BEACON` sigue siendo `15 + 2 * UWB_FRAME_N_CFP` y
`sched[]` sigue teniendo `UWB_FRAME_N_CFP` (11) entradas — pero su
**significado** en una celda de blink sí cambia: `sched[]` se transmite
reservado/cero (nadie lo lee) y la asignación real viene de `slot_index()`
(§1.1). Esto es exactamente la misma clase de cambio que ya justificó subir
`proto_ver` de 2 a 3 (`gw_core.h`: "el mapa pasó de tabla de propiedad a
calendario... el formato NO cambió, lo que cambió es el significado").

**El mecanismo de rechazo ya existe y no hay que inventarlo:**
`tag_testting/src/uwb_net.c` solo acepta un beacon en SCAN/JOINING cuando
`ev->proto_ver == UWB_NET_PROTO_VER` exacto (`uwb_net.c:250`, `:289`). Un tag
en `UWB_NET_PROTO_VER = 3` contra un gateway en modo blink (`proto_ver = 4`)
simplemente nunca sale de SCAN — sordo, no roto, el mismo comportamiento
documentado para el salto 2→3. Consecuencia operativa: pasar una celda a modo
blink es un evento de reflasheo coordinado (ambos firmwares son nuestros),
igual que la vez anterior.

### 1.4 Convivencia con el CAP: reusar `mac_budget.h`, no re-derivar

**Decisión: `mac_cell_max_slots()` ya resuelve esto — se reusa, no se
reescribe.** La función existe exactamente para "cuántos slots de `slot_ns`
caben en el tramo utilizable después de beacon, guardas y CAP"
(`src/mac_budget.h:174-179`). El único cambio es el `slot_ns` que se le pasa:
hoy recibe el airtime de un intercambio SS-TWR completo
(`MAC_SSTWR_EXCHANGE_PS`, 13.32 ms); en modo blink recibe el airtime de un
BLINK más su guarda:

```
blink_slot_ns = mac_frame_ns(&phy_frozen, BLINK_FRAME_LEN) + MAC_UUS_TO_NS(BLINK_SLOT_GUARD_UUS)
BLINK_N_SLOTS = mac_cell_max_slots(&cell, blink_slot_ns)
```

`mac_frame_ns()` ya existe y ya usa el PHY congelado; `BLINK_FRAME_LEN` (14
bytes) ya existe en `src/blink_frame.h` desde la Tarea 1. Nada nuevo que
inventar en `mac_budget.h` salvo, opcionalmente, una constante nombrada
`MAC_BLINK_SLOT_PS` si conviene fijarla en un `BUILD_ASSERT` como con
`MAC_SSTWR_EXCHANGE_PS` — decisión de implementación, no de diseño.

El CAP (JOIN/GRANT/KEEPALIVE) no cambia: sigue ocupando su tramo de
minislots antes del CFP, sin relación con cuántos slots de blink caben
después. No hay conflicto que resolver aquí porque `mac_cell_usable_ns()` ya
descuenta el CAP antes de calcular cuántos slots caben.

### 1.5 Qué pasa con un asiento cuyo lease expiró

**Ya está resuelto — desde `proto_ver` 3, y esta tarea no lo toca.** El hecho
ya documentado en `CLAUDE.md` ("Un tag ausente del mapa del beacon significa
'no te toca', NO 'te reclamaron el asiento' — desde `proto_ver` 3") es
precisamente la separación que hace esta pregunta un no-problema: la pérdida
de asiento se detecta por el ciclo de KEEPALIVE/renovación (mitad de
`GW_LEASE_SF`), no por ausencia en un mapa explícito. Un tag de blink sigue
renovando su lease exactamente igual que uno de TWR — `gw_core_keepalive()`
no cambia. Lo único que `slot_index()` decide es DÓNDE transmite un asiento
que YA sabe, por su propio conteo de lease, que sigue vigente. Si el gateway
reclama el asiento (lease agotado sin renovación), el tag deja de recibir
GRANT/ACK a su próximo KEEPALIVE y reingresa por JOIN — el mismo camino que
hoy, sin cambios.

---

## 2. Lo que cambia, archivo por archivo

| Archivo | Cambio |
|---|---|
| `src/blink_sched.{c,h}` (nuevo, ANCLA) | `BLINK_SLOT_GUARD_UUS` (provisional, §1.2), `blink_sched_n_slots()` (envuelve `mac_cell_max_slots()`, §1.4), `blink_sched_slot_index(uint8_t seat_id)` (§1.1), `blink_sched_seat_admissible(uint8_t seat_id)` (la política de desborde v1, §1.1). Puro C, host-testeable. |
| `src/gw_core.{c,h}` | `gw_core_join()` gana un modo (o un parámetro) que aplica `blink_sched_seat_admissible()` antes de asignar `seat_id`, en vez de tomar el primer libre. Reinterpretación, no reescritura — `gw_core_build_slotmap()` sigue existiendo para el modo TWR sin cambios. |
| `src/uwb_frame_802_15_4z.h` | `UWB_PROTO_VER` sube a 4 (ANCLA). Formato del beacon sin cambios de bytes. |
| `src/uwb_gateway.c` | Selecciona modo TWR o blink de la celda (config de build o NVS, a decidir en el plan) y arma el CFP en consecuencia; en modo blink no escribe direcciones reales en `sched[]`. |
| `tag_testting/src/uwb_net_runner.c` | El TX del BLINK deja de programarse desde `t0_ms` (reloj de ms) y pasa a TX diferido en DTU desde el RX timestamp del beacon (Tarea 1 del plan). El offset del slot usa `blink_sched_slot_index(seat_id)` en vez de `ctx.tx_slot`. |
| `tag_testting/src/uwb_net.h`/`.c` | `UWB_NET_PROTO_VER` sube a 4 en el firmware que soporta blink-CFP. Nada más cambia en la FSM — §1.5. |

---

## 3. Riesgo nuevo y explícito: primer TX diferido del tag, nunca antes ejercido

`tag_testting` no ha usado **nunca** `DWT_START_TX_DELAYED` — cada transmisión
en el árbol (`uwb_ss_initiator.c`, `uwb_net_runner.c`, `cal_run.c`,
`cal_diag.c`) es `DWT_START_TX_IMMEDIATE`. La Tarea 1 introduce el primer TX
diferido de este firmware, en un puerto (nRF52833, glue de radio propio, no
el módulo vendorizado de Zephyr que usa ANCLA). El patrón a copiar
(`anchor_respond.c`'s `poll_rx_ts + turnaround`, `ccp_master.c`'s
`dwt_forcetrxoff()` antes de armar) es del lado ANCLA, sobre un stack Zephyr
distinto. Esto NO es una copia mecánica de código — es la primera vez que se
ejerce esta clase de operación en este puerto, y el propio historial de este
proyecto (la trampa `DX_TIME - SHR` costó dos ciclos de banco la primera vez,
en un puerto que YA tenía TX diferido en otros lugares) es la razón por la
que la Tarea 1 del plan se trata como su propio ciclo de hardware, con
medición explícita antes de construir nada encima.

---

## 4. Lo que este documento NO establece

- El valor final de `BLINK_SLOT_GUARD_UUS` — depende de una medición de
  hardware no hecha todavía (Tarea 2 del plan).
- El mecanismo exacto por el que el gateway elige modo TWR vs. modo blink
  para una celda (Kconfig de build vs. campo NVS vs. comando de consola) — es
  una decisión de implementación menor, para el plan, no para este diseño.
- Cualquier forma de coexistencia TWR+BLINK dentro de una misma celda —
  diferido a la Fase 5 del spec de escala.
- Multiplexado por fase para la banda de desborde (`seat_id >= BLINK_N_SLOTS`)
  — diferido, ver §1.1.
