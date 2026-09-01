# Tarea 4B — acceso ranurado para BLINK a escala

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** que el ahorro de airtime del BLINK (10.9x sobre un barrido TWR) se
traduzca en capacidad de red real, no solo en batería del tag. Hoy el BLINK
ocupa la misma ranura de 13.32 ms que el barrido TWR que reemplaza — la red
sigue limitada a ~11 tags. Al final de este plan, una celda en modo blink
admite ~100-114 asientos y cada uno transmite en su propio slot sin colisión,
sin que el beacon crezca ni un byte.

**Spec:** `docs/superpowers/specs/2026-08-30-blink-slotted-mac-design.md` —
léase completo antes de empezar. Resuelve las cinco preguntas que
`docs/superpowers/plans/2026-08-25-fase3-tdoa.md` dejó abiertas bajo T4B.
Las decisiones de ahí (reparticionar el CFP en vez de extenderlo, celda en
modo TWR o modo blink pero no ambos, `slot_index(seat_id) = seat_id`, reusar
`mac_cell_max_slots()`) están tomadas y no se re-derivan aquí.

**Condicionalidad:** hereda las de
`docs/superpowers/plans/2026-08-25-fase3-tdoa.md` — el gate de Fase 2 corrió y
falló contra 1 ns, la Fase 3 procede por decisión de producto a ~45 cm. Este
plan no reabre esa decisión: es MAC pura, no toca sincronía ni el solve.

**Orden de dependencias entre tareas:**

```
Tarea 1 (tag, hardware)  ──┐
                            ├──> Tarea 5 (tag, hardware) ──> Tarea 6 (hardware, ambos repos)
Tarea 3 (ANCLA, host) ──> Tarea 4 (ANCLA) ──┘
Tarea 2 (tag, hardware, mide) ──> informa el valor final de BLINK_SLOT_GUARD_UUS en Tarea 3/4
```

Las Tareas 1 y 3 no tienen dependencia entre sí y pueden avanzar en paralelo
en los dos repos. La Tarea 2 necesita la Tarea 1 terminada (medir jitter de
un TX que todavía se programa en reloj de ms no dice nada del hardware). La
Tarea 5 necesita la 1 (mecanismo de TX) y la 4 (que exista un modo blink del
lado del gateway contra el cual probar). La Tarea 6 es el cierre de hardware
end-to-end y necesita todo lo anterior.

**Estado 2026-08-31:** Tareas 1, 3, 4 y 5 escritas y compiladas/host-testeadas
esta sesión (ninguna tocó hardware). Detalle completo en el `CLAUDE.md` de
ANCLA, sección TDoA §3. Resumen: `BLINK_N_SLOTS = 134` (Tarea 3),
`UWB_PROTO_VER`/`UWB_NET_PROTO_VER` en 4 y `anchor cell twr|blink` (Tarea 4),
TX diferido en DTU y slot por `seat_id` escritos en el tag pero SIN verificar
en banco (Tareas 1 y 5, con una brecha conocida documentada: los seat_id
~132-133 de 134 exceden el superframe por el redondeo a ms de las constantes
heredadas de TWR). Tareas 2 y 6 (medición y verificación de hardware) siguen
sin empezar — nada de lo anterior se ha ejecutado en un board real.

---

## Task 1: El tag programa el BLINK como TX diferido en DTU *(tag_testting, hardware)*

**Por qué primero:** es la precondición de todo lo demás (Hallazgo 2 del
stub) — sin esto, cualquier medición de jitter mide la cuantización del reloj
de milisegundos, no el hardware, y `BLINK_SLOT_GUARD_UUS` no puede fijarse con
un número real.

**Riesgo explícito:** primer uso de `DWT_START_TX_DELAYED` en este firmware —
ver spec §3. Tratar como su propio ciclo de banco, no como una copia mecánica
del patrón de `anchor_respond.c`/`ccp_master.c` del otro repo (puerto de radio
distinto, nunca ejercido en esta ruta).

**Files:**
- Modify: `src/uwb_net_runner.c` (el sitio de TX del BLINK, hoy programado
  desde `ctx.tx_slot * (T_SLOT_MS + T_GUARD_MS)` sobre `t0_ms`)
- Modify: dondequiera que `uwb_radio_rx_beacon()` esté implementado, para
  exponer el RX timestamp de 40 bits del DW3000 del beacon recién recibido
  (hoy solo se usa `uwb_radio_now_ms()` — reloj de kernel, no del radio)
- Puede requerir: `src/uwb_ss_initiator.h`/`.c` si el timestamp de RX se
  expone por ahí, siguiendo el precedente de `dwt_readrxtimestamplo32()` ya
  usado en ese archivo para las respuestas SS-TWR

**Interfaces (a confirmar contra el puerto real, no asumir del otro repo):**
- Consumes: `dwt_readrxtimestamp()` (40 bits completos — el de 32 bits que
  usa hoy `uwb_ss_initiator.c` no alcanza para un TX diferido, que necesita
  la base de tiempo completa), `dwt_setdelayedtrxtime()`,
  `dwt_starttx(DWT_START_TX_DELAYED)`, `dwt_forcetrxoff()`.
- Produce: el BLINK sale con su RMARKER a un offset FIJO en DTU después del
  RMARKER del beacon, en vez de a un offset variable en milisegundos después
  de que el software terminó de procesar el beacon.

- [x] Confirmar qué función del puerto (`port.c`, o donde viva
      `uwb_radio_rx_beacon()`) puede devolver el RX timestamp de 40 bits del
      beacon, y exponerla si no existe ya. No inventar una ruta nueva de SPI
      si la ISR de RX ya deja el dato disponible — mirar cómo
      `uwb_ss_initiator.c` lo hace para sus propias respuestas primero.
      Hecho: `uwb_radio_rx_beacon()` en `uwb_net_runner.c` ahora captura
      `dwt_readrxtimestamp()` en cada RX exitoso, expuesto vía
      `uwb_radio_last_rx_ts40()`.
- [x] Calcular el offset de armado: `DX_TIME` objetivo menos el SHR completo
      de la trama de BLINK (a PLEN_1024, 1 050 194 ns) — el plazo de armado
      real es contra el PREÁMBULO, no contra el RMARKER. Ver `CLAUDE.md`,
      la entrada sobre `DX_TIME - SHR` y la de "delayed TX armado
      inmediatamente después de otro TX" para las dos formas en que esto ya
      falló al 100% la primera vez que se escribió en el otro repo.
      Escrito siguiendo el patrón ya existente en
      `examples/uwb_ds_initiator.c` (`(rx_ts + offset_ticks) >> 8`); el
      margen contra el preámbulo en sí se midió en la Tarea 2 (2026-09-01):
      spread máximo 39.2 ns, muy por debajo del margen disponible.
- [x] Llamar `dwt_forcetrxoff()` antes de armar el TX diferido — el RX del
      beacon que acaba de completarse deja el Transmit Sequencing Engine en
      un estado que un `dwt_starttx()` inmediatamente posterior puede no
      tolerar sin él. Confirmar en hardware si este puerto tiene el mismo
      comportamiento que el DW3000 del otro repo antes de asumirlo.
      Llamada añadida en `blink_publish()`; confirmado en hardware por la
      Tarea 2 (BLINKs limpios, sin fallo de armado observado) -- la
      verificación end-to-end restante es Tarea 6.
- [x] Mantener, por ahora, el offset de slot ACTUAL (`ctx.tx_slot`) para el
      cálculo del `DX_TIME` — esta tarea cambia CÓMO se programa el TX, no
      DÓNDE cae el slot. Eso es la Tarea 5. Mezclar los dos cambios hace
      imposible saber cuál rompió qué si algo falla en el banco.
- [ ] Verificar en un sniffer que el BLINK sigue apareciendo, sin cambios de
      formato, y que su RMARKER cae dentro de la ranura esperada (con
      margen de sobra, ya que la guarda real todavía no se ha fijado).
      PENDIENTE — requiere hardware.
- [x] Commit. Hecho 2026-09-01 en `tag_testting`
      (`fix(net): blink-mode tags now honor tier cadence instead of never TXing`,
      branch `feat/rtls-scale-tdoa`) — junto con el bug de tier encontrado en
      banco (ver nota debajo) y el resto de Tarea 5.

---

## Task 2: Medir el jitter residual del instante de TX del tag *(tag_testting, hardware, medición)*

**Solo interpretable después de la Tarea 1** — antes de eso mediría la
cuantización de milisegundos, no el hardware (Hallazgo 2 del stub).

**Preferir un sniffer externo a instrumentación nueva en firmware.** Este
proyecto ya tiene una lección cara sobre instrumentación temporal que se
queda más de lo previsto (`pos_dbg.c`, retirado en la limpieza de la Tarea 8
de `docs/superpowers/plans/2026-08-25-fase3-tdoa.md` sin que su campaña de
captura llegara a correr nunca). Si un sniffer ya usado en este proyecto
(DWM3001CDK, ver `docs/antenna-delay-calibration.md`) puede capturar el
RMARKER real del BLINK en varias superframes consecutivas, es preferible a
añadir un contador o un log nuevo al firmware del tag.

- [x] Con la Tarea 1 en un tag, capturar N >= 200 BLINKs consecutivos con el
      sniffer y medir la dispersión del RMARKER real respecto al instante
      programado (offset fijo en DTU desde el RMARKER del beacon).
      Hecho 2026-09-01: `tools/blink_jitter.py` contra
      `COM7_2026_09_01.12.43.19.533.txt`, tres tags reales forzados a
      seat_id 120/121/122 (`CONFIG_ANCLA_DEBUG_FORCE_HIGH_SEAT`, ver
      `gw_core.h`/Kconfig) para ejercitar la cola de riesgo del redondeo a
      ms de la Tarea 5, no solo los seat_id bajos que una flota de 3 tags
      ocuparía por defecto. n=873, 822 y 733 respectivamente (todos >= 200;
      una primera captura, `COM7_...12.17.37.616.txt`, quedó descartada por
      mezclar un RESCAN de uno de los tags a mitad de captura -- spread
      bimodal de ~10 ms en esa dirección, señal clara de dos asignaciones
      de slot distintas dentro de la misma dirección de origen).
- [x] Registrar el resultado: media, desviación, peor caso. Comparar contra
      los 64 us de jitter de armado ya medidos en el ANCLA (gateway) — la
      pregunta que este número contesta es si el tag (nRF52833, puerto de
      radio distinto) se comporta igual, peor o mejor.
      Resultado: spread máximo entre los tres tags = **39.2 ns**
      (seat 120: 39.2 ns, seat 121: 23.8 ns, seat 122: 20.1 ns) — cuatro
      órdenes de magnitud POR DEBAJO de los 64 us del gateway. El tag se
      comporta muchísimo mejor, no peor: el término dominante en la guarda
      sigue siendo el jitter del gateway, no el del tag.
- [x] Actualizar `docs/superpowers/specs/2026-08-30-blink-slotted-mac-design.md`
      §1.2 con el número medido, reemplazando el valor de arranque
      (`BLINK_SLOT_GUARD_UUS = 200`) por uno derivado del dato real si el
      dato lo justifica. **No mover el valor en el código sin haber
      actualizado primero el documento que dice por qué.**
      Hecho: `BLINK_SLOT_GUARD_UUS = 100` (suma de términos conocidos
      64 + 8 + 0.04 us ≈ 72 us, redondeado hacia arriba con margen),
      `src/blink_sched.h` y la spec §1.1/§1.2 actualizados juntos.
      `BLINK_N_SLOTS` pasa de 134 a **144** (`tests/blink_sched/`, PASSED) —
      y al superar `GW_MAX_SEATS` (128), la banda de desborde que §1.1
      describía queda hipotética en este build.
- [x] Si el jitter medido es sorprendentemente alto (peor que el reloj de
      ms que se está reemplazando, lo cual sería una señal de que algo en
      la Tarea 1 está mal, no una propiedad del hardware), volver a la
      Tarea 1 antes de continuar — no fijar una guarda grande para
      compensar un bug de armado.
      No aplica: el jitter medido (39.2 ns) es órdenes de magnitud MENOR de
      lo esperado, no mayor -- no hay señal de bug de armado que investigar.

---

## Task 3: `blink_sched.{c,h}` — el número de slots y la asignación *(ANCLA, host-testeable)*

**Files:**
- Create: `src/blink_sched.h`, `src/blink_sched.c`
- Test: `tests/blink_sched/test_blink_sched.c`
- Modify: `CMakeLists.txt`, `CLAUDE.md`

**Interfaces:**
- Consumes: `mac_cell_max_slots()`, `mac_frame_ns()`, `mac_phy_frozen()`,
  `struct mac_cell`, `struct mac_phy` de `src/mac_budget.h`;
  `BLINK_FRAME_LEN` de `src/blink_frame.h`; `GW_MAX_SEATS` de `src/gw_core.h`.
- Produces:
  `BLINK_SLOT_GUARD_UUS` (`200u` — provisional, ver spec §1.2, comentario
  citando esa sección),
  `uint16_t blink_sched_n_slots(const struct mac_cell *cell)`,
  `uint8_t blink_sched_slot_index(uint8_t seat_id)`,
  `bool blink_sched_seat_admissible(uint8_t seat_id, uint16_t n_slots)`.

**Diseño, ya decidido en la spec — no re-derivar:**
- `blink_sched_slot_index()` es la identidad: `return seat_id;`. Válida
  siempre que el llamador ya haya comprobado
  `blink_sched_seat_admissible()`.
- `blink_sched_seat_admissible(seat_id, n_slots)` es `seat_id < n_slots`.
  Nótese que `n_slots` es un parámetro, no una constante — depende de
  `BLINK_SLOT_GUARD_UUS` y de la geometría del `mac_cell` del llamador
  (que en host test puede ser sintética), así que la función no debe fijar
  ella misma el valor.
- `blink_sched_n_slots()` es un envoltorio delgado sobre
  `mac_cell_max_slots(cell, blink_slot_ns)` con
  `blink_slot_ns = mac_frame_ns(&phy, BLINK_FRAME_LEN) + MAC_UUS_TO_NS(BLINK_SLOT_GUARD_UUS)`.
  No reimplementar el cálculo de cuántos slots caben — ya existe.

- [ ] **Step 1: Escribir el test que falla**

Crear `tests/blink_sched/test_blink_sched.c` cubriendo:

1. `test_slot_index_is_identity` — `blink_sched_slot_index(0..127)` devuelve
   el mismo valor.
2. `test_admissible_below_n_slots` — `blink_sched_seat_admissible(k, n)` es
   verdadero para `k < n` y falso para `k >= n`, en los bordes (`k = n - 1`,
   `k = n`).
3. `test_n_slots_matches_current_estimate` — con el PHY congelado
   (`mac_phy_frozen()`) y `BLINK_SLOT_GUARD_UUS`, `blink_sched_n_slots()`
   sobre un `mac_cell` con los valores actuales de `T_SUPERFRAME_UUS`,
   `BEACON_OCCUPANCY_UUS`, `T_GUARD_UUS`, `N_CAP` y `T_MINISLOT_UUS` de
   `uwb_mac.h`/`mac_budget.h` cae en el rango 90-115. Este test es
   deliberadamente un rango, no un número exacto: fija el orden de magnitud
   contra el que la spec razonó (96-114) sin acoplar el test a un cambio de
   una unidad en la guarda.
4. `test_n_slots_shrinks_with_more_guard` — más guarda produce menos o
   igual número de slots, nunca más. Control de sentido: si esto falla, la
   fórmula tiene un signo invertido.
5. `test_at_least_the_product_target` — `blink_sched_n_slots()` con la
   guarda de arranque (200 us) es `>= 100` (el objetivo de producto de
   `docs/superpowers/specs/2026-08-25-rtls-scale-tdoa-design.md` §1). Si
   este test falla, la guarda de arranque no alcanza el objetivo y hay que
   volver a la spec ANTES de escribir más código encima — no bajar la
   guarda a ciegas sin la medición de la Tarea 2.

- [ ] **Step 2: Correr y verificar que no compila**

```bash
gcc -Wall -Wextra -Isrc -o tests/blink_sched/test_blink_sched.exe \
    tests/blink_sched/test_blink_sched.c src/blink_sched.c src/mac_budget.c
```
Esperado: FALLA por `blink_sched.h` inexistente.

- [ ] **Step 3: Escribir header e implementación**, siguiendo el diseño de
      arriba. Comentario obligatorio en `BLINK_SLOT_GUARD_UUS` citando
      `docs/superpowers/specs/2026-08-30-blink-slotted-mac-design.md` §1.2 y
      marcándolo PROVISIONAL hasta la Tarea 2.

- [ ] **Step 4: Correr el test y verificar que pasa**

```bash
gcc -Wall -Wextra -Isrc -o tests/blink_sched/test_blink_sched.exe \
    tests/blink_sched/test_blink_sched.c src/blink_sched.c src/mac_budget.c && \
    ./tests/blink_sched/test_blink_sched.exe
```
**Anotar el número real que imprime `test_n_slots_matches_current_estimate`**
— es la cifra de `BLINK_N_SLOTS` para el resto de este plan y para la Tarea 4.

- [ ] **Step 5: Registrar, build, commit**

Añadir `src/blink_sched.c` a `CMakeLists.txt` (orden alfabético, junto a
`src/blink_frame.c`), `.gitignore` de test, entradas en `CLAUDE.md` citando
el número de slots medido. Correr `west build -d build_prod`; esperado 0
errores/warnings.

```bash
git add src/blink_sched.* tests/blink_sched CMakeLists.txt CLAUDE.md
git commit -m "feat(tdoa): blink slot count and identity-based slot assignment

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: El gateway sube a `proto_ver` 4 y admite en modo blink *(ANCLA)*

**Requiere la Tarea 3** (necesita `BLINK_N_SLOTS` y
`blink_sched_seat_admissible()`).

**Files:**
- Modify: `src/uwb_frame_802_15_4z.h` (`UWB_PROTO_VER` 3 -> 4)
- Modify: `src/gw_core.{c,h}` (admisión gated por
  `blink_sched_seat_admissible()` en modo blink; `gw_core_build_slotmap()`
  sigue existiendo sin cambios para el modo TWR)
- Modify: `src/uwb_gateway.c` (selección de modo de celda; en modo blink,
  `sched[]` se transmite reservado/cero, ver spec §1.3)
- Modify: `CMakeLists.txt`, `CLAUDE.md`, y el `diff` obligatorio contra
  `tag_testting/src/uwb_frame_802_15_4z.h` (regla de archivos compartidos,
  §8.0 del spec de escala)

- [ ] Decidir el mecanismo de selección de modo (Kconfig de build vs. campo
      persistido en NVS vs. comando de consola) — deliberadamente no fijado
      en la spec (§4), es una decisión de implementación. Preferir lo más
      simple que no obligue a un rebuild por celda si ya existe un patrón de
      configuración de rol equivalente (`anchor mode`) que se pueda extender.
- [ ] Subir `UWB_PROTO_VER` a 4. Confirmar con `diff` que el archivo
      compartido sigue siendo el que se copia al otro repo, no al revés.
- [ ] En `gw_core_join()`/`gw_core_keepalive()`, cuando la celda está en modo
      blink: rechazar (no asignar) un `seat_id` para el cual
      `blink_sched_seat_admissible(seat_id, BLINK_N_SLOTS)` sea falso, en vez
      de tomar cualquier índice libre del arreglo. Host-testear esto en
      `tests/gw_core/` como un caso nuevo, no asumir que el test existente lo
      cubre.
- [ ] En `uwb_gateway.c`, en modo blink: no escribir direcciones reales en
      `sched[]` del beacon (reservado/cero, spec §1.3); el resto de la
      construcción del beacon no cambia.
- [ ] Host tests de `gw_core` en verde, `west build -d build_prod` limpio.
- [ ] Commit.

---

## Task 5: El tag calcula su propio slot de blink desde `seat_id` *(tag_testting, hardware)*

**Requiere la Tarea 1** (TX diferido en DTU ya funcionando) **y la Tarea 4**
(que exista un gateway en modo blink contra el cual probar; sin él, no hay
forma de distinguir en el banco un tag mal programado de un gateway que
todavía habla `proto_ver` 3).

**Files:**
- Modify: `src/uwb_net.h`/`.c` (`UWB_NET_PROTO_VER` 3 -> 4, día-bandera
  emparejado con la Tarea 4 — ver spec §1.3, ya funciona por el chequeo
  exacto existente en `uwb_net.c:250`/`:289`, no hace falta tocar la FSM)
- Modify: `src/uwb_net_runner.c` (el offset de slot del BLINK usa
  `ctx.seat_id` en vez de `ctx.tx_slot` cuando `ctx.blink_mode` es verdadero)
- Copiar `src/blink_sched.h` desde ANCLA (`diff` vacío) si el offset se
  calcula con la misma función en vez de repetir la aritmética a mano —
  preferible, mismo criterio que `blink_frame.h`.

- [ ] Copiar `blink_sched.h` desde ANCLA; `diff` vacío. (`blink_sched.c` no
      hace falta si el tag solo necesita `blink_sched_slot_index()`, que es
      la identidad — evaluar si vale la pena copiar el `.c` completo por
      consistencia o si una línea inline basta; si se copia, debe seguir la
      misma regla de archivos compartidos que `blink_frame.c`.)
- [ ] En el cálculo del offset de TX del BLINK (Tarea 1), sustituir
      `ctx.tx_slot` por `blink_sched_slot_index(ctx.seat_id)` cuando
      `ctx.blink_mode` es verdadero. El offset en tiempo (multiplicar el
      índice de slot por `BLINK_FRAME_LEN` + `BLINK_SLOT_GUARD_UUS`, no por
      `T_SLOT_MS + T_GUARD_MS` que son las constantes de TWR) también cambia
      — no reusar los nombres de constante de TWR con un valor de blink por
      error.
- [ ] Subir `UWB_NET_PROTO_VER` a 4 en el firmware de blink. Confirmar en
      banco que un tag en esta versión contra un gateway todavía en
      `proto_ver` 3 (TWR) se queda sordo en SCAN — el comportamiento
      esperado, no un fallo.
- [ ] Verificar en sniffer, con dos o más tags, que cada uno transmite en el
      slot que le corresponde por `seat_id` y que no hay colisión — esto es
      la prueba de hardware de la propiedad que la spec probó por
      construcción (§1.1); confirmarla en el aire, no solo en el papel.
- [x] Commit. Hecho 2026-09-01, mismo commit que Tarea 1 (ver arriba). Incluye
      también un bug de banco encontrado y corregido en esta sesión, no listado
      en el plan original: un tag en modo blink nunca aparece en `in_map`
      (el gateway en modo blink transmite `sched[]` reservado/cero, spec §1.3),
      y `uwb_net_handle()` trataba eso como "perdí mi turno TWR" — SLEEP
      indefinido, luego TO_SCAN tras `UWB_NET_SCHED_GAP_MAX`, en loop infinito
      sin emitir jamás `UWB_ACT_SEND_BLINK`. Corregido saltando el gate
      TWR-only en modo blink; además `UWB_NET_BLINK_KA_CYCLES_MAX` como
      contención para un segundo bug de banco (asiento reclamado sin ack de
      KEEPALIVE, dos tags colisionando en el mismo slot). Ver el mensaje de
      commit y `tests/uwb_net/test_blink_mode_ignores_in_map`,
      `test_blink_mode_forces_rejoin_after_stale_ka` para el detalle completo.

---

## Task 6: Verificación de hardware end-to-end *(ambos repos)*

**Requiere todo lo anterior.** No implementar nada nuevo aquí — es el cierre
de hardware, igual que los "gates" de `docs/superpowers/plans/2026-08-25-fase3-tdoa.md`.

- [ ] Con el gateway en modo blink y N tags (tantos como haya disponibles en
      el banco — no hace falta llegar a 100 para validar el mecanismo),
      confirmar por sniffer que cada BLINK cae en su slot esperado y que
      ninguno colisiona con otro.
- [ ] Confirmar que un tag todavía en firmware TWR/`proto_ver` 3 (si hay uno
      disponible) se mantiene sordo ante el gateway en modo blink, en vez de
      comportarse de forma indefinida.
- [ ] Con la cadena de la Tarea 5-6 de `docs/superpowers/plans/2026-08-25-fase3-tdoa.md`
      ya verificada (BLINK -> estampado -> collect -> solve -> publish),
      confirmar que el gateway sigue resolviendo posiciones correctamente
      con tags transmitiendo en sus nuevos slots — este plan no debería
      haber tocado nada de esa cadena, y esta es la comprobación de que
      efectivamente no lo hizo.
- [ ] Actualizar `docs/superpowers/specs/2026-08-25-rtls-scale-tdoa-design.md`
      §5 (la nota de Fase 3 añadida en la limpieza de la Tarea 8): ahora sí
      corresponde decir que el objetivo de 100 tags tiene un mecanismo
      verificado, con el número real de `BLINK_N_SLOTS` alcanzado en
      hardware, no solo derivado.
- [ ] Actualizar ambos `CLAUDE.md` con el resultado. **Esta sesión no lo
      hace** — igual que en la Tarea 8, ningún `CLAUDE.md` se toca desde esta
      máquina; queda para quien sí sea la unidad de desarrollo principal.
