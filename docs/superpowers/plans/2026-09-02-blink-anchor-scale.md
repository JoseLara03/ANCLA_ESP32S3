# Más anclas en modo BLINK

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** que una celda en modo BLINK despliegue más de 4 anclas, con el techo
fijado por un presupuesto MEDIDO y no por una constante heredada de TWR. Al
final de este plan `anchor id` llega a 7, el survey mide 8 nodos, y hay un
número medido que dice cuántas anclas y cuántos tags son simultáneamente
posibles en este hardware.

**Spec:** `docs/superpowers/specs/2026-09-02-blink-anchor-scale-design.md` —
léase completo antes de empezar. Sus decisiones están tomadas y no se
re-derivan: `UWB_MAX_ANCHORS` pasa a ser el espacio de identidad y sube a 8
mientras `UWB_TWR_MAX_ANCHORS` (4) acota y APLICA el cap del camino TWR;
`POS_MAX_ANCHORS` sube a 8 y con eso se cierra la transferencia de propiedad de
`pos_solver`/`pos_residual`; `APOS_ENUM_SLOTS` sube a 32 por la aritmética de
colisiones de §3.4; y el gate de capacidad de §4 va **primero**.

## Precondiciones — este plan NO arranca sin ellas

- [ ] **`docs/superpowers/plans/2026-09-02-tdoa-accuracy-filter.md` CERRADO**,
      incluida su Tarea 5 de hardware. Razones en el spec (cabecera
      "Secuencia"): `pos_ekf_update_tdoa()` corre `n-1` updates por fix y su
      costo tiene que estar medido a 4 anclas antes de multiplicarlo, y la
      transferencia de propiedad de `pos_ekf` sienta el precedente que la de
      `pos_solver` sigue.
- [ ] **`0x0003` contesta enumeración.** Lleva sin hacerlo desde 2026-08-26 y
      es la cuarta ancla que ya existe: sin ella ni el caso de 4 anclas — el
      primero donde `rms_mm` significa algo — está probado. Precondición, no
      trabajo paralelo.
- [ ] **Al menos un board físico adicional** para la Tarea 6. Las Tareas 1-5
      son host y build y no lo necesitan; la verificación de hardware sí, y sin
      board este plan se cierra a medias y hay que decirlo así.

**Condicionalidad heredada:** una celda es TWR o BLINK, nunca ambas
(`2026-08-30-blink-slotted-mac-design.md`); la Fase 3 procede a ~45 cm por
decisión de producto; y la Tarea 6 de blink-slotted-mac sigue **abierta por
densidad** (3 tags corridos contra un objetivo de 100). Este plan no cierra esa
última y no lo intenta — pero la Tarea 1 de aquí mide una parte del mismo
presupuesto, así que sus números le sirven.

**Regla que atraviesa el plan:** más anclas mejora **GDOP**, que es geometría.
No toca el sesgo de retardo de antena **RX**, que es el otro techo de exactitud
y sigue sin campaña (ítem 8 del análisis de 2026-09-02). Ninguna tarea puede
reportar una mejora de GDOP como si fuera exactitud absoluta.

**Orden de dependencias:**

```
Tarea 1 (gate MEDIDO) ──> decide el techo desplegable y si 2-5 valen la pena
                            │
Tarea 2 (cap por modo) ─────┼──> Tarea 5 (backhaul) ──> Tarea 6 (hardware)
Tarea 3 (POS_MAX + propiedad) ──┤
Tarea 4 (apos: enum + gate) ────┘
```

Las Tareas 2, 3 y 4 son independientes entre sí. La 1 va primero y puede
cambiar el alcance de todo lo demás.

**Rama sugerida:** `feat/blink-anchor-scale`, desde la rama del plan anterior
ya fusionada.

**Estado 2026-09-02:** ninguna tarea empezada. Spec escrito esta sesión.
Bloqueado por las tres precondiciones de arriba.

---

## Task 1: Gate de capacidad — MEDIR antes de crecer *(ANCLA, host + hardware)*

**Por qué primero:** anclas y tags se pelean el mismo presupuesto por
superframe y **nadie lo ha medido**, ni a 4 anclas. El resultado puede ser
"8 anclas sí, pero no a 100 tags", que es un techo de producto declarable y no
un fracaso. Implementar antes de saberlo es multiplicar un multiplicando
desconocido.

**Leer antes:** spec §4 completo, con su tabla de obs/superframe y las tres
consecuencias en orden.

- [ ] Extender `tools/tag_density.py` con la cuenta de observaciones de §4
      (`n_anclas * blink_hz * n_tags * 0.2`) y con los tres techos que hoy
      están implícitos: `TDOA_GW_INGEST_MAX`, la tasa de fixes que
      `TDOA_GW_SOLVE_MAX` permite (un grupo por llamada), y `OBS_QUEUE_DEPTH`.
      Emitir la envolvente anclas-vs-tags, no un solo número.
- [ ] **Anotar en ese archivo que sigue siendo una reimplementación a mano y
      sin test de amarre** contra `mac_budget.{c,h}`, `blink_sched.{c,h}` y las
      constantes de `gw_core.h` — el propio `CLAUDE.md` ya lo marca como riesgo
      de deriva, y agregarle un cuarto modelo lo empeora si no queda dicho.
- [ ] Medir en hardware el costo real de un `tdoa_gw_step()` a 4 anclas:
      cuántos solves caben en un superframe sin que aparezca un solo
      `"beacon started but TXFRS never completed"`. Es el número que la
      aritmética no da, y el lazo es `K_PRIO_COOP(0)`.
- [ ] Medir el techo del backhaul: a qué tasa de observaciones/s el gateway
      empieza a contar `rx_drop_evict` en `blink stats`. **Ese es el contador
      donde se manifiesta haber crecido las anclas**, en otro módulo y con otro
      nombre que `reject_shed` — verificar que aparece ahí y no donde uno lo
      buscaría.
- [ ] Escribir el resultado como una tabla anclas-vs-tags en
      `docs/anchor-sync-measurement.md` o en un doc nuevo, con la fuente de
      alimentación anotada (USB-C, no batería — la fuente no sostiene el
      segundo TX del CCP y en batería el síntoma es indistinguible de una falla
      de firmware).

**Aceptación:** existe una tabla medida que dice, para 4 y para 8 anclas,
cuántos tags a 5 Hz sostiene este hardware, y con qué contador se ve el
desborde. Si el número a 8 anclas es peor de lo aceptable, **se revisa el
alcance de las Tareas 2-5 antes de escribirlas** en vez de implementarlas y
descubrirlo después.

**Riesgo:** ninguno en el código de producción — esta tarea mide. El riesgo es
saltársela.

---

## Task 2: El cap se vuelve dependiente del modo *(ANCLA)*

**Leer antes:** spec §2, incluida la alternativa descartada (dos constantes
paralelas con el direccionamiento colgado de una) y por qué.

- [ ] `UWB_MAX_ANCHORS` sube a 8 en `src/uwb_config.h` como espacio de
      **identidad**. El short address sigue siendo
      `UWB_ANCHOR_ADDR_BASE + anchor_id`, así que ahora es 0x0001..0x0008 —
      verificar que no choca con el pool de tags (`0x0100..0xFFFD`) ni con el
      `0x0000` reservado del gateway.
- [ ] `UWB_TWR_MAX_ANCHORS = 4`, nuevo, con la derivación en el comentario:
      es el stagger de `disc_schedule.h` lo que lo fija, no una preferencia.
- [ ] `anchor id` acepta 0..7 y **avisa** al fijarlo y en el banner de boot si
      el id es `>= UWB_TWR_MAX_ANCHORS`. El aviso dice el SÍNTOMA: en una celda
      TWR esa ancla no contestará DISCOVERY y se verá como una falla de RF.
- [ ] `anchor_respond_discovery()` y el responder WAVE **rehúsan** con un id
      fuera del cap TWR, en vez de contestar tarde. Una negativa registrada es
      diagnosticable; una respuesta que llega después de que el iniciador cerró
      su ventana no lo es. Mismo criterio que `anchor_respond_wave_poll()` ya
      aplica al rehusar sin posición.
- [ ] **`BUILD_ASSERT` que ate `TX_COMPLETE_TIMEOUT_MS` (`anchor_respond.c`) a
      `disc_resp_delay_uus(UWB_TWR_MAX_ANCHORS - 1)`** más airtime. Hoy el 18
      es un número derivado a mano cuya derivación vive solo en prosa, en dos
      archivos. Esta línea es una mejora por sí sola, independiente de crecer
      anclas.
- [ ] Tests en `tests/uwb_config/`: id 0..7 aceptado, 8 rechazado, short addr
      correcto en todo el rango. Y un test que compruebe que el `BUILD_ASSERT`
      nuevo falla si alguien baja `TX_COMPLETE_TIMEOUT_MS` — mismo patrón de
      shim que `tests/mac_budget/test_uwb_mac_asserts.c`, donde incluir el
      header ES el test.

**Aceptación:** `tests/uwb_config/` PASSED, `west build` limpio, y un board con
`anchor id 5` en modo TWR imprime el aviso en el boot en vez de fallar callado.

**Riesgo:** este task cambia el espacio de direcciones de las anclas. Un board
viejo con `anchor id` persistido en NVS sigue leyendo 0..3 y no se ve afectado,
pero **verificarlo explícitamente** con un `kernel reboot cold` en vez de
asumirlo.

---

## Task 3: `POS_MAX_ANCHORS` a 8, y cerrar la propiedad de `pos_solver` *(ANCLA)*

**Leer antes:** spec §3.1 (por qué subir la constante de un lado es el modo de
falla de `cal_math.c`), §3.2 (el gate de `pos_sink`) y §3.5 (por qué la memoria
se mide y no se calcula).

- [ ] **Diffear `src/pos_solver.{c,h}` y `src/pos_residual.{c,h}` contra las
      copias del tag ANTES de tocar nada**, y tratar cualquier divergencia como
      una decisión, no como un paste. Ese diff es el punto del task: es
      exactamente lo que no se hizo con `cal_math.c`.
- [ ] `POS_MAX_ANCHORS` a 8 en `src/pos_solver.h`. `anchor_bits` de
      `tdoa_collect` es `uint32_t` y aguanta 32 — no tocarlo.
- [ ] Mover la propiedad a ANCLA: declararlo en el `CLAUDE.md` de ANCLA,
      cederlo en el de `tag_testting`, y marcar muerta (o borrar) la copia del
      tag en este mismo ciclo. **No dejar dos copias vivas.**
- [ ] **Separar el gate de `pos_sink_publish()` de `UWB_FRAME_MAX_ANCHORS`.**
      El límite de un `n_anchors` que llegó por aire es el del frame; el de un
      fix construido en casa es `POS_MAX_ANCHORS`. Hoy comparten nombre por
      accidente histórico y con 5+ anclas **cada fix TDoA muere ahí**.
- [ ] Ese `LOG_WRN` pasa a **una vez por boot** con contador, como los cinco
      `warned_*` de `tdoa_gw.c`. Es el lazo `K_PRIO_COOP(0)` y
      `CONFIG_LOG_MODE_OVERFLOW` sobreescribe justo las líneas que el operador
      estaría leyendo.
- [ ] Corregir el comentario de `pos_json.h:49`, que dice que `anchor_id` está
      acotado a `UWB_MAX_ANCHORS-1` cuando `pos_json.c:267` solo lo acota a
      `0xFF`. El rechazo real pasa aguas abajo. Un sitio menos que cambiar y un
      comentario que deja de mentir.
- [ ] **Medir** con `CONFIG_THREAD_ANALYZER` (está en `debug.conf`) la marca de
      agua de `main` con `POS_MAX_ANCHORS = 8`: el `m[]` automático de
      `solve_one()` pasa de 96 a 192 B sobre un stack que ya pica en 1748 B.
      No aceptarlo por aritmética — el ABI ventaneado de Xtensa derrama
      ventanas encima de cada frame declarado.
- [ ] Anotar el delta de `.bss` (`struct tdoa_collect` de 1792 B a ~3.3 kB).
- [ ] Tests: `tests/tdoa_collect/`, `tests/tdoa_solve/`, `tests/pos_solver/`,
      `tests/pos_residual/`, `tests/pos_ekf/` y `tests/pos_json/` PASSED con la
      constante nueva. Agregar un caso de 8 anclas a `tests/tdoa_solve/` y a
      `tests/pos_ekf/`.

**Aceptación:** todos los suites PASSED; marca de agua de `main` **medida**, no
estimada; ningún `CLAUDE.md` afirmando que el otro repo es dueño de
`pos_solver`.

---

## Task 4: El survey a 8 nodos *(ANCLA, host)*

**Leer antes:** spec §3.3 y §3.4, con la tabla de colisiones.

- [ ] El gate de `apos_node.c:464` se abre solo al subir `UWB_MAX_ANCHORS`.
      **No relajarlo por otra vía**: es una de las dos condiciones que
      sostienen la propiedad de que un ancla desplegada nunca inicia una
      transmisión no solicitada, propiedad que se movió del set de compilación
      a estos gates cuando `ss_initiator.c` entró a producción. Verificar que
      sigue rechazando un peer fuera de 0..7.
- [ ] `APOS_ENUM_SLOTS` a 32, con la tabla de probabilidades de §3.4 en el
      comentario (a 8 anclas en 8 slots se pierde un ancla el **22%** de las
      veces contra el 1.9% documentado hoy a 4 anclas).
- [ ] `APOS_GW_ENUM_GAP_MS` a 1200, re-derivado: tiene que exceder el stagger
      peor caso, que pasa de `8*30 = 240 ms` a `32*30 = 960 ms`. Actualizar el
      comentario de `apos_gw.h:57`, que hoy cita los 240 ms explícitamente.
- [ ] Verificar que `APOS_GW_WINDOW_S` (120) sigue cubriendo la enumeración
      (~3.6 s) más 56 pares ordenados a ~300 ms (~20 s). Su comentario ya está
      escrito para 8 nodos; confirmar que sigue siendo cierto y no solo
      aspiracional.
- [ ] Tests en `tests/apos_geom/` y `tests/apos_table/` con 8 nodos: que
      `rms_mm` sea distinto de cero al corromper una arista (a N=8 en 2D hay
      **15** aristas sobrantes), y que `worst_i`/`worst_j` nombre la arista
      correcta — que a 5-7 nodos `CLAUDE.md` documenta que no siempre puede.

**Aceptación:** ambos suites PASSED a 8 nodos, incluido el caso que prueba que
`rms_mm` por fin es una señal real.

---

## Task 5: Constantes del backhaul, según lo que midió la Tarea 1 *(ANCLA)*

**Leer antes:** spec §4. **Este task no elige números: los toma de la Tarea 1.**

- [ ] Ajustar `TDOA_GW_INGEST_MAX`, `TDOA_GW_SOLVE_MAX` y `OBS_QUEUE_DEPTH`
      **juntos**, al punto de operación que la Tarea 1 midió. `CLAUDE.md` es
      explícito en que los tres se mueven juntos o no se mueven.
- [ ] Re-escribir la derivación en `tdoa_gw.h` con la cuenta de anclas como
      parámetro explícito, en vez de la cifra de "8 tags a 5 Hz sobre 4
      anclas" que hoy está horneada en prosa.
- [ ] Confirmar en hardware que el `tdoa_gw_step()` resultante sigue cabiendo
      en el superframe: ni un `"beacon started but TXFRS never completed"`.
      Es la única propiedad que importa en ese lazo.
- [ ] Si el punto medido no alcanza el objetivo de producto, **decirlo en
      `CLAUDE.md` como techo medido** en vez de subir constantes hasta que
      compile. El precedente es Fase 2: se midió, falló contra su umbral, y el
      umbral se re-derivó conscientemente.

**Aceptación:** los tres constantes coherentes entre sí y con la tabla de la
Tarea 1, y el beacon a tiempo bajo la carga nueva.

---

## Task 6: Verificación en hardware con 5+ anclas *(hardware)*

**Bloqueado por:** al menos un board adicional, y `0x0003` contestando
enumeración. Sin eso este task no corre y el plan se cierra a medias — decirlo
así en el reporte, no dejarlo implícito.

- [ ] Gateway en **USB-C**. La fuente no sostiene el segundo TX del CCP y en
      batería el síntoma es `sent:0` con `dropped` subiendo, indistinguible en
      consola de una falla de firmware. Anotar la fuente con cada medición.
- [ ] `apos enum` con 5+ anclas: todas aparecen, con EUI-64 y short address
      distintos. Repetir tres veces — la Tarea 4 cambió justo la aritmética de
      colisiones, así que una sola corrida exitosa no la valida.
- [ ] `apos run` + `apos apply`: `missing_pairs:0`, `placed:N/N`, y **`rms_mm`
      distinto de cero** — con 15 aristas sobrantes a N=8 es la primera vez que
      ese número significa algo en este proyecto. Leer también
      `max_reciprocal_mm` y `max_sd_mm`.
- [ ] **Cinta métrica.** Es la deuda que el `apos run` de 2026-08-26 dejó
      abierta (hasta 1037 mm de discrepancia contra los `anchor pos` que los
      mismos boards cargaban, sospechosamente parecida a una PERMUTACIÓN de las
      mismas tres longitudes). Con más aristas el survey por fin puede
      contradecirse solo, pero sigue sin poder fijar la escala absoluta.
- [ ] El beacon a tiempo durante enumeración, ranging, solve y persist: ni un
      `"beacon started but TXFRS never completed"`.
- [ ] Un tag blinkeando sobre 5+ anclas surveyadas: fixes publicados,
      `residual_m` **distinto de cero y forwardeado** (a `n_used >= 4` deja de
      ser cero por construcción y `tdoa_gw.c` ya lo forwardea en vez de
      zerearlo — verificar que ese camino se ejerce por fin).
- [ ] `blink stats` en cada ancla: `no_sync` plano en TODAS. Cada ancla nueva
      es un enlace de sincronía más que puede estar caído sin que se note entre
      las demás.
- [ ] `kernel reboot cold` y repetir.

**Aceptación, y su límite:** el survey acepta con `rms_mm` significativo, y un
tag produce fixes con `residual_m` real sobre 5+ anclas. **Eso valida la
geometría y el GDOP, no la exactitud absoluta** — el sesgo de retardo de
antena RX (ítem 8) sigue sin campaña y es el otro techo. Sin cinta métrica esto
sigue midiendo precisión.

- [ ] Escribir el resultado en el `CLAUDE.md` de ANCLA con la misma honestidad
      que la Tarea 7 de Fase 3: qué se midió, qué NO, con cuántos boards y con
      qué fuente de alimentación.
