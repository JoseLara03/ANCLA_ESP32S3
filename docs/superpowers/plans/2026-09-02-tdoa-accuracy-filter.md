# Precisión y suavizado de la posición TDoA

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** que la posición TDoA que el gateway publica sea menos ruidosa y
temporalmente coherente. Hoy no hay filtro en ninguna parte de la cadena: la
salida de mínimos cuadrados va directo a `pos_sink_publish()`. Al final de este
plan el gateway mantiene un EKF de velocidad constante por tag, alimentado con
diferencias de rango (no con la posición ya resuelta), con `dt` tomado del
reloj de hardware y no del lazo del gateway.

**Spec:** `docs/superpowers/specs/2026-09-02-tdoa-accuracy-filter-design.md` —
léase completo antes de empezar. Sus decisiones están tomadas y no se
re-derivan: filtrar mediciones y no la posición resuelta, mover (no copiar) la
propiedad de `pos_ekf`, `r_tdoa` como campo separado de `r_range`, `dt` desde
el `t_dtu` absoluto del ancla de referencia, y medir el barrido de
`SYNC_PHASE_EMA_SHIFT` antes de cambiar la constante.

**Alcance:** ítems 1-3 del análisis de 2026-09-02. Los ítems 4-8 (endurecer
`tdoa_solve`, modelo de altura, `BLINK_FLAG_MOVING`/ZUPT, ponderación,
campaña de retardo de antena RX) están fuera y quedan listados en la §5 del
spec para que nadie los re-derive.

**Condicionalidad heredada:** el gate de Fase 2 corrió y **falló** contra 1 ns
(782 ps a 30 cm, 1.44-1.53 ns a 3 m); la Fase 3 procede por decisión de
producto a ~45 cm. Este plan intenta mejorar ese número y **no** re-litiga
haber procedido. También hereda que la Tarea 6 del plan de blink-slotted-mac
sigue abierta por densidad — nada de aquí depende de eso ni lo desbloquea.

**Regla que atraviesa todo el plan:** *suavidad no es exactitud*. Sin punto de
verdad medido, todo lo que este plan puede demostrar es precisión
(repetibilidad). Ninguna tarea puede reportar "se ve más suave" como mejora de
exactitud, y la Tarea 5 lo dice explícitamente en su criterio de aceptación.

**Orden de dependencias:**

```
Tarea 1 (host, mide) ──> informa el default de r_tdoa en Tarea 4
Tarea 2 (host)  ──┐
Tarea 3 (ANCLA) ──┼──> Tarea 4 (ANCLA) ──> Tarea 5 (hardware)
                  │
Tarea 1 ──────────┘ (solo por r_tdoa; no bloquea escribir la Tarea 4)
```

Las Tareas 1, 2 y 3 son independientes entre sí y pueden ir en paralelo. La
Tarea 4 es la integración y necesita 2 y 3. La Tarea 1 solo alimenta un default
numérico de la Tarea 4, así que puede llegar tarde sin bloquear.

**Rama sugerida:** `feat/tdoa-accuracy-filter`, desde `feat/rtls-scale-tdoa`.
Ojo: el árbol de trabajo tenía cambios sin commitear en `Kconfig`,
`modules/dw3000-decadriver/platform/dw3000_hw.c` y un `old_board.conf` borrado
al abrir este plan — resolverlos ANTES de ramificar, no arrastrarlos.

**Estado 2026-09-02:** ninguna tarea empezada. Spec escrito esta sesión.

---

## Task 1: Barrer `SYNC_PHASE_EMA_SHIFT` en la simulación *(ANCLA, host, MIDE)*

**Por qué primero:** es el único ítem del plan que puede terminar en "no
cambiar nada", y cuesta un test run. Si gana, baja el ruido de TODA la cadena
aguas arriba del filtro, lo cual es estrictamente mejor que filtrarlo después.
Y su resultado fija el default de `r_tdoa` en la Tarea 4.

**Leer antes:** spec §2 completo — en particular §2.1 (el álgebra predice
ganancia casi nula), §2.2 (la simulación mide otra cosa y gana la simulación) y
§2.4 (la trampa de `SYNC_RESIDUAL_TO_JITTER`).

- [ ] Envolver `SYNC_PHASE_EMA_SHIFT` en `#ifndef` en `src/sync_model.h`, para
      que un test pueda pasarlo por `-D`. No cambiar el valor.
- [ ] Extender `tests/sync_model/test_sync_model.c` con un barrido de shift que
      reuse `worst_error()` tal cual: shift en {3, 4, 5, 6}, jitter en la tabla
      que `test_jitter_sensitivity_sweep()` ya usa, 12 semillas (una sola
      semilla es demasiado ruidosa para ver la tendencia — el propio
      `sync_model.h` documenta que leyó una tendencia falsa así).
- [ ] Imprimir la tabla y compilar el suite una vez por shift con
      `-DSYNC_PHASE_EMA_SHIFT=N`. Registrar los números crudos.
- [ ] Aplicar el criterio de §2.5: mejora >= 20% en el error peor -> cambiar la
      constante; < 20% -> **no cambiar nada**.
- [ ] Si se cambia: re-derivar `SYNC_RESIDUAL_TO_JITTER` en el MISMO commit
      (`test_residual_rms_measures_the_jitter()` es el instrumento), y
      actualizar la tabla de `sync_model.h` y
      `docs/anchor-sync-measurement.md` §4.1.
- [ ] Si NO se cambia: escribir el hallazgo NEGATIVO en `sync_model.h`, junto a
      la constante, con los números. Un hallazgo negativo sin registrar se
      vuelve a proponer.

**Aceptación:** `tests/sync_model/` PASSED en cualquiera de los dos caminos, y
la tabla del barrido queda en el repo (en la cabecera, no solo en el log de la
sesión).

**Riesgo:** ninguno en hardware — este task no toca ningún camino de radio. El
riesgo real es cambiar el shift y **no** re-derivar
`SYNC_RESIDUAL_TO_JITTER`, con lo cual `sync stats` reporta un jitter falso y
el gate de Fase 2 queda medido contra un instrumento roto.

---

## Task 2: Liberación temprana por anclas ESPERADAS, no por `POS_MAX_ANCHORS` *(ANCLA, host)*

**Por qué:** hoy `tdoa_collect_take_ready()` solo libera temprano a
`g->n >= POS_MAX_ANCHORS` (4) y el despliegue tiene 3 anclas vivas, así que
**ningún grupo se libera temprano jamás**: cada fix paga los 150 ms de ventana
más hasta 200 ms de espera al siguiente `tdoa_gw_step()`. Eso es ~350 ms de
latencia y una cadencia que bate contra el blink de 5 Hz. Un filtro CV
alimentado con esa cadencia hereda el batido.

**Leer antes:** spec §3.1. La cabecera de `tdoa_collect.h` (regla de liberación
y nota de agotamiento de ranuras) manda sobre cualquier "simplificación" que se
ocurra aquí.

- [ ] `tdoa_collect_set_expected(struct tdoa_collect *c, uint8_t n)` en
      `src/tdoa_collect.{c,h}`, acotando `n` a
      `[TDOA_MIN_ANCHORS, POS_MAX_ANCHORS]`.
- [ ] `expected` por defecto = `POS_MAX_ANCHORS`, para que un llamador que
      nunca invoque el setter conserve exactamente el comportamiento de hoy.
- [ ] La condición de liberación temprana pasa a `g->n >= c->expected`. **No
      tocar** la política de eviction ni la de descarte por debajo del mínimo —
      su razonamiento está en la cabecera y no se re-abre.
- [ ] `tdoa_gw_step()` llama al setter con `apos_store_get()->n_nodes`
      (`anchor_xyz()` ya lee ese struct). Si no hay survey aplicado, no llamar:
      un gateway sin survey ya descarta toda observación en `ingest_one()`.
- [ ] Tests en `tests/tdoa_collect/`: liberación temprana con `expected = 3`;
      que `expected = 4` reproduzca el comportamiento actual; que un `n` fuera
      de rango se acote y no rompa; y que un grupo con `n < TDOA_MIN_ANCHORS`
      siga descartándose al expirar la ventana.

**Aceptación:** `tests/tdoa_collect/` PASSED, incluyendo el test nuevo que
falla contra el código anterior.

**Riesgo declarado:** un ancla inventariada en el survey pero **apagada** deja
`expected` alto y devuelve la espera de ventana completa. No es regresión (es
el comportamiento de hoy) y arreglarlo requeriría meterle liveness al colector,
que su cabecera excluye a propósito. Documentarlo, no arreglarlo aquí.

---

## Task 3: Mover `pos_ekf` a ANCLA y agregar el update TDoA *(ANCLA, host)*

**Por qué:** el filtro ya existe, host-testeado, y está muerto en el tag desde
que la Fase 3 movió el solve al gateway.

**Leer antes:** spec §4.1 (la propiedad se MUEVE, no se copia, y por qué eso
rompe la regla verbatim-copy a propósito), §4.2 (TRAMPA 1) y §4.3 (`r_tdoa`).

- [ ] Copiar `tag_testting/src/pos_ekf.{c,h}` a `src/` y
      `tag_testting/tests/pos_ekf/` a `tests/pos_ekf/`. Verificar que el suite
      migrado PASA sin cambios antes de tocar una línea — es la línea base.
- [ ] Agregar `pos_ekf_update_tdoa()` con la firma y el modelo de §4.2.
      Updates escalares secuenciales, gate a `c->gate_k` sigmas, misma
      mecánica que `pos_ekf_update_ranges()`. Sin inversa de matriz.
- [ ] **TRAMPA 1:** `(m[i].t_dtu - m[0].t_dtu)` se resta en `int64_t` ANTES de
      convertir a float. Un test debe FALLAR si alguien invierte el orden —
      con timestamps del orden de 2^40, no con valores chicos donde el bug es
      invisible.
- [ ] `float r_tdoa` en `struct pos_ekf_cfg`, default 0.6 m en
      `pos_ekf_cfg_defaults()`, con el comentario que dice de dónde sale
      (jitter de Fase 2) y que se re-deriva si la Tarea 1 lo mueve.
- [ ] Tests nuevos en `tests/pos_ekf/`: trayectoria sintética con ruido de
      diferencia de 0.6 m — el filtro debe converger y su RMS debe quedar por
      DEBAJO del de `tdoa_solve()` sobre los mismos datos (esa comparación es
      el punto del task, no un extra); un tag quieto que no derive; que el gate
      rechace un outlier gordo; que la geometría degenerada no produzca NaN.
- [ ] Declarar la propiedad en el `CLAUDE.md` de ANCLA y cederla en el de
      `tag_testting`; marcar muerta (o borrar) la copia del tag en este mismo
      ciclo. No dejar dos copias vivas — es exactamente el modo de falla de
      `cal_math.c`.

**Aceptación:** `tests/pos_ekf/` PASSED (el suite heredado sin regresiones, más
los tests nuevos), y ningún `CLAUDE.md` afirmando que el otro repo es dueño.

**Riesgo:** `pos_ekf.c` gana una dependencia hacia `tdoa_solve.h`. Ambas son C
puro sin Zephyr, así que el suite host sigue compilando con gcc plano —
verificarlo explícitamente, porque es la propiedad que hace host-testeable a
todo este task.

---

## Task 4: Integrar el filtro en `tdoa_gw.c`, con `dt` de hardware *(ANCLA)*

**Por qué:** es donde el filtro se conecta y donde `dt` deja de ser falso.
Necesita las Tareas 2 y 3.

**Leer antes:** spec §3.2 (`dt` desde el `t_dtu` absoluto, disciplina de 40
bits, los dos casos de re-seed), §4.4 (por qué `static`) y §4.5 (el flujo
completo, y que el gate de salto de 10 m se conserva).

- [ ] `struct pos_ekf` dentro de `struct tag_memo`, más el `t_dtu` absoluto de
      referencia del último fix y un flag de sembrado. Todo dentro del `memo[]`
      que ya es `static` — **ningún automático nuevo en este hilo**, por el
      desborde silencioso de stack que `CLAUDE.md` documenta.
- [ ] En `solve_one()`, guardar `m[0].t_dtu` **antes** de
      `tdoa_dtu_rebase(m, n)`. Ese es el instante del fix.
- [ ] `dt` = diferencia con SIGNO sobre 40 bits contra el guardado del fix
      anterior (patrón `sdelta40()` de `sync_model.c`), convertida a segundos.
      Una resta plana es incorrecta al envolver cada ~17.2 s y falla raro.
- [ ] `TDOA_DT_MAX_MS` = 2000 en `tdoa_gw.h`, con el porqué. `dt <= 0` o
      `dt > TDOA_DT_MAX_MS` -> re-seed, no propagar.
- [ ] El flujo de §4.5, exactamente en ese orden: predict+update si hay filtro
      sembrado y `dt` válido; `tdoa_solve()` + `pos_ekf_seed()` si no;
      re-seed vía `tdoa_solve()` si `pos_ekf_needs_reseed()`; publicar
      `pos_ekf_get()`.
- [ ] Programación del ruido de proceso desde la velocidad estimada del propio
      filtro, con histéresis (§4.6). Comentar en el código que esto es un
      sustituto de ZUPT y que es un lazo cerrado sobre sí mismo — puede quedar
      pegado en "moviéndose" bajo ruido alto.
- [ ] Conservar el gate de salto de 10 m sin cambios. Cubre el camino de
      siembra, que es justo el que el filtro no cubre.
- [ ] Contadores nuevos, expuestos por `blink stats`: fixes sembrados, fixes
      filtrados, re-seeds, `dt` inválidos, ecuaciones rechazadas por el gate.
      Sin esto el filtro es una caja negra en banco y no hay forma de
      distinguir "no hay tags" de "el filtro rechaza todo".
- [ ] Todo `LOG_WRN` nuevo: **una vez por boot**, con el contador cargando la
      magnitud. Es el lazo `K_PRIO_COOP(0)` y `CONFIG_LOG_MODE_OVERFLOW`
      sobreescribe justo las líneas que el operador está leyendo. Mismo patrón
      que los cinco `warned_*` que este archivo ya tiene.
- [ ] Verificar que el payload publicado sigue siendo
      `{"Tid","x","y","z"}` — contrato congelado. Sin velocidad, sin sigma.
- [ ] `west build` limpio, y anotar el delta de `.bss` (se esperan ~1.3 kB).

**Aceptación:** compila limpio; `tests/tdoa_collect/`, `tests/pos_ekf/`,
`tests/tdoa_solve/`, `tests/tdoa_dtu/` y `tests/pos_json/` PASSED; ningún
automático nuevo en el hilo del gateway (revisar el diff, no confiar en que
compile).

**Riesgo:** este task corre en el lazo cooperativo que arma el beacon. Nada de
lo que agrega puede bloquear, transmitir ni escribir flash — las cuatro reglas
que `tdoa_gw.h` ya declara. El EKF son ~20 updates escalares por fix, sin
divisiones de matriz, así que cumple; verificarlo, no asumirlo.

---

## Task 5: Verificación en hardware *(hardware, ANCLA)*

**Por qué:** nada de lo anterior está verificado hasta que un board lo corre.
Es la lección que este repo ya pagó cuatro sesiones de banco (el desborde de
`gw_core_ctx`): host tests y code review no ven esta clase de defecto.

- [ ] Gateway en **USB-C**, no en batería. La fuente no sostiene el segundo TX
      del CCP y en batería esto se presenta como `sent:0` con `dropped`
      subiendo — indistinguible en consola de una falla de firmware. Anotar la
      fuente junto a cada medición de sincronía.
- [ ] Survey aplicado (`apos show` con `"valid":1`) antes de esperar un solo
      fix: un gateway sin survey descarta toda observación.
- [ ] Captura ANTES (imagen previa) y DESPUÉS, mismo tag, mismas posiciones:
      (a) tag quieto ~2 min, (b) tag caminando un recorrido repetible.
      Comparar dispersión entre fixes consecutivos y continuidad de la traza.
- [ ] `blink stats`: verificar que `no_sync`, `rx_drop_*` y `sub_fail` siguen
      planos, y leer los contadores nuevos del filtro.
- [ ] Confirmar que el beacon sigue a tiempo: ni un
      `"beacon started but TXFRS never completed"` durante la corrida.
- [ ] `kernel reboot cold` y repetir — el filtro arranca sin sembrar y tiene
      que recuperarse solo.
- [ ] Si la Tarea 1 cambió el shift: re-medir `sync stats` (`jitter_est`, no
      `rms`) a 30 cm y 3 m, tras `sync reset`, y actualizar
      `docs/anchor-sync-measurement.md` §4.1.

**Aceptación, y su límite:** la dispersión entre fixes consecutivos de un tag
quieto baja de forma medible, y la traza de uno caminando es continua sin
saltos de metros. **Eso es precisión, no exactitud.** Sin cinta métrica ni
punto de verdad, esta tarea NO puede cerrar el número de ~45 cm, y su reporte
tiene que decirlo. La exactitud absoluta sigue limitada por GDOP del arreglo de
1.2-2.5 m y por el retardo de antena RX sin calibrar (ítem 8), ninguno de los
dos tocado por este plan.

- [ ] Escribir el resultado en el `CLAUDE.md` de ANCLA, sección TDoA, con la
      misma honestidad que la Tarea 7: qué se midió, qué NO, y con qué fuente
      de alimentación.
