# Precisión TDoA, parte 2

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** cerrar los ítems 4-7 del análisis de 2026-09-02, que la parte 1 dejó
listados en su §5. Al final: `tdoa_solve()` no puede devolver su propia semilla
reportando éxito, el sesgo de altura está modelado, un tag quieto recibe ZUPT
desde el acelerómetro real en vez de desde una heurística sobre su propia
salida, y cada ecuación del EKF lleva la varianza que su ancla estimó.

**Spec:** `docs/superpowers/specs/2026-09-03-tdoa-accuracy-filter-part2-design.md`
— léase completo antes de empezar. Sus decisiones están tomadas y no se
re-derivan: la referencia es el `anchor_id` más bajo; `dz` por defecto **0.0**
y no un valor adivinado; el BLINK lleva su byte `flags` completo y no un
booleano; la heurística de movimiento de la parte 1 se **elimina**; y `quality`
se **mide** antes de ponderar con ella.

**Parte 1:** `docs/superpowers/plans/2026-09-02-tdoa-accuracy-filter.md`,
cerrada 2026-09-02. Resumen en `docs/tdoa-accuracy-filter-summary-2026-09-02.md`.
Lo que hereda este plan y NO se re-abre: el EKF filtra mediciones y no la
posición resuelta; `SYNC_PHASE_EMA_SHIFT` se queda en 3 (hallazgo negativo
medido); la propiedad de `pos_ekf` **no** se movió — el tag lo sigue usando en
su camino TWR, así que tocar sus funciones de rango es un cambio de dos repos.

**Alcance:** ítems 4-7. El ítem 8 (campaña de retardo de antena RX) y el
crecimiento de anclas (`2026-09-02-blink-anchor-scale.md`) quedan fuera.

**Regla que atraviesa el plan:** igual que en la parte 1, *suavidad no es
exactitud*. Nada aquí mueve el objetivo de ~45 cm, que sigue acotado por GDOP
del arreglo y por el retardo de antena RX sin calibrar. Y sin cinta métrica
esto sigue midiendo precisión, no exactitud.

**Orden de dependencias:**

```
Tarea 1 (banco, LÍNEA BASE) ─────────> instrumento de aceptación de la Tarea 6
Tarea 2 (solver, host) ──┐
Tarea 3 (dz, host+ANCLA) ─┼──> Tarea 6 (hardware, ambos repos)
Tarea 4 (ZUPT, 2 repos) ──┤
Tarea 5 (ponderación) ────┘
```

Las Tareas 2-5 son independientes entre sí. La Tarea 1 **no bloquea** escribir
código (decisión del operador), pero sí es el instrumento contra el cual la
Tarea 6 se acepta: si no existe, la Tarea 6 no puede afirmar mejora, solo
ausencia de regresión. La Tarea 5 tiene una mitad que puede terminar en "no
hacer nada".

**Rama sugerida:** `feat/tdoa-accuracy-part2`, desde `feat/tdoa-accuracy-filter`
ya fusionada.

**Orden de despliegue, el único seguro:** gateways, luego anclas, luego tags.
`POS_JSON_BLINK_MAX_LEN` sube en la Tarea 5 y el modo de falla al invertirlo no
es degradación: un gateway viejo rechaza **todas** las observaciones.

**Estado 2026-09-03:** Tareas 2, 3, 4 y **5b** escritas y host-testeadas.
Tarea 1 (línea base de banco) y Tarea 6 (hardware) sin empezar — nada de esto
ha tocado un board.

**Tarea 5a NO se puede cerrar en esta sesión, y no por falta de código.** Pide
medir la correlación entre `quality` (`diag.ipatovAccumCount`) y el residual
por ecuación *sobre una captura real*, y no hay captura ni hardware
disponible. Estado: **no se pondera por `quality`**, que es el
comportamiento seguro y es uno de los dos resultados que el spec §4.1 admite
de antemano. Lo que falta para cerrarla, para quien la retome:

1. Capturar, sobre un gateway vivo con >= 4 anclas, pares
   `(quality_i, |residual_i|)` por ecuación durante varios minutos con un tag
   quieto y otro caminando. Hoy `quality` llega al gateway y se descarta, así
   que hace falta instrumentación temporal — con la disciplina que la parte 1
   ya usó: agregar, medir, y RETIRAR una vez explicada, no dejarla en el
   árbol.
2. Si la correlación es débil (lo esperable: a PLEN fijo el conteo de
   acumulación puede ser casi constante), escribir el hallazgo NEGATIVO junto
   al campo con los números y dejar `quality` como diagnóstico. Ese es el
   mismo desenlace que tuvo el barrido de `SYNC_PHASE_EMA_SHIFT` en la
   parte 1.
3. Si es fuerte, la ponderación ya tiene dónde entrar: `struct tdoa_meas`
   lleva `sigma_m` y `pos_ekf_update_tdoa()` ya la consume por ancla. Sería
   combinar el término por-blink con el por-ancla, no cablear nada nuevo.

La 5b (la sigma de sincronía) sí está hecha, y con una **corrección al spec**:
publica `sync_model_jitter_est_dtu()` (medido) y no
`sync_model_error_dtu()` (que se calcula desde `SYNC_JITTER_DTU`, una
constante asumida de ~100 ps contra los 782 ps-1.53 ns que el hardware
midió — habría dado una sigma ~15x demasiado optimista).

---

## Task 1: Línea base del tag quieto *(banco, sin código)*

**Por qué primero:** la parte 1 cerró sin ANTES y con **4 muestras** del tag
quieto, que dieron ~0.8 m de dispersión — el único defecto observado que queda
abierto, y justo el que la Tarea 4 ataca. Repetir la captura al final sin tener
un antes es repetir literalmente el error de la ronda 3.

- [ ] Imagen actual (parte 1 ya fusionada), sin ninguno de los cambios de este
      plan. Gateway en **USB-C** — la fuente no sostiene el segundo TX del CCP
      y en batería el síntoma es indistinguible de una falla de firmware.
- [ ] Un tag inmóvil, en un punto fijo y **anotado**, captura larga (>= 10 min,
      no 4 muestras). Registrar la dispersión de `(x, y)`: desviación estándar,
      recorrido máximo, y la traza cruda.
- [ ] Un segundo tag caminando un recorrido repetible, para tener contra qué
      comparar continuidad además de dispersión.
- [ ] `blink stats` al inicio y al final: `seeded`/`filtered`/`reseed`/
      `no_update`/`dt_invalid`/`gate_rejected`, más `jump`, `solve_fail` y
      `no_anchor`. La aritmética `fixes == seeded + filtered + reseed` tiene
      que cuadrar (es el control que cazó el bug de la ronda 2).
- [ ] Anotar qué anclas contribuyeron de verdad — la parte 1 cerró sin saberlo.
- [ ] Guardar el log crudo en el repo o donde el equipo los guarde, no solo el
      número resumido.

**Aceptación:** existe una cifra de dispersión del tag quieto con `n`
suficiente, y la traza cruda, de una imagen SIN los cambios de este plan.

**Riesgo:** ninguno — no toca código. El riesgo es saltársela y llegar a la
Tarea 6 sin poder afirmar nada.

---

## Task 2: Endurecer `tdoa_solve()` *(ANCLA, host)*

**Por qué importa más de lo que suena:** `pos_solver.c` documenta que sin estas
dos defensas esa función *"returned the seed verbatim with valid = true and a
residual up to 113 m"* sobre 200k casos adversariales. `tdoa_solve()` no las
tiene **y desde la parte 1 recibe una semilla en cada fix** (la última posición
publicada), así que un solve estancado devuelve la posición anterior reportando
éxito — el bug de republicación de la ronda 2 por una segunda vía. **La
aritmética de contadores que lo cazó es ciega a esta:** entra por el camino de
siembra y cuenta como `n_seeded`.

**Leer antes:** spec §1 completo, y `gn_solve()` en `src/pos_solver.c` como
referencia de implementación.

- [ ] Búsqueda de línea con `POS_GN_MAX_HALVINGS` (8), usando `diff_residual()`
      como función de costo — **ya está escrita** en `tdoa_solve.c`, hoy solo
      se usa para llenar `out->residual_m`.
- [ ] Estacionariedad juzgada sobre el paso de Newton **sin amortiguar**, antes
      de la búsqueda de línea. Copiar las dos razones del comentario de
      `pos_solver.c`, no solo el código.
- [ ] Gate de gradiente final sobre `||J^T r||`. **El umbral se RE-DERIVA, no
      se copia:** las filas del Jacobiano aquí son DIFERENCIAS de cosenos
      directores (acotadas por 2, no por 1) y multiplican un residual de
      diferencia de rango. `POS_GN_GRAD_EPS` está dimensionado para otro
      modelo. Derivar contra `r_tdoa` (0.6 m) y **pinarlo en un test**.
- [ ] Ancla de referencia determinista: el `anchor_id` más bajo del grupo,
      intercambiado a la posición 0 **antes** de `tdoa_dtu_rebase()`. Todo
      aguas abajo (rebase, `plausible`, solve, EKF, `dt`) queda intacto.
      `struct tdoa_meas` no lleva `anchor_id`, así que el intercambio va donde
      el grupo todavía lo sabe — resolverlo sin filtrar el id a un struct que
      no lo necesita.
- [ ] Tests en `tests/tdoa_solve/`: un caso adversarial que **falle contra el
      código actual** devolviendo la semilla con `valid = true`; que el gate de
      gradiente lo rechace; que la búsqueda de línea converja donde el paso
      crudo diverge; y que la referencia sea la misma ante un `m[]` permutado.
- [ ] Actualizar la sección "KNOWN LIMITATION" de `tdoa_solve.h`, que hoy
      declara este hueco como diferido a propósito. Si se cierra, deja de ser
      cierta.

**Aceptación:** `tests/tdoa_solve/` PASSED, incluido el caso adversarial que
falla contra HEAD. `tests/tdoa_collect/`, `tests/tdoa_dtu/` y `tests/pos_ekf/`
sin regresión.

---

## Task 3: Modelo de altura *(ANCLA)*

**Leer antes:** spec §2, en particular por qué un `dz` uniforme **no** se
cancela en una diferencia de rangos — la intuición contraria es la que gana
sola.

- [ ] `dz_i = z_i(survey) - z_tag_asumido`. En un survey 2D todos los `z_i` son
      0 y se reduce a un `h` uniforme; en uno 3D respeta las alturas por ancla.
- [ ] **Default 0.0**, no un valor adivinado. Un `h` equivocado es un sesgo
      NUEVO, y arrancar en 1.4 m cambiaría en silencio los números de todo
      despliegue existente. Con 0.0 el cambio es un no-op hasta que alguien
      mida.
- [ ] Setter persistido en el árbol de consola que ya existe, **no** un
      Kconfig: el valor es por sitio, no por imagen. Documentar que **no** es
      `apos zoff`, que mueve el plano del survey y resuelve otra pregunta.
- [ ] Tests: que `dz = 0` reproduzca exactamente el comportamiento actual, y
      que un `h` distinto de cero mueva la solución en la dirección esperada
      (comprime, no traslada).
- [ ] Anotar en `CLAUDE.md` que este número es **por sitio y sin medir** hasta
      que un operador lo mida, con la misma franqueza que el resto de las
      constantes no verificadas del proyecto.

**Aceptación:** suites PASSED; con el default el binario produce fixes
idénticos a los de la parte 1.

---

## Task 4: ZUPT desde el acelerómetro *(ANCLA + tag_testting)*

**Leer antes:** spec §3. Es mucho más barato de lo que la parte 1 estimó: la
matemática ya existe en los dos lados.

**tag_testting:**

- [ ] `BLINK_FLAG_MOVING = 0x02` en `blink_frame.h`;
      `BLINK_FLAG_RESERVED_MASK` de `0xFE` a `0xFC`. **Byte por byte igual que
      la copia de ANCLA** — la regla de copia verbatim rige y este archivo
      existe en los dos repos.
- [ ] Poner el bit desde `motion_moving`, el `volatile bool` que
      `motion.c:40` ya fija desde la interrupción del LIS2HH12 y que ya
      alimenta el filtro de tier y el ZUPT del propio camino TWR del tag.
- [ ] `UWB_NET_PROTO_VER` 4 -> 5 en `uwb_net.h`.

**ANCLA:**

- [ ] Misma edición byte por byte en `src/blink_frame.{c,h}`;
      `UWB_PROTO_VER` 4 -> 5.
- [ ] `struct pos_blink_obs` gana `uint8_t flags` — **el byte completo, no un
      booleano `moving`**. Mismo costo, y `BLINK_FLAG_ALERT` (que ya existe y
      el gateway hoy no ve) queda visible gratis.
- [ ] Campo `"f"` en `pos_json_blink()` y su parser, tolerante a su ausencia
      (un ancla en firmware viejo no lo manda).
- [ ] En `tdoa_gw.c`: los flags del grupo salen de la observación de
      referencia. `pos_ekf_zupt()` cuando el bit de movimiento está en 0, y
      `pos_ekf_predict()` con `moving` real en vez de estimado.
- [ ] **ELIMINAR la heurística de la parte 1 §4.6** — programar
      `sigma_a_move`/`sigma_a_still` desde la velocidad del propio filtro con
      histéresis. No dejarla al lado: dos criterios peleando por el mismo
      parámetro es peor que cualquiera solo. Su propio spec la aceptó solo a
      cambio de no tocar el aire, y ese trato se acabó.
- [ ] Contador nuevo en `blink stats` para ZUPT aplicado. Sin él no hay forma
      de distinguir "el bit no llega" de "el bit llega y dice moviéndose".
- [ ] Tests en `tests/blink_frame/` (los dos repos: el bit nuevo, el mask
      reservado, y que un flag reservado siga rechazándose),
      `tests/pos_json_blink/` (campo presente y ausente) y `tests/pos_ekf/`
      (que ZUPT colapse la velocidad).

**Aceptación:** suites PASSED en ambos repos; `blink_frame.c` byte-idéntico
entre los dos; `west build` limpio en ambas imágenes.

**Riesgo declarado:** un tag reportado quieto mientras lo cargan despacio recibe
ZUPT que no le toca y el filtro se pega. La defensa existente es el streak
`reset_after` -> `pos_ekf_needs_reseed()`, ya cableado en la parte 1. **No
agregar defensa nueva** — verificar en la Tarea 6 que esa dispara.

---

## Task 5: Ponderación por observación *(ANCLA, host + banco)*

**Leer antes:** spec §4. Este task tiene dos mitades y **la primera puede
terminar en "no hacer nada"**.

**5a — `quality`: medir antes de usar.**

- [ ] `obs.quality` **ya viaja y ya se parsea** (`pos_json.c:286`); usarla no
      cuesta un byte de formato. Pero es `diag.ipatovAccumCount`, que **a PLEN
      fijo puede ser casi constante** y no llevar información de calidad.
- [ ] Medir la correlación entre `quality` y el residual por ecuación sobre una
      captura real antes de ponderar con ella. Misma disciplina que la Tarea 1
      de la parte 1.
- [ ] Si la correlación es débil: **no ponderar por `quality`**, y escribir el
      hallazgo negativo junto al campo, con los números. Su propio comentario
      ya dice que es diagnóstico; que se quede así con evidencia.

**5b — la sigma de sincronía, que sí es principiada.**

- [ ] Publicar `sync_model_error_dtu()` por observación: campo `"e"` en el JSON,
      campo nuevo en `struct pos_blink_obs`.
- [ ] `pos_ekf_update_tdoa()` acepta una `R` por ecuación. La varianza de la
      ecuación `i` es `sigma_i^2 + sigma_0^2` — la referencia entra en
      **todas** las ecuaciones, misma razón por la que `r_tdoa` se derivó con
      un `sqrt(2)` en la parte 1.
- [ ] `r_tdoa` se **conserva como fallback** para observaciones sin sigma (un
      ancla en firmware viejo). No borrarlo.
- [ ] **Subir `POS_JSON_BLINK_MAX_LEN`.** Es también el rechazo duro del
      parser: peor caso de hoy 66 B contra un techo de 95, y los dos campos
      nuevos (`"f"` y `"e"`) consumen ~18, dejando 11. Dejarlo convertiría el
      siguiente campo en una emergencia. Actualizar el comentario de
      versionado, que es donde vive la regla de desplegar gateways antes que
      anclas.
- [ ] Tests: `tests/pos_json_blink/` con los dos campos y con el nuevo peor
      caso medido (ese suite ya imprime el peor caso — usar ese número, no uno
      estimado); `tests/pos_ekf/` con `R` heterogénea, comprobando que una
      observación con sigma grande mueve menos el estado.

**Aceptación:** suites PASSED; el peor caso del payload **medido** por el propio
test y con holgura registrada; y una decisión escrita sobre `quality` con los
números que la sostienen, sea cual sea.

---

## Task 6: Verificación en hardware *(hardware, ambos repos)*

**Bloqueado por:** las Tareas 2-5. Y sin la Tarea 1 solo puede afirmar ausencia
de regresión, no mejora — decirlo así en el reporte.

- [ ] **Desplegar en orden: gateways, luego anclas, luego tags.** Con el orden
      invertido un gateway viejo rechaza *todas* las observaciones — pérdida
      total y silenciosa, no degradación.
- [ ] Gateway en **USB-C**, survey aplicado (`apos show` con `"valid":1`) antes
      de esperar un solo fix.
- [ ] **Repetir la captura de la Tarea 1 exactamente**: mismo punto anotado,
      misma duración, mismo recorrido. Comparar dispersión del tag quieto ANTES
      vs DESPUÉS. Ese es el criterio de aceptación del ítem 6.
- [ ] Verificar que el bit de movimiento llega: el contador de ZUPT de la
      Tarea 4 se mueve, y se mueve solo cuando el tag está quieto.
- [ ] Provocar el caso de riesgo: llevar el tag despacio, de forma que el
      acelerómetro reporte quieto. Confirmar que `reset_after` ->
      `pos_ekf_needs_reseed()` dispara y que el filtro se recupera, en vez de
      quedarse pegado.
- [ ] La aritmética `fixes == seeded + filtered + reseed` sigue cuadrando, y
      `no_update` en 0. Es el control que cazó el bug de la ronda 2 y la
      Tarea 2 agrega un camino que ese control no cubría — verificar los dos.
- [ ] `kernel reboot cold` y repetir.
- [ ] Ni un `"beacon started but TXFRS never completed"`: la Tarea 2 agrega
      hasta 8 evaluaciones de costo por iteración al solve, en el lazo
      `K_PRIO_COOP(0)`.
- [ ] Si se fijó un `h` distinto de 0 (Tarea 3), **medirlo con cinta** y
      anotar cuál se usó. Un `h` inventado es un sesgo nuevo presentado como
      corrección.

**Aceptación, y su límite:** la dispersión del tag quieto baja de forma medible
contra la línea base de la Tarea 1, y nada regresa. **Eso es precisión.** La
exactitud absoluta de ~45 cm no se toca aquí: sigue acotada por GDOP (plan de
anclas, con sus propias precondiciones) y por el retardo de antena RX sin
calibrar (ítem 8, sin campaña).

- [ ] Escribir el resultado en `CLAUDE.md` con la franqueza de la parte 1: qué
      se midió, qué no, con cuántas muestras y con qué fuente de alimentación.
