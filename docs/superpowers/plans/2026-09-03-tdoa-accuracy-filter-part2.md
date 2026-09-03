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

---

## Task 7: el filtro se FUGA, y la recuperación por divergencia está muerta *(ANCLA)*

**Añadida 2026-09-03, después de correr la parte 2 en hardware.** No es tuning
y no es una mejora: es un defecto que la parte 2 introdujo y que su propia
verificación de host no podía ver.

### La evidencia

Captura `COM15_2026_09_03.11.58.45.067.txt`, gateway con survey de **3** anclas
en `(0.000, 0.000)`, `(1.752, 0.920)`, `(2.385, 0.000)` — triángulo plano, base
2.385 m y ápice a 0.920 m. Un tag, 421 fixes, 155.8 s, moviéndose y luego
quieto.

```
Solo 32.5% de los 421 fixes cayeron DENTRO del triangulo de anclas.
Una sola fuga: 30.2 s, 137 fixes, de (1.58, 1.95) a (-2.45, 17.64).
Maximo 17.24 m FUERA de un arreglo de 2.4 m.
22.6% de todos los fixes a mas de 5 m fuera del casco.
Periodo quieto (ultimos 60 s): RMS 0.394 m, media en (0.820, -0.305).
```

`blink stats` al final de esa corrida:

```
seeded:11  filtered:304  reseed:0  dt_invalid:12  gate_rejected:70
no_update:2  zupt:99   (fixes:315 = 11 + 304, cuadra)
```

La escapada es **monótona a ~1.5 m/s en una dirección durante 30 segundos** —
no es ruido, y `reseed` se quedó en **0** todo el tiempo.

Dos cosas que la captura descarta y conviene no volver a proponer:

- **El bit `MOVING` funciona.** `zupt:99` de `filtered:304` = 32.6%, o sea que
  llega y VARÍA. La hipótesis de polaridad invertida en `motion.c` queda
  descartada.
- **`residual` no podía avisar.** Sale `0.000` en las 421 líneas, que a 3
  anclas es cero por construcción (contrato de `tdoa_solve.h`). La única señal
  de calidad es estructuralmente ciega justo en esta configuración.

### La causa, exacta

`pos_ekf_needs_reseed()` exige `gate_streak >= reset_after` (3), y
`gate_streak` solo incrementa cuando **`accepted == 0`**: todas las ecuaciones
rechazadas. Con 3 anclas, `pos_ekf_update_tdoa()` produce **2** ecuaciones, así
que basta que una se acepte para resetear el streak. `gate_rejected:70` sobre
~608 ecuaciones es 11.5% (contra ~0.3% esperable a 3 sigma), pero casi nunca
las dos a la vez.

Y el mecanismo de la fuga: **una sola ecuación de diferencia de rangos
restringe una sola dirección.** El filtro acepta un update por ciclo — streak
reseteado, reseed nunca — mientras corre por la dirección que esa ecuación no
restringe. En este triángulo la dirección débil es `y`, y `y` es lo que llegó a
17.64.

**Origen: la parte 1.** El criterio "todas las mediciones rechazadas" fue
diseñado para `pos_ekf_update_ranges()`, donde 4 anclas dan 4 updates escalares
independientes y cada uno restringe un rango completo. Se reusó verbatim para
diferencias de rango con `n-1 = 2` ecuaciones, y eso vació la recuperación sin
que nada fallara. Los host tests no lo vieron porque ninguno corre 30 segundos
de datos ruidosos con geometría delgada.

### Lo que se hace, y en este orden

Tres defensas INDEPENDIENTES, porque la fuga demostró que una sola condición
puede fallar en silencio. Cada una con su contador — sin contador no hay forma
de saber cuál actuó.

- [ ] **Un test host que REPRODUZCA la fuga, antes de arreglarla**, con esta
      geometría (triángulo plano de 3 anclas) y huecos de `dt` como los
      medidos. Si el test no falla contra HEAD, no está reproduciendo el
      defecto. Ninguno de los tests actuales lo hace: corren geometrías
      cuadradas y `dt` constante. Va primero por eso.
- [ ] **Cota física sobre la posición FILTRADA.** Un tag no puede estar a 17 m
      de un arreglo de 2.4 m. Se acota contra el survey: `R_anclas` = máxima
      distancia ancla-centroide, y el fix filtrado tiene que caer dentro de
      `R_anclas + TDOA_GW_MAX_HULL_EXCESS_M`. Fuera de eso, **re-sembrar desde
      un solve fresco** (no clampear: clampear esconde el problema y publica
      una posición inventada) y contarlo. En este arreglo `R_anclas` es 1.413 m,
      así que un margen de 3 m da una cota de 4.4 m del centroide; la fuga
      llegó a 17.6 m y se corta en el primer paso.
      **Límite del producto que hay que declarar:** eso también recorta el
      seguimiento legítimo de un tag lejos del arreglo. Con 3 anclas y este
      GDOP esas posiciones no son confiables de todos modos, pero el margen es
      una constante nombrada y no un número escondido.
- [ ] **`pos_ekf_pos_sigma()` como detector de divergencia.** Ya está escrita y
      **nadie la llama** (verificado: cero consumidores fuera de `pos_ekf.c`).
      Durante una fuga `P` crece en cada `predict()` y los updates rechazados no
      la encogen, así que es el indicador correcto y no depende de la cuenta de
      ecuaciones — que es exactamente el defecto de arriba.
      **El umbral se MIDE, no se elige:** una semilla fresca ya arranca en
      `sqrt(1.5^2 + 1.5^2) = 2.12 m` (`POS_EKF_SEED_POS_VAR`), así que un
      umbral por debajo de eso re-sembraría en cada siembra. Barrer en host
      contra las tres trazas sintéticas que ya existen (quieto, quieto con bit
      pegado, para-tras-caminar) más la geometría delgada de esta captura.
- [ ] **Bajar `TDOA_DT_MAX_MS`.** Está en 2000 y la captura muestra `dt` p90 =
      **0.800 s**, máximo **8.000 s**, con 20 huecos de más de 1 s (cifras de
      `tools/pos_trace.py`, que es el instrumento reproducible; un primer
      análisis a mano dijo p90 = 1.000 s por un off-by-one en el percentil —
      el argumento no cambia). Un hueco de 0.8-1 s se acepta hoy y se
      predice: a 1.5 m/s es más de 1 m en un solo paso.
      Predecir 1-2 s de velocidad constante en un arreglo de 2.4 m no es
      defendible. Candidato ~600 ms (tres blinks), con el hueco cayendo al
      camino de solve fresco — que es lo correcto y es baratísimo aquí
      (`solve_fail:2` de 315, el solve funciona).
- [ ] Contadores nuevos en `blink stats`, uno por defensa. Y revisar que la
      aritmética `fixes == seeded + filtered + reseed` siga cerrando con los
      caminos nuevos: es el control que cazó el bug de la ronda 2, la Tarea 2
      ya le agregó una vía, y esta le agrega otras dos.

**Aceptación:** el test de fuga falla contra HEAD y pasa después; y en
hardware, sobre una corrida equivalente, el porcentaje de fixes dentro del
triángulo sube muy por encima del 32.5% medido, con `reseed` moviéndose en vez
de quedarse en 0.

**Lo que esto NO arregla, y hay que decirlo:** el triángulo de 3 anclas con
ápice a 0.920 m sobre una base de 2.385 m tiene `y` mal observable por
geometría, y `residual` es cero por construcción a 3 anclas. Ninguna defensa de
software cambia eso — es el plan de anclas
(`docs/superpowers/plans/2026-09-02-blink-anchor-scale.md`) y la cinta métrica.
Y `apos tagz` sigue en 0.0, con su sesgo del ~23% medido en la Tarea 3, que
aquí es de segundo orden frente a una fuga de 17 m.

---

## Task 1 — CERRADA 2026-09-03, con números y con su atribución

Captura `COM15_2026_09_03.12.26.58.574.txt`, tag **inmóvil dentro del
triángulo, entre el ancla origen y la del eje x**, 687 s, imagen con la
parte 2 completa. Analizada con `tools/pos_trace.py`.

### El número de la Tarea 1

```
dispersion sobre la media de la traza:  RMS 1.023 m
                                        std x 0.372 m   std y 0.953 m
paso consecutivo p50: 0.063 m
media reportada: (0.665, -1.504)
```

**Eso es PRECISIÓN.** Y hay además un **sesgo de exactitud de ~1.5 m en `y`**:
el tag estaba sobre la línea base (`y ~ 0`) y se reportó 1.5 m por debajo. Cero
de 291 fixes cayeron dentro del triángulo.

### Lo que el filtro sí arregló, medido contra el log del tag en movimiento

| | tag moviéndose | tag quieto |
|---|---|---|
| `zupt / filtered` | 99/304 = 32.6% | **1066/1253 = 85.1%** |
| `gate_rejected` / ecuaciones | 70/608 = 11.5% | **39/2506 = 1.6%** |
| dispersión RMS | 4.772 m | **1.023 m** |

ZUPT dispara el 85% de los ciclos con el tag quieto y el gate de innovación
baja a 1.6%. **El bit `MOVING` funciona** — la hipótesis de polaridad invertida
en `motion.c` queda enterrada y no se vuelve a proponer.

### La atribución del sesgo, medida y no supuesta

El sitio donde estuvo el tag es el PEOR de este arreglo. Sensibilidad de las dos
ecuaciones en `(0.665, 0.0)`:

```
ecuacion vs apice (1.752, 0.920):  d/dy = -0.647
ecuacion vs xaxis (2.385, 0.000):  d/dy = -0.001   <- CERO informacion de y
```

Con el tag sobre la base, el par origen-xaxis no aporta **nada** en `y`: por
simetría, moverlo perpendicular a la base cambia los dos rangos casi igual.
Todo `y` cuelga de una sola ecuación con ganancia 0.647.

Amplificación (metros de error de posición por metro de sesgo en la diferencia
de rangos, mayor singular de la pseudo-inversa del Jacobiano):

```
punto del tag (0.665, 0.0), geometria de hoy : 2.09
mismo punto, apice a 1.840 (el doble)        : 1.49
mismo punto, apice a 2.760 (el triple)       : 1.33
CENTRO del triangulo (1.379, 0.307), hoy     : 1.16
```

Y la conversión directa: **1 DTU (4.69 mm) de sesgo en la ecuación del ápice
mueve `y` 8.0 mm.** Los −1.504 m observados necesitan **~187 DTU = 2.9 ns**
(~0.88 m) de sesgo en esa diferencia de rangos.

**Dos causas candidatas, las dos ya abiertas en `CLAUDE.md`, y esto NO decide
entre ellas:**

1. **Retardo de antena RX sin calibrar** (ítem 8). En TDoA el observable lleva
   `(dR_i - dR_0)` a 4.69 mm/DTU **sin nada que lo cancele**, y ninguna
   medición TWR puede restringirlo. La campaña con láser de 2026-08-28 midió
   sesgos por board con un spread de 485 mm en la SUMA (~207 unidades), así que
   187 DTU entre dos boards es exactamente del orden ya observado en estos
   mismos boards.
2. **La geometría del survey nunca se validó con cinta.** `CLAUDE.md` tiene
   abierto que el `apos run` de 2026-08-26 discrepó hasta **1037 mm** contra
   los `anchor pos` que los boards cargaban. Si la `y` surveyada del ápice
   (0.920) está mal, sesga `y` directamente.

La causa (2) es **gratis de descartar** y va primero: medir las tres aristas
con cinta.

### Lo que este análisis DESCARTA como causa principal

**El modelo de altura (`apos tagz`) no explica esto.** Medido con esta
geometría exacta: incluso con una separación real de 1.6 m, resolver con
`dz = 0` da 0.222 m de error y mueve `y` solo a −0.042. Y el signo es el
contrario del observado — el sesgo por altura tira hacia el centroide, no hacia
afuera. Sigue valiendo la pena medir el sitio, pero aquí es de segundo orden
por un orden de magnitud.

### Dos acciones físicas, ambas más grandes que cualquier arreglo de software

- **Para la próxima captura de línea base: pon el tag cerca del CENTRO del
  triángulo** (~1.38, 0.31), no sobre la base. La amplificación pasa de 2.09 a
  **1.16** sin tocar hardware. La corrida de hoy midió el peor punto del
  arreglo.
- **Para el despliegue: sube el ápice.** De 0.920 a ~1.840 m lleva la
  amplificación peor caso del área de **4.21 a 1.73**. Triplicarla apenas
  mejora más (1.33 vs 1.49), así que duplicar captura casi toda la ganancia
  disponible.

### Lo que queda abierto de la Tarea 1

- [ ] Medir las tres aristas con cinta y compararlas contra el survey. Decide
      entre las causas (1) y (2) de arriba, y es gratis.
- [ ] Una captura de línea base con el tag en el CENTRO, para tener la cifra de
      precisión en un punto usable y no en el peor.
- [x] Cifra de dispersión con `n` suficiente: 291 fixes sobre 687 s.
- [x] Traza cruda guardada y reducible con `tools/pos_trace.py`.
- [x] Qué anclas contribuyeron: las tres, `no_anchor:0` y `ingested/3 = 1590`
      blinks.

**Un aviso sobre la captura, que costó casi un hallazgo falso:** el gateway
contó **1424** fixes y la consola entregó **291** (20.4%), con la pérdida
concentrada en la cola. Eso se lee igual que un tag bajando a un tier lento y
no lo es — `ingested/anclas` muestra que el tag blinkeó a ~2.3 Hz los 687 s
completos. Zephyr no reporta drops porque la pérdida está por debajo de su
contabilidad (consola USB / terminal). `tools/pos_trace.py` ahora lo detecta y
lo grita antes de que las cifras de cadencia se lean como propiedades del
stream de fixes. **`dt_invalid: 180` de 1424 (12.6%) sí es real** — sale del
reloj DTU, no del log.

### Actualización 2026-09-03: el survey está VALIDADO, el sesgo es el ítem 8

El operador midió las aristas con cinta y **el survey está correcto**. Eso
elimina la causa (2) de arriba, así que el sesgo de ~1.5 m queda atribuido
**por eliminación al retardo de antena RX** (ítem 8). Confirmado en el
propio hardware: un ancla reporta `ant_tx: 16356` (calibrado) y
`ant_rx: 16385` — **el default de fábrica, intacto**. En TDoA nada cancela
la parte RX, a 4.69 mm/DTU.

Y la prueba definitiva, con el tag en el MISMO punto físico las dos veces:

```
3 anclas -> (0.665, -1.504)     error en y: -1.50 m
4 anclas -> (1.222, +1.553)     error en y: +1.55 m
la posicion REPORTADA se movio 3.11 m por agregar un ancla

std y (RUIDO):   0.953 -> 0.403 m   bajo 58%
media y (SESGO): -1.504 -> +1.553   se movio 3.06 m
```

El ruido bajó a la mitad mientras la posición reportada saltó 3.11 m. Un
sistema dominado por ruido habría convergido a la MISMA media con menos
dispersión. **El error dominante es sesgo por ancla, y ninguna tarea de este
plan lo arregla.**

Nota de alcance del operador: pasar de SS-TWR a DS-TWR se considera trabajo
futuro y queda **fuera** de este plan. No re-litigar aquí.

### La `sigma` publicada es ~3x sobreconfiada, y alimenta la fuga

`sync stats` en un ancla desplegada: `jitter_est_dtu: 33` (516 ps),
`rms_dtu: 52`, `max_dtu: 633`, `verdict: "marginal"`, `count: 3710`.

Así que la Tarea 5b **está viva, no inerte** — se publica `sigma_dtu = 33`.
Pero 33 DTU x 4.69 mm = **0.155 m** por ancla, y por ecuación
`sqrt(2) x 0.155 = 0.219 m`. Contra eso, el residual medido con 4 anclas tuvo
**mediana 0.514 m**: el error real es ~3.3x mayor que la sigma que el filtro
cree. `jitter_est` mide el ruido del **CCP**, no el del timestamp del BLINK,
que no está en esa cifra.

Consecuencia mecánica, y conecta la 5b con la 7: `R` demasiado chica cierra
el gate a 3 sigma sobre mediciones legítimas, el filtro se queda en
`predict()` puro, y se fuga. Es `gate_rejected: 70` a 3 anclas y 39 a 4.
**Corregir la sigma publicada es probablemente parte del arreglo de la
Tarea 7, no una tarea aparte.**

Y `max_dtu: 633` = 9.9 ns = **~2.97 m** en una sola observación, 12x el RMS.
La propia shell lo llama cola no gaussiana. Es una fuente de outliers real
que ninguna sigma constante describe.

### La primera casilla FALLÓ, y eso es información

El test que debía reproducir la fuga **pasa contra HEAD**:

```
thin geometry, unbiased noise: worst 0.54 m outside; needs_reseed 0, all-gated 0
```

Se conserva en `tests/pos_ekf/` renombrado a
`test_thin_geometry_stays_bounded_under_noise()`, porque lo que sí fija es
una propiedad real, y porque el intento acota el mecanismo. **Ruido no
sesgado sobre esta geometría NO hace que el filtro se escape.**

Y la aritmética dice que la recuperación nunca estuvo ni ARMADA:
`gate_streak` solo avanza en un ciclo donde **todas** las ecuaciones fueron
rechazadas, y `gate_rejected: 70` sobre 304 ciclos filtrados de 2 ecuaciones
permite como máximo 35 de esos — mientras `reset_after` necesita **3
CONSECUTIVOS**. O sea que durante la fuga el filtro estaba **aceptando al
menos una ecuación casi cada ciclo**. Eso es una situación distinta de "el
gate se cerró y el filtro navegó a ciegas", y las defensas hay que
dimensionarlas contra la real.

También se descartó una parte de la hipótesis original leyendo el flujo:
`pos_ekf_seed()` pone `gate_streak = 0`, y `pos_ekf_needs_reseed()` se
consulta DESPUÉS de la rama de siembra, así que en un ciclo sembrado no puede
disparar nunca. Inofensivo, pero no es la causa.

### Por lo tanto la primera casilla cambia: instrumentar, no adivinar

- [ ] **Instrumentación temporal en el gateway**, con la disciplina que la
      parte 1 ya usó (agregar, medir, root-causear, **RETIRAR**): por ciclo y
      para un tag, registrar el camino tomado (filtered/seeded), `dt_s`,
      `accepted` de `n-1`, `gate_streak`, `pos_ekf_pos_sigma()` y el estado
      `(x, y, vx, vy)`. Acotado, y es lo único que dice qué pasa realmente
      durante una fuga. Dos defectos de esta sesión se encontraron así y
      ninguno era alcanzable desde host tests.
- [ ] Solo DESPUÉS de eso, escribir el test de reproducción con el mecanismo
      real en la mano, y entonces sí exigir que falle contra HEAD.

Las tres defensas de abajo siguen siendo el plan, pero sus umbrales se
dimensionan contra lo que la instrumentación mida — y la sigma sobreconfiada
de arriba entra como cuarta pieza del mismo arreglo.
