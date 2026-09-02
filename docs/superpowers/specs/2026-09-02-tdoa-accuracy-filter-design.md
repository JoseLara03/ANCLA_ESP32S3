# Precisión y suavizado de la posición TDoA — diseño

**Contexto:** `docs/superpowers/plans/2026-08-25-fase3-tdoa.md` (Tarea 7 corrió
en hardware 2026-08-30: la cadena BLINK -> `sync_model_to_master()` ->
`tdoa_collect` -> `tdoa_dtu` -> `tdoa_solve` -> MQTT produce fixes reales) y
`docs/anchor-sync-measurement.md` §4.1 (el gate de Fase 2 midió 782 ps a 30 cm
y 1.44-1.53 ns a 3 m, falló contra 1 ns, y la exactitud objetivo se re-derivó
conscientemente de 10-30 cm a **~45 cm**).

**Objetivo:** que la posición que el gateway publica sea (a) menos ruidosa y
(b) temporalmente coherente — que un tag quieto se vea quieto y uno caminando
se vea caminar, no saltar. Hoy no hay ningún filtro en toda la cadena: la
salida de mínimos cuadrados va directo a `pos_sink_publish()`, así que el 100%
del ruido de sincronía llega al cliente.

**Alcance:** ítems 1-3 del análisis de 2026-09-02. Sin cambio de formato en
aire, sin firmware de tag, sin bump de `UWB_PROTO_VER`.

---

## 0. Lo que este documento NO reabre

- **La decisión de producto de proceder a ~45 cm** (Fase 2 falló contra 1 ns).
  Este documento intenta mejorar ese número; no re-litiga haber procedido.
- **TDoA en vez de TWR.** Sigue siendo la migración de capacidad.
- **`residual_m` como señal de calidad a 3 anclas.** Es cero por construcción
  (contrato de `tdoa_solve.h`); nada aquí lo convierte en señal ni lo cita
  como tal.
- **El contrato de payload de posición** `{"Tid","x","y","z"}` está congelado
  con la plataforma del cliente. El filtro cambia el VALOR de `x`/`y`, no el
  documento. No se agrega velocidad ni sigma al payload.

---

## 1. Presupuesto de error, y por qué el filtro ataca solo una mitad

| término | magnitud | clase | ¿lo toca este documento? |
|---|---|---|---|
| Baseline del arreglo, 1.2-2.5 m | multiplicador GDOP | geometría | **no** — es despliegue |
| Retardo de antena **RX** sin calibrar | cientos de mm, 4.69 mm/DTU, **nada lo cancela** en TDoA | sesgo | **no** — procedimiento nuevo (ítem 8) |
| Jitter de sincronía CCP 1.44-1.53 ns | ~43-46 cm 1σ | ruido | §2 |
| Ruido del timestamp del BLINK (una sola muestra por blink, sin promediar) | dentro del anterior | ruido | §4 (solo el filtro puede) |
| Ausencia de filtro | pasa el 100% del ruido | ruido | §4 |
| `dz = 0` | sesgo real si anclas y tag difieren en altura | sesgo | diferido (ítem 5) |

**La distinción es la tesis de este documento:** *suavidad* es puramente ruido,
*exactitud* está dominada por sesgo y GDOP. Un filtro arregla lo primero y no
lo segundo. No confundirlos, y no reportar "se ve más suave" como "es más
exacto".

---

## 2. Jitter de sincronía: MEDIR el barrido de `SYNC_PHASE_EMA_SHIFT` antes de cambiarlo

La tentación es subir `SYNC_PHASE_EMA_SHIFT` de 3 a 4 o 5 (promediar la fase
sobre 16/32 CCPs en vez de 8) y cobrar ~sqrt(2) a 2x menos ruido. **El análisis
algebraico dice que esa ganancia probablemente NO existe, y la simulación dice
algo intermedio. Por eso esto se mide, no se cambia a ciegas.**

### 2.1 Por qué el álgebra predice ganancia casi nula

`sync_model_to_master()` convierte así:

```
converted = m_raw + (l - l_raw) + corr + phase_corr
```

`m_raw` es exacto (es el tiempo de TX declarado por el master). Pero `l_raw`
es **una única muestra ruidosa** del timestamp local, y entra con coeficiente
-1 vía `d = l - l_raw`. El EMA reduce el ruido de `phase_corr`, no el de
`l_raw`. En cuadratura, a shift = 3:

```
sigma_conv ~= sigma_j (+) sigma_j/sqrt(8) = sigma_j * 1.06
```

es decir, el EMA ya aporta solo ~6% del total, y llevar el shift a 5 lo baja a
~0.8%. **Ganancia despreciable.**

### 2.2 Por qué la simulación NO dice eso, y por qué gana la simulación

La tabla del propio `sync_model.h` (12 semillas, `SYNC_JITTER_DTU = 6`) mide
error peor ~= 4 DTU, o sea **~0.67 sigma_j** — mejor que el 1.06 sigma_j que
predice §2.1 y peor que el 0.35 sigma_j que predice `sync_model_error_dtu()`.
La razón es que `phase_corr` es realimentación: el residual contiene
`-noise(l_raw_prev)`, y al plegarse cancela PARCIALMENTE el ruido de `l_raw`
actual. El lazo no es analíticamente separable con el álgebra de §2.1.

Conclusión de diseño: **`tests/sync_model/`, no el álgebra, es el instrumento
que decide esta constante.** `worst_error()` ya existe en ese suite y ya barre
jitter; barrer shift es una extensión de tres líneas.

### 2.3 Lo que hay que tocar para poder barrer

`SYNC_PHASE_EMA_SHIFT` está `#define`ado sin guarda, así que un test no puede
sobreescribirlo con `-D`. Se envuelve en `#ifndef`. Es el mismo mecanismo que
`tools/tag_density.py` expone en CLI y por la misma razón: una constante que
es una CONCLUSIÓN medida tiene que ser barrible por quien la re-derive.

### 2.4 La trampa que ya está documentada, y que este cambio activa

`SYNC_RESIDUAL_TO_JITTER` (1550) es **empírico para shift = 3**. Subir el
shift baja el RMS del residual sin bajar el jitter real, así que
`sync_model_jitter_est_dtu()` — el número que `sync stats` imprime y contra el
que se juzga el gate de Fase 2 — **empieza a mentir** a menos que la constante
se re-derive en el mismo commit. `tests/sync_model/`
(`test_residual_rms_measures_the_jitter`) fija la relación, así que la mentira
falla como test y no como sesión de banco.

**Regla de no-divergencia:** si el shift cambia, `SYNC_RESIDUAL_TO_JITTER`,
la tabla de `sync_model.h` y `docs/anchor-sync-measurement.md` §4.1 se
actualizan en el mismo commit, o ninguno.

### 2.5 Criterio de decisión

Barrer shift en {3, 4, 5, 6} x jitter en {la tabla existente}, 12 semillas.

- Si el error peor baja **>= 20%** entre shift 3 y algún shift mayor: se cambia
  la constante, se re-deriva `SYNC_RESIDUAL_TO_JITTER`, y se re-mide en
  hardware con `sync stats` a 30 cm y 3 m antes de creerlo.
- Si baja **< 20%**: **no se cambia nada**, y el resultado se escribe en
  `sync_model.h` como hallazgo negativo, para que nadie vuelva a proponerlo.
  El costo de este ítem es entonces un test run, que es exactamente el punto.

Lo que NO se hace aquí: acortar el intervalo del CCP. La fuente de
alimentación ya no sostiene el segundo TX que tiene (hallazgo de batería/CCP
en `CLAUDE.md`). El experimento de ALARGARLO a 2 superframes (remedio 2 de
`docs/anchor-sync-measurement.md` §5) sigue pendiente y sigue siendo la forma
de saber si el piso es wander del cristal o ruido de timestamp; es su propio
ciclo, no este.

---

## 3. Cadencia y latencia: el grupo nunca se libera temprano, y `dt` es falso

Dos defectos separados, en el mismo camino.

### 3.1 `POS_MAX_ANCHORS` no es "las anclas que hay"

`tdoa_collect_take_ready()` libera un grupo temprano solo si
`g->n >= POS_MAX_ANCHORS`, que es **4**, una constante de compilación. El
despliegue tiene **3** anclas vivas (`0x0003` no contestó enumeración en tres
intentos). Consecuencia medible: **ningún grupo se libera temprano, nunca**.
Todo fix espera los 150 ms completos de ventana y después hasta 200 ms más al
siguiente `tdoa_gw_step()`.

- Latencia: hasta ~350 ms.
- Cadencia: cuantizada al superframe de 200 ms y batiendo contra el blink de
  5 Hz — el intervalo entre fixes publicados alterna, sin que el tag haya
  cambiado nada.

**Decisión: la cuenta esperada es un dato de despliegue, no de compilación.**
`tdoa_collect` gana `tdoa_collect_set_expected(c, n)`, con `n` tomado de
`apos_store_get()->n_nodes` (el gateway ya lee ese struct en `anchor_xyz()`).
Se sigue acotando por `POS_MAX_ANCHORS` y por `TDOA_MIN_ANCHORS`.

Riesgo aceptado y declarado: un ancla **inventariada pero apagada** deja
`expected` alto y devuelve el comportamiento de hoy (esperar la ventana
completa). Eso no es una regresión — es exactamente lo que ya pasa — y la
alternativa (liveness tracking dentro del colector) le daría al módulo un
reloj y una noción de sesión que su cabecera dice explícitamente que no tiene.

### 3.2 `now_ms` del lazo del gateway no es el tiempo del BLINK

Un modelo de velocidad constante necesita el `dt` REAL. `now_ms` es el instante
en que el gateway drenó la cola, cuantizado a 200 ms y sujeto al batido de
§3.1: alimentar eso a un filtro CV fabrica velocidad que no existe.

**Decisión: `dt` sale del `t_dtu` ABSOLUTO del ancla de referencia**, guardado
en `tdoa_gw.c` ANTES de que `tdoa_dtu_rebase()` lo convierta en diferencias.
Es un timestamp de reloj común, en hardware, a 15.65 ps. No requiere cambiar
la API de `tdoa_collect`: el gateway ya tiene `m[]` absoluto en la mano.

Disciplina obligatoria, igual que en todo este árbol: el contador es de 40
bits y **envuelve cada ~17.2 s**, así que la resta es diferencia con signo
sobre 40 bits (el patrón `sdelta40()` de `sync_model.c`), no una resta plana.

Dos casos que NO son un `dt` válido y fuerzan re-seed del filtro en vez de
propagarse:

1. `dt <= 0` (reordenamiento, o el mismo blink dos veces).
2. `dt` mayor que `TDOA_DT_MAX_MS` — un reboot del gateway re-basa
   `sync_model` y la base maestra salta; un tag que reaparece tras minutos no
   tiene velocidad que extrapolar. Valor propuesto: **2000 ms** (diez blinks a
   5 Hz), el mismo orden que `TDOA_GW_SEED_AGE_MS`.

El ancla de referencia (`m[0]`) es hoy la que el colector guardó primero, o sea
arbitraria. Para `dt` da igual: el spread de propagación en un arreglo de 2.5 m
es ~8 ns contra un `dt` de 200 ms. Para el CONDICIONAMIENTO del solve no da
igual, pero eso es el ítem 4 y está fuera de alcance aquí.

---

## 4. El filtro: EKF sobre DIFERENCIAS de rango, en el gateway, por tag

### 4.1 Se reusa `pos_ekf`, y la propiedad del archivo se MUEVE

`tag_testting/src/pos_ekf.{c,h}` ya existe: estado `[x, y, vx, vy]`, velocidad
constante con ruido blanco de aceleración, updates escalares secuenciales, sin
ninguna inversa de matriz, C puro sin Zephyr, con un suite host de 652 líneas
(`tag_testting/tests/pos_ekf/`). Se escribió para el tag y **hoy es código
muerto ahí**: la Fase 3 movió el solve al gateway.

**Decisión: la propiedad de `pos_ekf.{c,h}` se MUEVE a ANCLA, no se copia.**
Esto rompe deliberadamente con la regla verbatim-copy que rige
`uwb_frame_802_15_4z.c`, `cal_math.c` y `pos_solver.c`, y la razón es que esa
regla existe para archivos con **dos consumidores vivos**. Aquí queda uno.
Copiar y luego agregarle una función produciría exactamente el modo de falla
que `CLAUDE.md` documenta para `cal_math.c`: una bifurcación silenciosa donde
un cambio de un lado desactiva una comprobación del otro.

Consecuencias que hay que ejecutar, no solo declarar:
- El `CLAUDE.md` de ANCLA declara la propiedad; el de `tag_testting` la cede.
- La copia del tag se marca muerta (o se borra) en el mismo ciclo.
- Esto **no** cierra la transferencia pendiente de `pos_solver`/`pos_residual`
  que el `CLAUDE.md` de ANCLA ya tiene abierta — es un archivo distinto. Pero
  sí sienta el precedente, y el diff de esas dos copias debería revisarse
  cuando alguien toque ambos `CLAUDE.md` a la vez.

### 4.2 Se filtran las MEDICIONES, no la posición resuelta

La cabecera de `pos_ekf.h` ya argumenta esto para rangos y el argumento
transfiere entero a diferencias de rango: filtrar la salida de mínimos
cuadrados descarta la geometría, cuyo error es no-gaussiano, correlacionado con
el GDOP, y **cambia de carácter cada vez que la cuenta de anclas salta 3<->4** —
que en este despliegue pasa de rutina.

Función nueva, mecánica idéntica a `pos_ekf_update_ranges()`:

```c
int pos_ekf_update_tdoa(struct pos_ekf *f, const struct pos_ekf_cfg *c,
                        const struct tdoa_meas *m, size_t n);
/* Para cada i >= 1, un update escalar secuencial, con gate a c->gate_k sigmas:
 *   h = r_i - r_0
 *   H = [ (x-x_i)/r_i - (x-x_0)/r_0 ,  (y-y_i)/r_i - (y-y_0)/r_0 , 0, 0 ]
 *   z = (t_i - t_0) * TDOA_M_PER_DTU
 * Devuelve cuántas ecuaciones se aceptaron. */
```

`r_i` usa `dz` con la misma convención que `struct pos_meas` (z del ancla menos
z del tag), o sea la misma que `tdoa_solve.c::slant()`.

**TRAMPA 1 vuelve a aplicar aquí:** `(t_i - t_0)` se resta en `int64_t` ANTES
de cualquier conversión a float. Son tiempos de dispositivo de hasta ~2^40, y
una mantisa de 24 bits solo resuelve ~65536 DTU (307 m) a esa magnitud —
convertir primero y restar después destruye la medición y devuelve un número
plausible. Es la misma trampa que `tdoa_solve.c` documenta dos veces, y no
hereda automáticamente.

Nota de acoplamiento: esta función mete una dependencia de `pos_ekf.c` hacia
`tdoa_solve.h` (por `struct tdoa_meas` y `TDOA_M_PER_DTU`). Ambas cabeceras son
C puro sin Zephyr, así que el suite host sigue compilando con gcc plano; es la
razón por la que la firma toma `struct tdoa_meas` en vez de inventar un tipo
nuevo.

### 4.3 Campo de configuración nuevo: `r_tdoa`, no reusar `r_range`

`r_range` es la desviación estándar de un rango. La medición aquí es una
DIFERENCIA de dos timestamps independientes, así que su sigma es ~sqrt(2) veces
la de un timestamp: a ~1.5 ns de sincronía por ancla (~45 cm), la diferencia
sale a **~0.64 m**. Reusar `r_range` (tuneado contra rangos TWR del tag) le
diría al filtro que confíe en la medición mucho más de lo que merece, y el
resultado se vería *menos* suave, no más.

`struct pos_ekf_cfg` gana `float r_tdoa;`, con default **0.6 m**, declarado como
derivado del jitter medido de Fase 2 y por lo tanto **acoplado al resultado de
§2**: si el shift del EMA baja el jitter, este default se re-deriva.

### 4.4 Dónde vive el estado

En `struct tag_memo` de `tdoa_gw.c` — que ya es la ranura por tag, ya es LRU de
16, y ya está `static`. `struct pos_ekf` mide ~84 B, así que 16 ranuras son
~1.3 kB más en `.bss`.

**Debe ser `static`, no automático.** No es preferencia: un automático de
2588 B en este mismo hilo ya desbordó el stack de 4096 B de `main` en total
silencio (sin línea de log, sin dump, sin reporte de ISR de `k_timer`); el
hallazgo completo está en `CLAUDE.md`. `tdoa_collect` ya son 1792 B estáticos
por la misma razón.

### 4.5 `tdoa_solve()` no se borra: se degrada a sembrador

Sigue siendo la única forma de arrancar sin prior, y la vía de recuperación:

```
grupo listo
  -> filtro sembrado y dt válido?
       sí -> pos_ekf_predict(dt) ; pos_ekf_update_tdoa()
       no -> tdoa_solve() ; pos_ekf_seed(x, y)
  -> pos_ekf_needs_reseed()?  -> tdoa_solve() ; pos_ekf_seed()
  -> publicar pos_ekf_get()
```

**Beneficio de robustez que no es suavizado:** el gate de innovación a
`gate_k` sigmas rechaza de forma natural la solución espejo que
`tdoa_solve.h` advierte que puede converger reportando `valid = true`. Hoy la
única defensa es el gate de salto de 10 m de `tdoa_gw.c`, que necesita una
semilla fresca — así que el PRIMER fix de un tag no tiene ninguna. El filtro no
cierra ese hueco (el primer fix sigue siendo un `tdoa_solve()` desnudo), pero
sí cubre todos los siguientes con un criterio estadístico en vez de un umbral
fijo.

El gate de salto de 10 m se **conserva** tal cual. Es la defensa del camino de
siembra, que es justo el que el filtro no cubre.

### 4.6 Sin ZUPT en este alcance, y qué se pierde

`pos_ekf_zupt()` ya está escrito y su propia cabecera lo llama "la mayor mejora
visual disponible, porque un tag quieto es el caso común". Necesita el estado
del LIS2HH12 del tag en aire — `BLINK_FLAG_MOVING` sobre los 7 bits libres de
`blink_frame.flags` — y eso es cambio de formato en aire, los dos repos, y
`UWB_PROTO_VER` 4->5. **Diferido por decisión de alcance (ítem 6).**

Sustituto en este alcance: programar el ruido de proceso desde la propia
estimación de velocidad del filtro (`sigma_a_still` vs `sigma_a_move`), con
histéresis para que no vibre en el umbral. Es peor que ZUPT y hay que decirlo:
un filtro que decide "quieto" a partir de su propia velocidad es un lazo
cerrado sobre sí mismo, y puede quedarse pegado en "moviéndose" bajo ruido alto
— exactamente el caso donde ZUPT ayudaría más. Se acepta a cambio de cero
cambio en aire.

---

## 5. Lo que este documento deja explícitamente abierto

Los ítems 4-8 del análisis de 2026-09-02, sin re-derivar:

- **Ítem 4** — `tdoa_solve.c` es estrictamente más débil que `pos_solver.c`:
  no tiene gate de gradiente (`POS_GN_GRAD_EPS`), no tiene búsqueda de línea
  (`POS_GN_MAX_HALVINGS`), el ancla de referencia es arbitraria, y no hay
  ponderación. `obs.quality` (ipatovAccumCount) se publica y se descarta, y
  `sync_model_error_dtu()` existe en cada ancla y nunca se publica.
- **Ítem 5** — modelo de altura. Un survey en `APOS_GEOM_2D` fija todo z en 0,
  así que `dz = 0` para todas; anclas en techo y tag en persona es un sesgo
  real que una constante de altura asumida quita.
- **Ítem 6** — `BLINK_FLAG_MOVING` + ZUPT.
- **Ítem 7** — ponderación desde `quality` y sigma de sincronía publicada.
- **Ítem 8** — campaña de retardo de antena **RX**. Es el techo de exactitud:
  ninguna medición TWR de ningún tipo puede restringirlo, y es la diferencia
  entre ~45 cm y ~15 cm absolutos.

## 6. Cómo se verifica, y qué NO prueba

Host, y es donde vive casi todo: `tests/sync_model/` (barrido de shift),
`tests/tdoa_collect/` (liberación temprana con `expected` < `POS_MAX_ANCHORS`),
`tests/pos_ekf/` (el suite del tag, migrado y extendido con el update TDoA
contra trayectorias sintéticas y con ruido de 0.6 m).

Hardware, y sin esto nada de lo anterior está verificado: 3 anclas + gateway +
un tag, quieto y caminando, comparando la traza publicada antes y después.
`kernel reboot cold`. Y la advertencia de siempre: **una traza más suave no es
una traza más exacta.** Sin cinta métrica o punto de verdad, esto mide
precisión (repetibilidad), igual que la Tarea 7. La exactitud de ~45 cm sigue
siendo un objetivo, no un número verificado.
