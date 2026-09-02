# Precisión TDoA, parte 2 — diseño

**Contexto:** `docs/superpowers/specs/2026-09-02-tdoa-accuracy-filter-design.md`
y su plan, cerrados el 2026-09-02 (`docs/tdoa-accuracy-filter-summary-2026-09-02.md`).
Esa parte 1 puso un EKF de velocidad constante por tag alimentado con
diferencias de rango, arregló la liberación temprana de grupos, y midió que
subir `SYNC_PHASE_EMA_SHIFT` no sirve (hallazgo negativo, registrado). También
encontró dos defectos reales en hardware que ni los host tests ni la revisión
de código vieron.

**Objetivo:** los ítems 4-7 del análisis de 2026-09-02, que la parte 1 dejó
explícitamente listados en su §5 para no re-derivarlos:

- **Ítem 4** — endurecer `tdoa_solve()`: búsqueda de línea, gate de gradiente,
  ancla de referencia determinista.
- **Ítem 5** — modelo de altura (`dz`).
- **Ítem 6** — ZUPT, con un bit de movimiento en el BLINK.
- **Ítem 7** — ponderación por observación.

**Un solo bump de formato en aire** (`UWB_PROTO_VER` 4 -> 5) para los ítems 6 y
7 juntos, en vez de dos ciclos.

**Lo que la parte 1 dejó abierto y aquí es la Tarea 1:** nunca se capturó un
ANTES, y el tag quieto quedó con ~0.8 m de dispersión sobre **4 muestras**. Sin
línea base nada de aquí puede demostrar que mejoró algo — y el tag quieto es
justo lo que el ítem 6 ataca.

---

## 0. Lo que este documento NO reabre

- **El EKF filtra MEDICIONES, no la posición resuelta.** Decisión de la parte 1
  §4.2, ya implementada y verificada en hardware.
- **`SYNC_PHASE_EMA_SHIFT` se queda en 3.** Medido: 8.8% de mejora peor caso
  contra una barra de 20%. Hallazgo negativo registrado junto a la constante
  precisamente para que no se re-proponga.
- **La propiedad de `pos_ekf` NO se movió.** La premisa de la parte 1 §4.1 era
  incorrecta: el tag sigue usándolo en su camino TWR. Rige el precedente de
  `pos_solver.c` (copia compartida, el tag conserva la propiedad). Todo cambio
  a las funciones de rango de `pos_ekf` sigue siendo un cambio de dos repos.
- **El gate de salto de 10 m** (`TDOA_GW_MAX_JUMP_M`) se conserva.
- **El contrato de payload de posición** `{"Tid","x","y","z"}`, congelado.
- **La exactitud de ~45 cm** sigue acotada por GDOP del arreglo y por el
  retardo de antena RX sin calibrar. Nada aquí la mueve, y el ítem 8 (la
  campaña de RX) sigue fuera.

---

## 1. Ítem 4: `tdoa_solve()` puede devolver su propia semilla reportando éxito

**Esta es la sección con más urgencia del documento, y no por robustez
abstracta: es una segunda vía al bug que la ronda 2 acaba de arreglar.**

`src/pos_solver.c` — el solver del tag, mismo esqueleto de ecuaciones normales
2x2 — lleva dos defensas que `tdoa_solve.c` **no copió**, y su propio comentario
dice qué pasa sin ellas:

> *"over 200k adversarial cases this path returned the seed verbatim with
> valid = true and a residual up to 113 m"*

`tdoa_solve.h` ya admite el hueco por escrito ("KNOWN LIMITATION — 'converged'
is not always 'correct'") y lo difirió a propósito. **Lo que cambió desde
entonces es que la parte 1 le pasa una semilla en CADA fix** (la última posición
publicada del tag, vía `memo`). Antes la semilla era normalmente el centroide;
ahora es la respuesta anterior. Así que un solve estancado no devuelve un punto
arbitrario: devuelve **la posición anterior**, con `valid = true`.

**Y la aritmética de contadores que cazó el bug de la ronda 2 no lo detecta.**
Ese control compara `fixes` contra `seeded + filtered + reseed`; un solve
estancado que devuelve su semilla entra por el camino de siembra, cuenta como
`n_seeded`, re-siembra el filtro con el mismo `(x, y)`, y **cuadra**. La
verificación que funcionó una vez es ciega a esta vía.

### 1.1 Búsqueda de línea

Portar `POS_GN_MAX_HALVINGS` (8). El costo es `diff_residual()`, que
`tdoa_solve.c` **ya tiene escrita** — se usa hoy solo para llenar
`out->residual_m`. Cada halving es una evaluación más sobre `n-1` ecuaciones.

Copiar también el razonamiento de dónde se juzga la estacionariedad: sobre el
paso de Newton **sin amortiguar**, antes de que la búsqueda de línea lo toque.
`pos_solver.c` documenta las dos razones (una semilla que ya es la solución
produce `dp ~ 0` y ningún paso mejorante, y eso debe leerse como convergido, no
como estancamiento; y probar el paso amortiguado dejaría que ocho halvings de un
paso legítimo de 2.5 cm caigan bajo el umbral).

### 1.2 Gate de gradiente — el umbral NO se copia, se re-deriva

Portar el gate final de `||J^T r||`, que es el que convierte "la búsqueda de
línea se estancó" en "esto no es un punto estacionario".

**Trampa, y es la razón por la que este bullet existe:** `POS_GN_GRAD_EPS`
(5e-3) está dimensionado para un Jacobiano cuyas filas son cosenos directores,
acotadas por **1** en cada componente. Las filas de `tdoa_solve()` son
DIFERENCIAS de dos cosenos directores — `(x-x_i)/r_i - (x-x_0)/r_0` — acotadas
por **2**, y el residual que multiplican es un residual de diferencia de rango,
no de rango. El umbral es una cantidad con unidades y **copiarlo verbatim es
importar un número dimensionado para otro modelo**. Se re-deriva contra el
sigma real de la medición (`r_tdoa`, 0.6 m) y se pina en un test.

### 1.3 Ancla de referencia determinista

Hoy `m[0]` es la observación que el colector guardó primero, o sea arbitraria y
distinta entre fixes del mismo tag. Tres consecuencias:

- La linealización cambia de referencia entre fixes: el condicionamiento varía
  sin que nada físico haya cambiado, y eso es jitter fabricado.
- `pos_ekf_update_tdoa()` usa el mismo `m[0]` como referencia de sus `n-1`
  ecuaciones escalares, así que hereda el mismo bamboleo.
- El `dt` de la parte 1 sale del `t_dtu` absoluto de `m[0]`. Con referencia
  variable, `dt` se mide contra un ancla distinta cada vez. El error es el
  spread de propagación del arreglo (~8 ns contra un `dt` de 200 ms, o sea
  despreciable) — **no es un defecto, pero deja de existir gratis.**

**Decisión: la referencia es el `anchor_id` MÁS BAJO presente en el grupo**, y
se implementa intercambiándolo a la posición 0 antes de `tdoa_dtu_rebase()`.
Todo aguas abajo (rebase, `plausible`, solve, EKF, `dt`) queda intacto.

Alternativas descartadas, con la razón:

- *La de mejor `quality`*: se voltea entre fixes cuando dos anclas están
  parejas, que es exactamente la inestabilidad que se está quitando.
- *La más cercana al fix anterior*: mejor condicionamiento (un `r_0` chico da
  el gradiente más grande) pero depende del prior, y hace que la geometría del
  solve dependa de su propia salida anterior — un lazo cerrado más, justo lo
  que la parte 1 §4.6 ya aceptó a regañadientes en otro lugar.

Determinismo dado el conjunto de anclas es la propiedad que se quiere; el
condicionamiento es de segundo orden frente a la inestabilidad de voltearse.
Nota honesta: el id más bajo **sí** cambia si esa ancla falta en algunos fixes.
Es determinista dado el conjunto, no invariante — y eso es todo lo que `dt` y la
linealización necesitan.

---

## 2. Ítem 5: el modelo de altura, y por qué un `dz` uniforme NO se cancela

`tdoa_gw.c` pone `t.meas.dz = z` desde el survey. Un survey en `APOS_GEOM_2D`
**fija todo z en 0**, así que hoy `dz = 0` para todas las anclas. Si las anclas
están en el techo y el tag en una persona, la geometría real es 3D.

**La intuición equivocada, y hay que decirla porque es la que gana sola:** "si
todas las anclas están a la misma altura, el `dz` es un offset común y se
cancela en una diferencia de rangos". **Falso.** El modelo es

```
r_i = sqrt(rho_i^2 + h^2)        (rho_i = distancia horizontal)
r_i - r_0 = sqrt(rho_i^2 + h^2) - sqrt(rho_0^2 + h^2)
```

que **no** es `rho_i - rho_0`. La raíz es no lineal: un `h` uniforme comprime
las diferencias de rango, así que un modelo con `h = 0` tiene que moverse a
donde las diferencias horizontales sean menores — o sea **HACIA ADENTRO**, al
centroide de las anclas. Es un sesgo geométrico real, no un offset.

**CORRECCIÓN (2026-09-03).** Una versión anterior de este párrafo decía "hacia
afuera". Está mal, y se corrigió midiendo, no razonando
(`tests/tdoa_solve/test_height_model`):

| arreglo | H | error peor | offset al centro |
|---|---|---|---|
| 10.0 m | 1.5 m | 0.098 m | 4.000 -> 3.902 |
| **2.5 m** | **1.4 m** | **0.230 m** | **1.000 -> 0.770** |
| 2.5 m | 0.0 m | 0.002 m | no-op |

La fila del medio es el arreglo de ESTE proyecto (aristas de 1.2-2.5 m): una
separación de 1.4 m sin modelar encoge el offset al centro un **~23%**, el
mismo orden que todo el objetivo de ~45 cm. En un arreglo de 10 m la misma `H`
es un efecto del 2.5%, que es justo por qué esto se descarta fácil sobre papel.

**Decisión: un solo escalar de sitio**, `dz_i = z_i(survey) - z_tag_asumido`.
En un survey 2D todos los `z_i` son 0, así que se reduce a un `h` uniforme; en
uno 3D respeta las alturas por ancla que el survey sí midió.

**El default es 0.0, no un valor adivinado.** Un `h` equivocado es un sesgo
NUEVO, y arrancar con 1.4 m "porque suena a techo" cambiaría en silencio los
números de todo despliegue existente. Con default 0 el cambio es un no-op hasta
que un operador mida la altura real, que es la misma disciplina que el resto del
proyecto aplica a las constantes no medidas.

Consola: un setter persistido en el árbol que ya existe, no un Kconfig — el
valor es por sitio, no por imagen. Sigue el patrón de `apos zoff`, que ya
resuelve la pregunta hermana (dónde está el piso respecto al plano del gauge) y
**no** sirve para esta: `zoff` mueve el plano del survey, no la separación entre
el plano de las anclas y el del tag.

---

## 3. Ítem 6: ZUPT — mucho más barato de lo que la parte 1 estimó

La parte 1 §4.6 lo difirió estimando que costaba matemática nueva más un cambio
de aire. **La matemática ya existe en los dos lados**, verificado:

- `src/pos_ekf.h` (copia de ANCLA) ya declara `pos_ekf_zupt()` y el campo
  `r_zupt`.
- `tag_testting/src/uwb_net_runner.c:1234` **ya llama** `pos_ekf_zupt()` en su
  camino TWR, gobernado por `motion_moving` — un `volatile bool` que
  `motion.c:40` fija desde la interrupción de actividad del LIS2HH12 y que ya
  alimenta el filtro de tier.

Lo único nuevo es exponer ese bit en el BLINK y llamarlo en el gateway.

**Decisión: el BLINK lleva su byte `flags` COMPLETO a la observación, no un
booleano `moving`.** Mismo costo en bytes y en código, y así `BLINK_FLAG_ALERT`
(`0x01`, que ya existe y hoy el gateway no ve) queda visible gratis y un flag
futuro no necesita campo nuevo.

- `BLINK_FLAG_MOVING = 0x02`; `BLINK_FLAG_RESERVED_MASK` pasa de `0xFE` a
  `0xFC`. El parser ya rechaza cualquier bit reservado, así que un gateway
  viejo rechaza un BLINK con el bit nuevo — de ahí el bump de proto.
- `struct pos_blink_obs` gana `uint8_t flags`, y el JSON un campo `"f"`.
- A nivel de grupo: todas las observaciones de un mismo blink llevan los mismos
  flags (es el mismo frame, oído por varias anclas). Se toma el de la
  observación de referencia. Un desacuerdo solo es posible con corrupción, y
  no se intenta arbitrar: no hay nada que arbitrar bien con 2 o 3 votos.

**Y una remoción, no solo una adición:** la heurística de la parte 1 §4.6 —
programar `sigma_a_move`/`sigma_a_still` desde la velocidad estimada del propio
filtro, con histéresis — **se ELIMINA**, no se deja al lado. Su propio spec la
declaró un lazo cerrado sobre su salida que puede quedarse pegado en
"moviéndose" bajo ruido alto, aceptado solo a cambio de no tocar el aire. Con un
bit real del acelerómetro, dejar las dos peleando por el mismo parámetro es
peor que cualquiera de las dos sola.

**Riesgo declarado:** un tag reportado quieto mientras lo cargan despacio recibe
ZUPT que no le toca, y el filtro se pega. `pos_ekf.h` ya nombra ese caso y la
defensa existente es el streak `reset_after` -> `pos_ekf_needs_reseed()`, que la
parte 1 ya cableó. No se agrega defensa nueva; se verifica que esa dispara.

---

## 4. Ítem 7: ponderación — una mitad es gratis y la otra hay que medirla antes

### 4.1 `quality` ya viaja en el aire, y puede no significar nada

`obs.quality` **ya se publica y ya se parsea** (`pos_json.c:286`); el gateway
simplemente la ignora. Usarla no cuesta ni un byte de formato.

Pero es `diag.ipatovAccumCount` (`uwb_slave.c:120`), el conteo de acumulación
del CIR. **A PLEN fijo puede ser casi constante y no llevar información útil de
calidad**, en cuyo caso ponderar por ella es ruido disfrazado de estadística.

**Decisión: medir la correlación entre `quality` y el residual ANTES de
ponderar con ella.** Misma disciplina que la Tarea 1 de la parte 1, que terminó
en un "no cambiar nada" registrado. Si la correlación es débil, la conclusión se
escribe como hallazgo negativo junto al campo y **`quality` se queda como
diagnóstico**, que es lo que su propio comentario ya dice que es.

### 4.2 La sigma de sincronía sí es principiada, y es campo nuevo

`sync_model_error_dtu(m, ahead)` existe en cada ancla, devuelve una estimación
de 1-sigma en DTU del error de conversión al reloj del master, y **nunca se
publica**. Esa sí es una varianza derivada de un modelo, no un proxy: es
exactamente lo que la `R` de cada ecuación escalar del EKF quiere.

- `struct pos_blink_obs` gana la sigma; el JSON un campo `"e"`.
- `pos_ekf_update_tdoa()` acepta una `R` por ecuación en vez del `r_tdoa` plano.
  La diferencia de rangos combina DOS timestamps, así que la varianza de la
  ecuación `i` es `sigma_i^2 + sigma_0^2` — la referencia entra en **todas**
  las ecuaciones, que es la misma razón por la que `r_tdoa` se derivó con un
  `sqrt(2)` en la parte 1 §4.3.
- `r_tdoa` se conserva como **fallback** para toda observación que llegue sin
  sigma (un ancla en firmware viejo). No se borra.

### 4.3 El techo de 95 bytes es una restricción de versionado, no un tamaño

`POS_JSON_BLINK_MAX_LEN` (96) es también el **rechazo duro** del parser: un
payload de 96 bytes o más se rechaza entero. El peor caso de hoy son 66 bytes,
o sea 29 de holgura. Los dos campos nuevos (`"f"` ~8 B, `"e"` ~10 B) consumen
18 y dejan 11.

**Decisión: subir el techo en el mismo ciclo, y desplegar GATEWAYS ANTES QUE
ANCLAS.** Es la regla que el propio header documenta, y el modo de falla si se
invierte no es degradación: un gateway viejo rechaza **todas** las
observaciones, o sea pérdida total y silenciosa de la entrada TDoA. Con 11 bytes
de holgura restante, dejarlo como está convertiría el siguiente campo en una
emergencia.

---

## 5. El bump de proto

`UWB_PROTO_VER` 4 -> 5 (ANCLA) y `UWB_NET_PROTO_VER` 4 -> 5
(`tag_testting/src/uwb_net.h`), por el bit `BLINK_FLAG_MOVING`, que un parser
anterior rechaza por bit reservado. Los ítems 4 y 5 no tocan el aire; van en el
mismo ciclo por conveniencia, no por dependencia.

Orden de despliegue, y es el único orden seguro: **gateways, luego anclas,
luego tags.**

---

## 6. Qué puede y qué no puede demostrar esta parte

**La Tarea 1 del plan es la captura de línea base**, porque la parte 1 terminó
sin ANTES y con 4 muestras del tag quieto. Sin eso, el ítem 6 — el que más
promete — no tiene contra qué compararse, y se repetiría literalmente el error
de la ronda 3.

Lo que sí se puede afirmar al cerrar:

- Dispersión del tag quieto, sobre una captura larga y con ANTES y DESPUÉS.
- Que `tdoa_solve()` ya no puede devolver su semilla reportando éxito.
- Que el sesgo de altura está modelado (no que sea correcto: eso depende de que
  alguien mida el `h` del sitio).

Lo que **no**: la exactitud absoluta de ~45 cm. Sigue acotada por GDOP del
arreglo de 1.2-2.5 m (que ataca
`docs/superpowers/plans/2026-09-02-blink-anchor-scale.md`, con sus propias
precondiciones) y por el retardo de antena RX sin calibrar (ítem 8, sin campaña).
Y sigue faltando un punto de verdad medido: sin cinta métrica esto mide
precisión.

## 7. Lo que queda abierto

- **Ítem 8**, la campaña de retardo de antena RX. Es el techo de exactitud y
  ninguna medición TWR puede restringirlo.
- Más anclas / GDOP: su propio plan, con sus propias precondiciones (`0x0003`
  sin contestar enumeración, y un board físico adicional).
- El fallback de `r_tdoa` de §4.2 vuelve innecesario el día que toda la flota
  publique sigma; retirarlo es trabajo de otro ciclo, no de este.
