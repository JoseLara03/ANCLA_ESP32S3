# Más anclas en modo BLINK — diseño

**Contexto:** `docs/superpowers/specs/2026-08-30-blink-slotted-mac-design.md` (una
celda corre en modo TWR o en modo BLINK, nunca ambos) y el análisis de
2026-09-02 que abrió
`docs/superpowers/specs/2026-09-02-tdoa-accuracy-filter-design.md`, cuyo §1
identificó el **GDOP del arreglo de 1.2-2.5 m como el término #1 de exactitud**
y lo declaró explícitamente fuera de alcance porque no lo arregla ni el filtro
ni la sincronía. Este documento es el que lo ataca.

**Objetivo:** que una celda en modo BLINK pueda desplegar más de 4 anclas, con
un techo determinado por presupuesto MEDIDO y no por una constante heredada de
TWR.

**Secuencia:** este trabajo **empieza después de que cierre**
`docs/superpowers/plans/2026-09-02-tdoa-accuracy-filter.md`, y no en paralelo.
Dos razones concretas, no de orden estético:

1. `pos_ekf_update_tdoa()` corre `n-1` updates escalares secuenciales por fix.
   A 4 anclas son 2; a 8 son 7. Multiplicar la cuenta de anclas antes de que
   ese lazo exista y esté medido en el lazo `K_PRIO_COOP(0)` es medir dos
   cambios a la vez y no poder atribuir el costo.
2. Subir `POS_MAX_ANCHORS` toca `pos_solver.h`, que sigue siendo copia
   verbatim del tag. El plan anterior ya decide mover la propiedad de
   `pos_ekf` y sienta el precedente; este cierra la transferencia de
   `pos_solver`/`pos_residual` que `CLAUDE.md` tiene abierta. Hacerlo antes
   sería re-litigar una decisión que el plan anterior está tomando.

---

## 0. Lo que este documento NO reabre

- **Una celda es TWR o BLINK, nunca ambas.** Decisión de
  `2026-08-30-blink-slotted-mac-design.md`. Todo lo de aquí aplica solo al
  modo BLINK.
- **SS-TWR y no DS-TWR entre anclas para el survey.** Decisión de
  `2026-08-14-anchor-auto-positioning-design.md` §4.
- **El contrato de payload de posición** `{"Tid","x","y","z"}`, congelado con
  la plataforma. Más anclas no agregan campos.
- **`APOS_MAX_NODES = 8`** ya está elegido y ya está host-testeado a 8 nodos.
  Este documento lo consume; no lo re-deriva ni lo sube.

---

## 1. Lo que TDoA ya liberó, verificado en código

`CLAUDE.md` documenta cuatro blockers para crecer más allá de 4 anclas. **Tres
son de la era TWR y el modo BLINK los desactiva.** Verificado, no supuesto:

- **El tag en modo BLINK salta DISCOVERY entera.**
  `tag_testting/src/uwb_net.c:307`: `if (c->blink_mode) { c->state =
  UWB_ST_RANGING; return UWB_ACT_NONE; }`, con el comentario *"TDoA has no
  tag<->anchor binding at all: a blinking tag needs no anchor list"*. Por lo
  tanto el stagger de `disc_schedule.h` y el `TX_COMPLETE_TIMEOUT_MS = 18` de
  `anchor_respond.c:47` — el par cuyo modo de falla era perder **en silencio**
  toda respuesta DISCOVERY de un `anchor id` >= 5 — **no acotan la cuenta de
  anclas en una celda BLINK**.
- **El tag ya no emite `0xEA` POS**, así que el camino que valida `n_anchors`
  contra `UWB_FRAME_MAX_ANCHORS` en el codec no lo ejerce TDoA. Ver §3.2, que
  es donde ese mismo constante sí muerde por otra vía.
- **El BLINK no lleva lista de anclas.** 14 bytes: `src_addr`, `seq`,
  `batt_soc`, `flags` (`blink_frame.h`). Cada ancla publica su propia
  observación por MQTT. **El formato en aire dejó de ser el límite**, que era
  justo el blocker que `apos_gw.h:290` cita como "on the far side of a frozen
  wire format".

Corolario que hay que decir en voz alta: **crecer anclas ya no requiere tocar
el firmware del tag ni un solo byte de formato en aire.** Eso es lo que hace
este trabajo viable ahora y no lo era antes de la Fase 3.

---

## 2. El cap se vuelve dependiente del MODO

`UWB_MAX_ANCHORS` (4) hoy hace dos trabajos a la vez: define el **espacio de
identidad** (`anchor id` 0..3, short addr = `UWB_ANCHOR_ADDR_BASE + id`) y
**acota el camino TWR** cuyo stagger no llega más lejos. Al crecer solo en modo
BLINK, esos dos trabajos se separan.

**Decisión: `UWB_MAX_ANCHORS` pasa a ser el espacio de identidad y sube a 8;
se introduce `UWB_TWR_MAX_ANCHORS = 4` como el cap del camino TWR, y se
APLICA, no se comenta.**

Alternativa descartada: dos constantes paralelas (`UWB_MAX_ANCHORS` 4 y
`UWB_MAX_ANCHORS_BLINK` 8) con el direccionamiento colgado de la primera. Eso
deja el short address de un ancla dependiendo del modo de la celda, que es
exactamente la clase de acoplamiento que produjo el defecto de `short_addr`
mode-aware que ya se arregló una vez (commit d4c1f3e).

Cómo se aplica, y por qué así:

- `anchor id` acepta 0..7, pero **avisa fuerte** al fijarlo y otra vez en el
  banner de boot si el id es `>= UWB_TWR_MAX_ANCHORS`. El aviso tiene que decir
  el síntoma, no solo el número: *"en una celda TWR esta ancla no contestará
  DISCOVERY y se verá como una falla de RF"*.
- Los responders TWR (`anchor_respond_discovery()`, el responder WAVE)
  **rehúsan** si el id está fuera del cap TWR, en vez de contestar tarde. Una
  negativa registrada es diagnosticable; una respuesta que llega después de
  que el iniciador cerró su ventana no lo es. Mismo criterio que
  `anchor_respond_wave_poll()` ya aplica al rehusar sin posición.
- **`BUILD_ASSERT` que ate `TX_COMPLETE_TIMEOUT_MS` a
  `disc_resp_delay_uus(UWB_TWR_MAX_ANCHORS - 1)`.** Hoy el 18 es un número
  derivado a mano cuya derivación vive solo en prosa, en dos archivos. Que
  falle al compilar es una mejora independiente de este trabajo: es
  precisamente el acoplamiento que `CLAUDE.md` describe como capaz de perder
  cada respuesta DISCOVERY en silencio.

---

## 3. Lo que sí bloquea, y la decisión para cada uno

### 3.1 `POS_MAX_ANCHORS` (4) y la propiedad de `pos_solver`

`pos_solver.h:9`. Acota el `meas[POS_MAX_ANCHORS]` de cada grupo en
`tdoa_collect`, la cota de `n` en `tdoa_solve()`, y el `m[]` de `solve_one()`.
Sube a 8.

`anchor_bits` en `struct tdoa_group` es `uint32_t`, así que aguanta 32 anclas
sin tocarlo. No es un límite.

**Esto fuerza cerrar la transferencia de propiedad que `CLAUDE.md` deja
abierta.** `pos_solver.h` es copia verbatim del tag, y el tag ya no resuelve su
propia posición desde la Fase 3. Subir la constante de un solo lado es
literalmente el modo de falla de `cal_math.c`: un cambio unilateral que deja
inalcanzable una comprobación del otro lado, sin que nada falle ruidosamente.
La propiedad se mueve a ANCLA, con el mismo procedimiento que el plan anterior
aplica a `pos_ekf`, y se hace **en el mismo ciclo** que el diff de las dos
copias se revisa.

### 3.2 `pos_sink_publish()` descarta todo fix con `n_anchors > 4`

`pos_sink.c:15`:

```c
if (fix->n_anchors < 3 || fix->n_anchors > UWB_FRAME_MAX_ANCHORS) {
        LOG_WRN("POS from 0x%04X: n_anchors=%u out of range, dropping", ...);
        return;
}
```

**Este es el hallazgo que más fácil se pasa por alto y el que rompería el
despliegue completo.** El gate se escribió para sanear un byte que llegaba en
un frame `0xEA` **recibido**, y su comentario lo dice. Pero `tdoa_gw.c` llena
`fix.n_anchors` **localmente** desde `res.n_used`. Con 5 o más anclas
resolviendo, cada fix TDoA muere aquí — y la única evidencia sería un `LOG_WRN`
por fix, en el lazo `K_PRIO_COOP(0)`, que con `CONFIG_LOG_MODE_OVERFLOW`
sobreescribe justo las líneas que el operador estaría leyendo.

**Decisión: el gate se conserva pero se separa de `UWB_FRAME_MAX_ANCHORS`.** El
límite legítimo de un `n_anchors` que llegó por aire sigue siendo el del frame;
el de un fix construido en casa es `POS_MAX_ANCHORS`. Son dos cotas distintas
que hoy comparten un nombre por accidente histórico. Y el `LOG_WRN` pasa a
**una vez por boot** con contador, como los cinco `warned_*` que `tdoa_gw.c` ya
tiene, por la razón que ese archivo ya documenta.

### 3.3 `apos_node.c` rehúsa un `RANGE_CMD` a un peer fuera de 0..3

`apos_node.c:464-465`: `peer_addr >= UWB_ANCHOR_ADDR_BASE + UWB_MAX_ANCHORS`.
Al subir `UWB_MAX_ANCHORS` a 8 el gate se abre solo. **No relajar el gate por
otra vía**: es una de las dos condiciones que sostienen la propiedad de
seguridad de que un ancla desplegada nunca inicia una transmisión no
solicitada, propiedad que se movió del set de compilación a estos gates cuando
`ss_initiator.c` entró a la imagen de producción.

Nota que ahorra trabajo: **el resto del survey ya está dimensionado para 8
nodos.** `apos_gw.h:40` lo dice con estas palabras: *"56 ordered pairs at
~300 ms each is under 20 s"* para "the largest supported deployment", y
`tests/apos_geom/` ya prueba 8. El survey no es el cuello de botella.

### 3.4 `APOS_ENUM_SLOTS` (8) sí es un cuello, y la aritmética cambia mucho

`apos_gw.h:53` documenta la garantía actual: *"With 8 slots, 4 anchors and 3
independent draws, the chance an anchor is lost in all three is under 2%"*. Esa
cuenta no sobrevive a 8 anclas.

Probabilidad de que un ancla dada colisione en una ronda, con `k` anclas en `s`
slots, es `1 - ((s-1)/s)^(k-1)`; en 3 rondas salteadas independientes se eleva
al cubo:

| anclas | slots | por ronda | perdida en 3 rondas |
|---|---|---|---|
| 4 | 8 | 0.393 | **1.9 %** (lo documentado hoy) |
| 8 | 8 | 0.607 | **22.4 %** |
| 8 | 16 | 0.362 | 4.8 % |
| 8 | 32 | 0.201 | **0.8 %** |

**Decisión: `APOS_ENUM_SLOTS` sube a 32.** Un 22% de probabilidad de que un
ancla no aparezca en la enumeración es inaceptable, y ya hay precedente de
cuánto cuesta: el `apos run` de 2026-08-26 corrió con `0x0003` fuera y el
resultado fue un survey de 3 nodos donde `rms_mm` es cero por construcción.

Costo, que hay que declarar: el stagger peor caso pasa de `8 * 30 = 240 ms` a
`32 * 30 = 960 ms`, así que `APOS_GW_ENUM_GAP_MS` (400) tiene que subir por
encima de eso — propuesta **1200 ms** — y la enumeración completa pasa de
~1.2 s a ~3.6 s. Es una operación de una sola vez en la puesta en marcha; no
toca ningún camino en régimen. `APOS_GW_WINDOW_S` (120) lo cubre de sobra.

Alternativa considerada y no elegida: bajar `APOS_ENUM_SLOT_MS` para que quepan
más slots en el mismo tiempo. Ese número está dimensionado contra el airtime de
un `ENUM_RSP` (~1.5 ms) más el jitter de un `k_sleep()` en el lazo del SLAVE;
recortarlo cambia una constante medida para ahorrar segundos en una operación
que ocurre una vez.

### 3.5 Memoria, y por qué se mide en vez de calcularse

`struct tdoa_meas` son 24 B (3 floats, padding, un `int64_t`). Subir
`POS_MAX_ANCHORS` a 8:

- `struct tdoa_collect`: de 1792 B a ~3.3 kB. Es `static` y debe seguirlo
  siendo.
- El `struct tdoa_meas m[POS_MAX_ANCHORS]` **automático** de `solve_one()`: de
  96 B a 192 B, en el stack de 4096 B de `main`, que ya pica en 1748 B durante
  el `do_solve()` del survey.

Sumado al ~1.3 kB de `.bss` que el plan anterior agrega por los EKF por tag,
son ~2.8 kB nuevos. **Nada de esto se acepta por aritmética.** Un automático de
2588 B en este mismo hilo ya desbordó ese stack en silencio total, y
`CLAUDE.md` registra por qué la aritmética no alcanza: el ABI ventaneado de
Xtensa derrama ventanas de registros encima de cada frame declarado, así que la
marca de agua real siempre está arriba de lo que `-fstack-usage` puede mostrar.
`CONFIG_THREAD_ANALYZER` es el único instrumento que responde.

---

## 4. EL GATE: anclas y tags se pelean el mismo presupuesto

**Esta sección es la razón por la que este documento no empieza por
implementar.** Igual que el gate de Fase 2 midió antes de dejar avanzar la
sincronía, aquí hay un número que decide el techo y que **nadie ha medido**.

La carga de observaciones es lineal en anclas:

```
observaciones por superframe = n_anclas * blink_hz * n_tags * 0.2
```

`CLAUDE.md` ya fija el punto de operación de hoy: `TDOA_GW_INGEST_MAX` (32),
`TDOA_GW_SOLVE_MAX` (8) y `OBS_QUEUE_DEPTH` (32) sostienen **8 tags a 5 Hz
sobre 4 anclas**, y los tres se mueven juntos o no se mueven.

| anclas | tags | blink | obs/superframe | obs/s |
|---|---|---|---|---|
| 4 | 8 | 5 Hz | 32 | 160 |
| 8 | 8 | 5 Hz | 64 | 320 |
| 4 | 100 | 5 Hz | 400 | 2000 |
| 8 | 100 | 5 Hz | **800** | **4000** |

Tres consecuencias, en orden de cuánto duelen:

1. **Duplicar anclas divide a la mitad los tags que el camino de ingesta
   aguanta**, a constantes fijas. A 8 anclas, los 32 de `TDOA_GW_INGEST_MAX`
   sostienen 4 tags, no 8. El exceso NO aparece como `reject_shed`: se pierde
   aguas arriba como `rx_drop_evict` (`blink stats`), que es un contador
   distinto en otro módulo — o sea que el síntoma de haber crecido las anclas
   se manifiesta en un lugar donde nadie lo estaría buscando.
2. **`tdoa_collect_take_ready()` libera a lo más UN grupo por llamada**, y
   `TDOA_GW_SOLVE_MAX` la itera 8 veces por superframe: 40 fixes/s, o sea 8
   tags a 5 Hz. Ese techo es independiente de la cuenta de anclas, pero el
   **costo de cada solve** sí crece con ella (`n-1` ecuaciones en Gauss-Newton
   y otras tantas en el EKF). Para 100 tags a 5 Hz harían falta ~100 grupos por
   superframe, es decir ~100 solves cada 200 ms en el lazo cooperativo que arma
   el beacon. **Ese número no se ha medido ni a 4 anclas.**
3. **A 8 anclas y 100 tags el backhaul MQTT lleva ~4000 mensajes/s** de ~66 B
   de payload, o sea ~264 kB/s antes de overhead de MQTT y TCP, sobre el WiFi
   de un ESP32-S3 donde el blob de WiFi corre a prioridad preemptible por
   debajo del lazo del gateway. Es plausible que el cuello de botella real del
   sistema completo deje de estar en el aire UWB y pase al backhaul, que es
   precisamente la superficie que la migración a TDoA construyó y nunca cargó.

**Decisión: la primera tarea del plan es un gate MEDIDO, y su resultado puede
ser "8 anclas sí, pero no a 100 tags".** Ese sería un resultado legítimo y
declarable, no un fracaso: fija el techo real del producto en vez de dejar dos
objetivos (100 tags, más anclas) que nadie ha comprobado que sean simultáneos.

Y hay que decirlo derecho: **el objetivo de 100 tags sigue siendo aritmética de
schedule, no un número medido** — la Tarea 6 del plan de blink-slotted-mac
sigue abierta justo por densidad, con 3 tags corridos contra un objetivo de
100. Este documento no puede cerrar eso y no lo intenta; sí necesita saber
dónde cae el presupuesto antes de multiplicar el multiplicando.

---

## 5. Lo que NO cambia

Vale enumerarlo porque es lo que hace este trabajo más chico de lo que suena:

- **Cero cambios en `tag_testting`.** Un tag que blinkea no conoce anclas.
- **Cero cambios de formato en aire.** Ni `UWB_PROTO_VER`, ni el BLINK, ni el
  beacon, ni ningún frame `0xEx`.
- **Cero cambios en el contrato MQTT de posición.**
- **El árbol CCP no tiene límite de cuenta**: el CCP es un broadcast y cada
  ancla lo recibe por su cuenta, así que el costo de aire no crece con las
  anclas. Lo que sí crece es la cantidad de enlaces de sincronía que hay que
  verificar, y cada ancla nueva es una que puede tener `no_sync == rx` sin que
  se note entre las demás.
- **`pos_json_blink_parse()` ya no acota `anchor_id` a `UWB_MAX_ANCHORS`** —
  solo a `0xFF` (`pos_json.c:267`), aunque el comentario de `pos_json.h:49`
  afirme lo contrario. Un sitio menos que cambiar, y un comentario que hay que
  corregir para que deje de mentir.

---

## 6. El beneficio, y cómo se sabrá que llegó

- **GDOP.** Es el término #1 de exactitud del análisis de 2026-09-02 y no lo
  toca nada más. Más anclas, y más separadas, es el único arreglo.
- **`rms_mm` deja de ser cero por construcción.** En 2D los parámetros libres
  son `2N-3` contra `N(N-1)/2` aristas: a N=3 es isostático (0 sobrantes), a
  N=4 hay 1, a N=8 hay **15**. Recién ahí el survey tiene una señal de calidad
  propia en vez de "nada contradijo los rangos", y `worst_i`/`worst_j` empieza
  a poder nombrar la arista culpable.
- **`residual_m` por fix deja de ser cero por construcción.** El contrato de
  `tdoa_solve.h` dice que solo desde `n_used == 4` la cifra informa algo. Hoy
  `tdoa_gw.c` la pone en cero explícitamente y avisa una vez por boot. A 5+
  anclas se vuelve una señal real por fix, que es la primera que este sistema
  tendría.

Ninguno de los tres se declara logrado sin medición. Y el de siempre: **más
anclas mejora GDOP, que es geometría; no toca el sesgo de retardo de antena RX
(ítem 8 del análisis), que es el otro techo de exactitud y sigue sin campaña.**

---

## 7. Lo que queda abierto

- **Hardware.** Esto no se puede verificar sin al menos un board más. El
  despliegue son 5 boards (4 anclas + gateway) y `0x0003` **no contesta
  enumeración** desde 2026-08-26, así que ni el caso de 4 anclas — el primero
  donde `rms_mm` significa algo — está probado. Arreglar `0x0003` es
  precondición, no trabajo paralelo.
- Los ítems 4-8 del análisis de 2026-09-02, sin re-derivar: endurecer
  `tdoa_solve` (gate de gradiente, búsqueda de línea, ancla de referencia
  determinista, ponderación), modelo de altura, `BLINK_FLAG_MOVING`/ZUPT, y la
  campaña de retardo de antena RX.
- **Cuántas anclas de verdad.** Este documento implementa hasta
  `APOS_MAX_NODES` (8) y deja el tamaño desplegable colgado del gate de §4.
  Pasar de 8 es otro ciclo: `APOS_MAX_NODES`, `APOS_MAX_EDGES` y la memoria de
  `apos_geom` crecen cuadráticamente.
