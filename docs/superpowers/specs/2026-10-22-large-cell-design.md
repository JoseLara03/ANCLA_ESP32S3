# Paso 2: celda grande (hasta 254 anclas) con el edge como autoridad del sitio — diseño

**Estado: DISEÑO. Nada implementado, compilado ni corrido en hardware.**

**Contexto.** El paso 1 (`2026-10-21-anchor-eui-identity-design.md`, rama
`feat/anchor-eui-identity`) hizo del EUI la identidad de cada ancla y le dio al
gateway `apos assign`. Ese paso se probó en banco con 4 anclas (2026-10-22, por
el operador). El límite que queda es `UWB_MAX_ANCHORS = 4`, y el objetivo de
producto es cubrir zonas grandes, lo que implica muchas anclas.

**Objetivo.** Una celda (un gateway, una raíz de sincronía) con **hasta 254
anclas**. El número de anclas deja de limitar el sistema: cada pieza queda
acotada por algo local (cuántas anclas oyen un blink, cuántos vecinos tiene un
ancla) y no por el total.

**Fuera de alcance: el paso 3.** Multihop de sincronía y beacon, o varias celdas
con roaming. Este paso asume que **todas las anclas de la celda están al alcance
de radio del gateway, en ambos sentidos** (ver §8, precondición P1).

**Decisiones del operador (2026-10-22):**

- **El edge está siempre presente.** Es la autoridad del sitio: registro de
  anclas, solve del survey y solve de TDoA. El gateway queda para la radio:
  beacon, CCP, `apos assign` y la ejecución de las mediciones del survey.
- **El survey es incremental.** Las anclas ya ubicadas quedan fijas, y solo se
  miden y ubican las nuevas contra sus vecinas. Las coordenadas en producción no
  se mueven al agregar anclas.

**Relación con documentos anteriores.**

- **Reemplaza** a `2026-09-02-blink-anchor-scale-design.md` en lo que toca a
  cuántas anclas y cómo se surveyan. Se conservan de ese documento la separación
  `UWB_TWR_MAX_ANCHORS` y el gate de `pos_sink` (§2.2 aquí).
- **Toma prestado** del trabajo sin publicar de `master` ("network-scaling-v3"):
  el chequeo de rigidez (`7563263`) y la idea de pares candidatos por evidencia
  de vecindad (`b87a767`). No se toma la evidencia por bitmap de 32 bits ni el
  solver denso, que no escalan a 254 (§3).
- **Respeta** el contrato multihop de `docs/sync-multihop-design.md` §9: este
  paso mantiene una sola raíz y un solo nivel, y no cierra ninguna puerta del
  paso 3.

---

## 0. Lo que este documento NO reabre

- **TWR queda en 4 anclas.** El formato del tag (`UWB_FRAME_MAX_ANCHORS`) y el
  escalonamiento de DISCOVERY lo fijan. Una celda grande es una celda blink.
- **SS-TWR entre anclas para el survey**, con aristas del gateway en un solo
  sentido (`docs/sync-multihop-design.md` §3).
- **El payload de posición congelado** y el schema de `uwb/anchor/setup/<zone>`.
  Los nombres siguen siendo `ANC-LOBBY-NNN` con NNN = id + 1 (3 dígitos alcanzan
  para 254).
- **La identidad por EUI y `apos assign`** del paso 1.
- **El formato del BLINK y el firmware del tag:** cero cambios.

---

## 1. Hechos verificados en código (2026-10-22)

1. **El espacio de direcciones alcanza para 254.** Las anclas usan
   `0x0001 + id`, los tags empiezan en `0x0100` y `0x00FF` es UNASSIGNED. Los ids
   asignables son por lo tanto 0..253 (`0x0001`..`0x00FE`). Los campos de 1 byte
   (el `wire_id` del WAVE/ranging, `"a"` en la observación) cubren ese rango
   exacto.
2. **El solver del survey en el ESP32 no escala.** Es un LM denso
   (`apos_geom.c:886`, `JtJ[3N][3N]` estático). Con `master` a 32 nodos ya hizo
   falta recortar memoria (`APOS_MAX_PARAMS = 3N−6 = 90`). Con 254 nodos serían
   756 parámetros, ~2.3 MB solo para la matriz.
3. **La enumeración ciega no escala.** `APOS_ENUM_SLOTS = 64` con 3 rondas. Con
   254 respondedores en 64 slots, casi todas las respuestas chocan.
4. **La evidencia de vecindad de `master`** es un bitmap `uint32_t` de ids
   oídos: alcanza para 32, no para 254.
5. **`tdoa_collect` del firmware** deduplica con `anchor_bits` `uint32_t` y
   `aid < POS_MAX_ANCHORS (4)`.
6. **El collector del edge libera un grupo** cuando junta `len(anchors)`
   observaciones o cuando vence la ventana (`edge/tdoa_server/tdoa.py:298-330`).
   Con 254 anclas en el registro, un grupo nunca llega a "completo" y siempre
   espera la ventana entera.
7. **El canal de comandos firmados** (`cmd_proto`) tiene payloads de ≤ 256 B,
   un challenge de un solo uso que rota cada 10 s y `MIN_GAP_MS = 1000`. Sirve
   para comandos sueltos, no para mandar cientos de trabajos.
8. **Los buffers MQTT del gateway son de 256 B** (`net_uplink.c:84-85`).
9. **`storage_partition` mide 192 KB.** Un mapa de 254 entradas son ~2.5 KB.
10. **La documentación del edge ya lo describe así:** *"so the gateway (ESP32)
    is left with the beacon, the CCP and the survey"* (`edge/README.md`).

---

## 2. Arquitectura

### 2.1 Roles

| nodo | hace | deja de hacer en una celda grande |
|---|---|---|
| **Gateway** | Beacon, CCP, `apos assign`, ejecutar trabajos de radio del survey (rondas de sondeo, `RANGE_CMD`, su propio ranging a cada ancla) y reenviar los resultados al edge | El solve del survey y el solve de TDoA de producción. Se conservan para celdas chicas (≤ `APOS_MAX_NODES`) y como diagnóstico |
| **Ancla** | Recibir blinks y publicarlos; participar en las rondas de sondeo y en el ranging; publicar su tabla de vecinos por MQTT; aplicar su enlace de sync (tau) cuando el edge se lo instala | — |
| **Edge** | El registro del sitio, planear el survey, resolver la geometría incremental, calcular tau, instalar enlaces, solve de TDoA con las mejores K y publicar `setup/<zone>` | — |

### 2.2 Ids y modo TWR

- `UWB_MAX_ANCHORS` pasa a **254** (ids 0..253). Nace
  `UWB_TWR_MAX_ANCHORS = 4` y se **aplica**: DISCOVERY y WAVE rechazan un id
  ≥ 4, con log rate-limited, como ya hacen con UNASSIGNED.
- `anchor id <n>` acepta 0..253 y avisa cuando n ≥ 4 ("en una celda TWR esta
  ancla no contesta DISCOVERY").
- `pos_sink`: el gate de `n_anchors` se separa de `UWB_FRAME_MAX_ANCHORS`, como
  ya decidía el spec del 2026-09-02 §3.2.
- **El TDoA del gateway queda como diagnóstico.** Un ancla con id ≥ 4 que no
  quepa en `tdoa_collect` se cuenta como `no_anchor` y no rompe nada. El solve
  de producción es el del edge.

---

## 3. Decisiones

### D1. El registro del sitio vive en el edge

Es un archivo JSON versionado en el volumen del contenedor
(`edge/data/registry.json`), escrito de forma atómica (archivo temporal más
rename). Una entrada por EUI:

```
eui, id, cell(=zone), state ∈ {seen, assigned, measured, placed, active},
xyz, placed_gen, link{tau_dtu, gen, parent_eui, installed}, last_status, notes
```

- Sus fuentes son el status de cada ancla (EUI, id y estado, que ya llegan por
  `status/<zone>/#`) y los resultados del survey.
- **Un ancla entra al solve de TDoA solo en estado `active`:** ubicada y con su
  enlace instalado y confirmado. Así reemplaza el "hold" global del apply del
  paso B14 por uno **por ancla**. Agregar 10 anclas nuevas no suspende las
  posiciones de toda la celda; las nuevas simplemente no participan hasta
  quedar `active`.
- **El mapa de direcciones sigue en el gateway**, que es la autoridad de radio
  de su celda (paso 1), y el registro lo refleja. Mover esa autoridad al edge es
  del paso 3 (multi-celda).

### D2. Enumeración por cubetas

`SURVEY_BEGIN` gana dos campos: `bucket`/`n_buckets` y una bandera
`unassigned_only`. Un ancla contesta solo si `fnv(EUI) % n_buckets == bucket`,
o si la bandera está puesta y ella no tiene dirección.

- El gateway elige `n_buckets` para que cada cubeta tenga ≤ ~16 anclas
  esperadas (con el mapa como estimación): 254 anclas son 16 cubetas.
- Con 16 respondedores en 64 slots, un ancla choca en una ronda con
  probabilidad 1 − (63/64)^15 ≈ 21 %. Con 3 rondas independientes, la
  probabilidad de perderla en las 3 es ~1 %. Una pasada de verificación con
  `unassigned_only` recoge a las que falten.
- **Puesta en marcha incremental:** `apos assign` con `unassigned_only`
  encuentra solo las placas nuevas, normalmente pocas, en una cubeta, sin
  enumerar las 254. Una auditoría completa (todas las cubetas) es un comando
  aparte.
- **Cambio de formato:** `SURVEY_BEGIN` crece 3 bytes y cambia de longitud, así
  que un ancla con firmware anterior falla el parseo. Es un flag day
  deliberado: toda ancla necesita este firmware para estar en una celda grande.

### D3. Vecindad por rondas de sondeo, y las tablas suben por MQTT

Nuevo subtipo APOS `PROBE_ROUND` (gateway → broadcast: sesión, ronda,
`slot_ms`). Cada ancla asignada responde con un `PROBE` (src = su dirección,
payload = su EUI) en el slot `((id + ronda·k) mod 254) · slot_ms` después del
broadcast.

- Todas las demás anclas registran, por emisor, cuántas veces lo oyeron y su
  nivel de RX (`rx_level`, que ya se lee en el diag del CIA). La tabla es de 254
  entradas × 4 B, ~1 KB.
- `slot_ms` ≈ 3 ms (un frame de ~0.4 ms a PLEN_256 más el margen y la ruta SPI),
  así que una ronda de 254 slots dura ~0.8 s.
- El desfase `ronda·k` (con k coprimo con 254) hace que un id que en una ronda
  cae sobre el beacon (y lo suprime `beacon_guard`) caiga en otro lugar en la
  siguiente. Se hacen 3 rondas.
- **Las tablas no suben por UWB.** Cada ancla las publica por MQTT en
  `uwb/anchor/neigh/<zone>/<EUI16>`, un topic interno nuevo que la plataforma no
  consume. Así se evita mandar 254 tablas por el aire.
- **Seguridad:** un `PROBE` es una transmisión de un ancla que no responde a un
  `RANGE_CMD`, así que es una ruta nueva de TX en producción. Lleva los mismos
  dos gates que el ranging: ventana de survey abierta y sesión que coincide con
  un `PROBE_ROUND` de `0x0000`. Una sola por ronda y por ancla.

### D4. El edge planea; el gateway ejecuta trabajos firmados por lotes

Con las tablas de vecindad, el edge elige los **pares candidatos**:

- Para cada ancla nueva, sus ≤ D vecinos más fuertes entre las ya ubicadas y
  entre las nuevas, con D = 8 por defecto, y siempre lo suficiente para la
  rigidez (D6).
- Una evidencia de un solo sentido basta. Un ancla que no oyó a nadie conserva
  todos sus pares posibles con las que sí la oyeron, por la misma regla
  conservadora de `master` (`b87a767`): la ausencia de evidencia no es
  aislamiento.

El edge manda al gateway **trabajos** por un topic interno firmado
(`uwb/anchor/job/<zone>/gw`):

```
{"job":<id monótono>,"op":"range","pairs":"<hex addr,addr,...>","sig":"<HMAC>"}
{"job":<id>,"op":"probe","rounds":3}
{"job":<id>,"op":"local","addrs":"<hex ...>"}      // gateway -> ancla, para tau
```

- **Autenticación:** HMAC con la misma clave del sitio (`cmd_key`) sobre
  `zone|gw|job|op|args`, y protección contra replay por **id de trabajo
  monótono**, persistido en el gateway. No se usa el challenge de un solo uso,
  que limitaría a un trabajo cada 10 s (§1.7).
- **Lotes de ≤ 24 pares**, para caber en el buffer MQTT de 256 B (§1.8). El
  gateway los encola y los ejecuta con la maquinaria de hoy: un `RANGE_CMD` por
  step, reintentos y la ventana de sordera del beacon.
- **Resultados:** el gateway publica cada uno en
  `uwb/anchor/meas/<zone>` como `{"from":EUI,"to":EUI,"mean":mm,"sd":mm,"n":k}`.
  Ya conoce los EUIs por el mapa. Su propio ranging a cada ancla (para tau) sale
  por el mismo topic con `from` = EUI del gateway.
- **Costo:** con D = 8, cada ancla nueva cuesta ~16 pares dirigidos. Una puesta
  en marcha de 254 anclas desde cero son ~4000 pares × ~0.25 s, **~17 min**.
  Agregar 10 anclas, **~40 s**.

### D5. Ubicación incremental en el edge, sin dependencias nuevas

En Python puro, sin numpy: cada paso resuelve 3 incógnitas.

- **Primera puesta en marcha:** el gauge del operador (por id o EUI; 3 anclas en
  2D, 4 en 3D) se coloca con la forma cerrada que ya existe, portada de
  `apos_geom.c` igual que `tdoa.py` porta `tdoa_solve.c`, y se verifica contra el
  C con el mismo método de `edge/tools/xcheck.py`.
- **Después, por crecimiento:** se repite "toma el ancla no ubicada con más
  aristas hacia ubicadas; si cumple D6, resuélvela con Gauss-Newton, con las
  ubicadas fijas". Al final, unas pocas pasadas de refinamiento solo sobre las
  nuevas (Gauss-Seidel por nodo: cada una se re-resuelve con las demás fijas).
  **Las ya ubicadas nunca se mueven** (decisión del operador).
- **Residual por ancla:** como cada ancla nueva tiene ≥ D aristas contra 3
  incógnitas, su residual es una señal de calidad real, no cero por
  construcción. El reporte la da por ancla, junto con la peor arista.
- **Puntos de control opcionales:** el operador puede fijar coordenadas medidas
  con cinta a algunas anclas (`survey fix <eui> x y z`). Con eso la ubicación en
  cadena no acumula deriva sobre todo el sitio, que es el riesgo real de un
  survey incremental grande (§8).

### D6. Rigidez por ancla, no global

Un ancla se ubica solo si tiene aristas hacia **≥ 4 ancla ubicadas no
coplanares (3D) o ≥ 3 no colineales (2D)**, con el mismo criterio de
condicionamiento que `apos_geom_gauge_conditioning()` aplica al gauge: altura o
distancia a la recta contra el piso de ruido. Si no, queda `measured` y el
reporte dice qué le falta: "2 vecinas ubicadas, necesita 3".

La rigidez global de `master` (conexión, grado mínimo y conteo de aristas) se
reporta para todo el sitio. El chequeo por ancla es el que bloquea, porque en un
survey incremental es el que decide.

### D7. Instalación de enlaces por comando firmado a cada ancla

`tau = round(distancia(gateway, ancla) / 4.690 mm)`, la misma conversión que
`sync_link.c`. El gateway es un nodo ubicado del registro.

- El edge instala el enlace **directamente en cada ancla por MQTT** con un
  comando firmado nuevo, `link_set`: `a = tau`, `b = generación`, y un campo
  `p = EUI16` del padre. Esto extiende `cmd_proto` con un campo hexadecimal
  opcional y un vector de prueba nuevo en C y Python.
- Cada ancla tiene su propio challenge, así que las instalaciones van en
  paralelo y no en serie por el gateway.
- La generación es **por ancla**: la de una ancla nueva no toca la de las
  existentes. El ancla queda `active` en el registro solo cuando su ack confirma
  el tau y la generación.
- Hoy `tau_set` borra el registro del enlace ("un tau a mano ya no es de
  survey"). `link_set` es lo contrario: instala el registro completo, con el
  mismo orden de persistencia que `SETTAU` (tau, modo, enlace).
- Las coordenadas en el ancla **no se instalan**. En una celda blink el ancla no
  las usa; solo el responder WAVE (TWR, ids < 4) las necesita, y para esas el
  `SETPOS` del gateway sigue existiendo.

### D8. TDoA en el edge con las mejores K

- **El grupo se libera** al vencer la ventana (150 ms) o cuando junta
  `K_MAX = 8` observaciones, no "todas las anclas del registro" (§1.6).
- **Selección:** de las observaciones del grupo, solo de anclas `active`, se
  descartan las de σ > `SIGMA_MAX` (ya existe), se ordenan por σ ascendente, y
  se toman las primeras **K = 6** que pasen un chequeo de geometría: el
  determinante de la normal 2×2 sobre el umbral, que ya calcula el solver. Si
  con K no pasa, se prueba con más.
- **K se ajusta en el banco.** 6 es un punto de partida: da 5 ecuaciones contra
  2 incógnitas, con residual significativo, y es barato en Python.
- El "hull" de protección del edge se calcula sobre **las anclas usadas en ese
  solve**, no sobre todo el sitio. Con 254 anclas en una nave grande, la caja
  del sitio entero no protege nada.

### D9. `setup/<zone>` lo publica el edge desde el registro

- El edge publica el retained `uwb/anchor/setup/<zone>` en el broker local y en
  el de la nube, con el **mismo schema congelado**: una entrada por ancla
  `active`, de nombre `ANC-LOBBY-NNN`, y el origen del gauge con `apos ref`.
- El gateway deja de publicarlo **solo cuando el registro del edge existe**,
  mediante una bandera persistida `apos edge on` en el gateway. Así una celda
  chica sin edge registrado sigue como hoy, y nunca hay dos publicadores
  peleando el mismo retained.

### D10. Interfaz de operador en el edge

Una herramienta CLI, `edge/tools/survey.py`, igual que `nodecmd.py`:

```
survey status                    registro: cuántas en cada estado, quién falta y por qué
survey assign [--all]            pide al gateway `apos assign` (unassigned_only por defecto)
survey neighbors                 lanza las rondas de sondeo y espera las tablas
survey gauge <id|eui> ...        solo en la primera puesta en marcha
survey fix <eui> x y z           punto de control medido con cinta
survey measure                   planea y ejecuta los pares de las anclas nuevas
survey place                     ubica y muestra el reporte por ancla (no instala nada)
survey apply                     instala enlaces y marca `active` (pide confirmación)
survey forget <eui>              baja del registro, más `apos forget` en el gateway
```

`survey place` y `survey apply` están separados por la misma razón que
`apos run` y `apos apply`: el operador ve el reporte antes de que algo cambie.

---

## 4. Lo que cambia en el firmware

| área | cambio |
|---|---|
| `uwb_config` | `UWB_MAX_ANCHORS` = 254, `UWB_TWR_MAX_ANCHORS` = 4; responders y shell |
| `apos_addrmap` | `APOS_ADDRMAP_MAX` = 254. `apos_addr_plan()` con bitmap de direcciones tomadas, para pasar de O(n²·254) a O(n·254). A 254 anclas el algoritmo actual cuesta decenas de ms, fuera del presupuesto de un step |
| `apos_gw` assign | Lista de pares propia de 254 entradas (EUI + dirección, ~2.5 KB), separada de `apos_table`. Enumeración por cubetas y `unassigned_only`. El plan y la escritura del mapa siguen bajo `APOS_GW_SOLVE_BUDGET_UUS` |
| `apos_frame` | `SURVEY_BEGIN` crece (D2); subtipos nuevos `PROBE_ROUND` 0x0C y `PROBE` 0x0D |
| `apos_node` | Responder por cubeta, emitir el `PROBE` con sus gates, tabla de vecinos y publicarla por MQTT |
| `net_uplink` | Gateway: suscripción a `job/<zone>/gw`, verificación HMAC con id monótono y publicación en `meas/<zone>`. Ancla: publicación en `neigh/<zone>/<EUI>`. Bandera `apos edge` para el `setup` |
| `node_cmd`/`cmd_proto` | Comando `link_set` (a, b, p) |
| `apos_gw` survey | Cola de trabajos (range, probe, local) ejecutada con la maquinaria existente; sin solve |

**Qué no cambia en el firmware:** el beacon, el CCP, el blink, el tag, el
`tdoa_gw` del gateway (queda como diagnóstico) y el survey chico con
`apos run`/`apply` (≤ 8 nodos).

**Memoria nueva estimada en el gateway:** unos 6 KB (mapa de 2.5 KB, lista de
assign de 2.5 KB, cola de trabajos y estado). Se mide, no se acepta por
aritmética; ver el hard-won fact del `gw_core_ctx`. En el ancla, ~1 KB por la
tabla de vecinos.

## 5. Lo que cambia en el edge

| área | cambio |
|---|---|
| registro | `registry.py`: modelo, escritura atómica, versión, alimentado por status, meas y neigh |
| survey | `survey_plan.py` (pares candidatos), `survey_solve.py` (forma cerrada del gauge, crecimiento con Gauss-Newton, rigidez por ancla, puntos de control), con port verificado contra `apos_geom.c` |
| enlaces | Instalación de `link_set` por ancla, con confirmación por ack |
| TDoA | Liberación por ventana o `K_MAX`, selección de las mejores K entre las `active`, hull local |
| setup | Publicar desde el registro (D9) |
| CLI | `tools/survey.py` (D10) |

---

## 6. Topics MQTT nuevos (internos; la plataforma no los consume)

| topic | quién → quién | contenido |
|---|---|---|
| `uwb/anchor/neigh/<zone>/<EUI16>` | ancla → edge | tabla de vecinos de la última ronda (retained) |
| `uwb/anchor/job/<zone>/gw` | edge → gateway | trabajos firmados (D4) |
| `uwb/anchor/meas/<zone>` | gateway → edge | un resultado de ranging por mensaje |

El status y los comandos por EUI son los del paso 1. `setup/<zone>` no cambia de
topic ni de schema.

---

## 7. Flujos de operador

**Primera puesta en marcha de un sitio:**
1. Encender gateway y anclas.
2. `survey assign --all`.
3. `survey neighbors`.
4. `survey gauge ...` y, opcionalmente, `survey fix ...` en algunas anclas.
5. `survey measure`.
6. `survey place` y revisar el reporte.
7. `survey apply`.

**Agregar anclas a un sitio en producción:**
1. Montar y encender las nuevas.
2. `survey assign`, que encuentra solo las UNASSIGNED.
3. `survey neighbors`.
4. `survey measure`, que mide solo los pares de las nuevas.
5. `survey place`.
6. `survey apply`, que instala solo las nuevas.

Las existentes no se mueven ni dejan de producir posiciones en ningún momento.

**Reemplazar un ancla:** `survey forget <eui-vieja>` y después el flujo de
agregar con la nueva. Recibe la dirección liberada y su propia ubicación.

---

## 8. Precondiciones, riesgos y gates

**Precondiciones de este paso (si no se cumplen, el caso es del paso 3):**

- **P1. Cada ancla oye al gateway y el gateway la oye a ella:** `SURVEY_BEGIN`,
  `RANGE_CMD`, `RANGE_RSP` y el CCP. En la prueba de campo del 2026-10-12, el
  enlace entre anclas superó los 100 m.
- **P2. Los tags oyen el beacon del gateway** en toda la zona de la celda. **No
  está medido:** de esa prueba solo hay ~40 m tag → ancla. Es lo primero que hay
  que medir, porque fija el tamaño real de la celda.

**Riesgos conocidos:**

- **R1. Deriva del survey en cadena.** Cada ancla ubicada contra otras ya
  ubicadas hereda su error. En un sitio de cientos de metros eso se acumula.
  Mitigación: puntos de control con cinta (D5), un gauge en el centro y no en
  una esquina, y el residual por ancla. **No hay ajuste global** (bundle
  adjustment) en este paso; si la deriva medida lo exige, es una mejora
  posterior del edge que no toca firmware.
- **R2. Sincronía a distancia.** El jitter medido es de 18-21 DTU a > 3 m a
  PLEN_256. A 50-100 m del gateway no está medido, y TDoA depende de él
  directamente. Gate H-S abajo.
- **R3. Carga de MQTT.** Las observaciones crecen con las anclas que oyen cada
  blink, no con las 254. Un sitio denso puede tener 10-15 por blink. El edge ya
  tiene `MAX_GROUPS` = 256; medir la ingesta.
- **R4. Duración de la puesta en marcha desde cero** (~17 min de ranging,
  §D4). Es aceptable una vez; si no, bajar D.

**Gates de hardware (todos NO EJECUTADO):**

| gate | qué | criterio |
|---|---|---|
| H-B | Alcance del beacon gateway → tag (P2) | Distancia a la que el tag sigue uniéndose y blinkeando sin perder slots. Fija el radio de celda |
| H-S | `sync stats` en anclas a 10, 30, 60 y 100 m del gateway | `jitter_est` por distancia; decide hasta dónde vale una celda para el objetivo de ~45 cm |
| H-E | Enumeración por cubetas con ≥ 8 anclas y `n_buckets` forzado a 2 | Todas enumeradas en una pasada |
| H-P | Rondas de sondeo con ≥ 8 anclas | Tablas de vecinos publicadas y coherentes; ningún `TXFRS never completed` en el gateway |
| H-I | Agregar 2 anclas a una celda ya activa | Las existentes siguen publicando posiciones todo el tiempo; las nuevas quedan `active` con residual por ancla < 50 mm |
| H-T | TDoA con las mejores K y > 6 anclas oyendo | Dispersión estacionaria igual o mejor que con 4 anclas en el mismo punto |
| H-M | Memoria | `CONFIG_THREAD_ANALYZER` en el gateway durante assign y trabajos de survey; margen de stack medido |

---

## 9. Tareas (orden sugerido)

1. Ids a 254 y `UWB_TWR_MAX_ANCHORS` (firmware, host tests).
2. `apos_addrmap` a 254 con el plan optimizado (host tests y mutación).
3. Enumeración por cubetas y `unassigned_only` (frame, nodo, gateway; tests de
   `apos_gw` con 40+ anclas falsas).
4. Rondas de sondeo y tablas de vecinos por MQTT (frame, nodo, uplink).
5. Canal de trabajos firmado y publicación de mediciones (gateway; vector HMAC
   compartido con el edge).
6. `link_set` (`cmd_proto`, `node_cmd`, edge).
7. Registro del edge.
8. Planificador y solver incremental del edge, con port verificado contra
   `apos_geom.c`.
9. TDoA con las mejores K y hull local en el edge.
10. `setup` desde el registro y la bandera `apos edge`.
11. `tools/survey.py` y el procedimiento de operador.
12. Gates H-B y H-S **primero** en el banco, porque pueden cambiar el tamaño de
    celda antes de invertir en el resto; después H-E, H-P, H-I, H-T y H-M.

Las tareas 1-2 (firmware) y 7 (edge) no comparten archivos y pueden ir en
paralelo. La 8 depende de la 7, y la 3-5 de la 2.

---

## 10. Abierto

- **Tamaño real de celda:** lo deciden H-B y H-S. Si salen cortos, el paso 3
  (multihop o multi-celda) se adelanta.
- **D (vecinos por ancla) y K (anclas por solve):** valores iniciales 8 y 6, a
  ajustar en banco.
- **Ajuste global del survey (R1):** no entra en este paso.
- **Autoridad del mapa de direcciones en el edge:** no entra en este paso; es
  del paso 3, cuando haya varios gateways.
- **Los 32 commits sin publicar de `master`:** este diseño toma ideas de ellos
  (rigidez y pares candidatos) pero no su código tal cual. Conviene cerrar esa
  línea (archivar o mergear lo útil) antes de implementar, para que nadie
  mergee después encima.
