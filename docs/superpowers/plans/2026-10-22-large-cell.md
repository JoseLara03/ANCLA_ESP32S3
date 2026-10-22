# Paso 2: celda grande (hasta 254 anclas) — plan de implementación

Fecha: 2026-10-22.
Estado: **plan, sin implementar.**
Spec: `docs/superpowers/specs/2026-10-22-large-cell-design.md`. D1–D10, P1–P2,
R1–R4 y los gates H-* son de ese documento. Si una tarea descubre que una
decisión del spec es inviable, **se detiene y se reporta**; no se improvisa.

Los gates de banco H-B y H-S los corre el operador **en paralelo** con la
implementación (decisión del 2026-10-22). Si alguno sale corto, se revisa el
tamaño de celda antes de cerrar las tareas de survey (T7–T11).

## 0. Reglas para todas las tareas

- **Rama:** `feat/large-cell`, cortada de `feat/anchor-eui-identity` (el paso 1
  todavía no está mergeado). Si el paso 1 se mergea antes, cortar de esa base.
- **Línea "network-scaling-v3" de `master`:** resuelto el 2026-10-22. Sus 32
  commits quedaron archivados en `archive/network-scaling-v3` (`535d015`, en
  `origin`) y el `master` local volvió a `origin/master` (`c2a1fb0`). De esa
  línea **no se hace cherry-pick**: lo útil se reimplementa en T0, T5 y T8,
  como se indica en cada tarea, porque la base divergió el 2026-08-24 y los
  commits chocan. Lo diferido (reusar la dirección del tag por EUI `36bf9b0`,
  `gw seed` `4cb4d61`/`25a326f`, POS que renueva el lease `a5a48c6`) es de otro
  frente y queda en el archivo.
- **Cada tarea de firmware cierra con:**
  - las tres imágenes compilando (producción, `cal.conf`, `poll.conf`);
  - el delta de `dram0_0_seg` anotado en el commit;
  - la suite host completa;
  - las líneas de build nuevas agregadas a `CLAUDE.md` §Host tests **en la
    misma tarea**.
- **Cada tarea del edge cierra con** `python -m unittest discover -s tests`
  (desde `edge/`).
- **Lazo `K_PRIO_COOP(0)`:** un frame por step, toda escritura NVS bajo
  `APOS_GW_SOLVE_BUDGET_UUS`, ningún cálculo O(n²) sin presupuesto, ninguna
  espera sin cota.
- **Lógica nueva del gateway:** se prueba en `tests/apos_gw/` contra el radio y
  las anclas falsas, con mutaciones deliberadas; cada mutación tiene que romper
  al menos un caso.
- **Lógica nueva del edge que porta C:** se verifica contra el C con el método
  de `edge/tools/xcheck.py`.
- **Estimado:** ~35–43 min de agente por tarea, ~8–10 h para T1–T13 (convención
  de `CLAUDE.md` §Reporting progress). Es un orden de magnitud y no dice nada de
  cuándo funciona en hardware.

---

## Fase A — firmware base (sin cambios de protocolo)

### T0. Diagnósticos rescatados del archivo (opcional, independiente)

Reimplementación de dos commits de `archive/network-scaling-v3`, chicos y
útiles ya, antes de que el paso 2 cargue más el lazo del gateway:

- **`45a6449`:** `LOG_WRN` cuando un JOIN de tipo y longitud correctos no
  parsea; un catch-all en `LOG_DBG` (tipo, longitud y origen) para los frames
  que `dispatch()` no reclama; y `uwb_gateway.c` registrado con
  `ANCLA_LOG_LEVEL` en lugar de `LOG_LEVEL_INF`, para que la imagen de debug
  pueda verlos.
- **`e9b06aa`, parte del solve:** medir `do_solve()` con `k_uptime_get()` y
  agregar `solve_ms` a la línea `apos_solve`, más un aviso **antes** de
  resolver por encima de `APOS_GW_SOLVE_TIMED_NODES` (8). La parte de
  `SURVEY_BEGIN` va en T5.

**Cierre:** compilan las tres imágenes; en banco, `apos run` muestra
`solve_ms`.

### T1. Ids hasta 254 y modo TWR limitado a 4 (spec §2.2)

**Archivos:** `src/uwb_config.{c,h}`, `src/anchor_respond.c`,
`src/anchor_shell.c`, `src/apos_shell.c`, `src/cal_shell.c`, `src/sync_shell.c`,
`src/gw_selftau.c`, `src/pos_sink.c`, `src/apos_gw.h` (comentario),
`tests/uwb_config/`.

- `UWB_MAX_ANCHORS` pasa a **254**, con `BUILD_ASSERT`/`_Static_assert` de que
  `UWB_ANCHOR_ADDR_BASE + UWB_MAX_ANCHORS - 1 < UWB_ANCHOR_ADDR_UNASSIGNED`. Esto
  fija la trampa `0x0001 + 0xFF = 0x0100` en tiempo de compilación.
- Nuevo `UWB_TWR_MAX_ANCHORS 4` y
  `bool uwb_config_twr_capable(const uwb_config_t *)`.
- `anchor_respond_discovery()` y el responder WAVE rechazan con
  `!uwb_config_twr_capable()`, reutilizando `log_unassigned_refusal()`
  generalizado.
- **`disc_resp_delay_uus()` nunca se llama con id ≥ 4.** `BUILD_ASSERT` de que
  `TX_COMPLETE_TIMEOUT_MS` cubre `disc_resp_delay_uus(UWB_TWR_MAX_ANCHORS - 1)`
  (spec de 2026-09-02 §2).
- `anchor id <n>`: acepta 0..253 y avisa con n ≥ 4.
- Shells (`apos gauge`, `cal`, `sync tau measure`): el rango crece solo con la
  constante. Revisar mensajes y anchos de formato.
- `pos_sink`: el gate pasa de `UWB_FRAME_MAX_ANCHORS` a un tope propio para
  fixes armados en el gateway (`POS_MAX_ANCHORS`), con el log una vez por boot
  (spec de 2026-09-02 §3.2).
- **No se toca** `tdoa_collect` ni `POS_MAX_ANCHORS`: el TDoA del gateway queda
  como diagnóstico (spec §2.2). Se documenta en `tdoa_gw.h` que un ancla con
  aid ≥ 4 se cuenta en `no_anchor`.

**Pruebas:** `tests/uwb_config/`:
- `set_id` acepta 0..253 y `0xFF`, y rechaza 254;
- `short_addr(253) == 0x00FE`;
- `twr_capable` en 3, 4 y UNASSIGNED;
- el assert de compilación existe.

**Cierre:** en banco, un ancla con `anchor id 10` no contesta DISCOVERY (log de
rechazo), sí publica blinks con `"a":10`, y `apos assign` la adopta.

### T2. `apos_addrmap` a 254 con plan O(n·254) (spec §4)

**Archivos:** `src/apos_addrmap.{c,h}`, `tests/apos_addrmap/`.

- `APOS_ADDRMAP_MAX` = **254**, desacoplado de `APOS_MAX_NODES`. El blob
  máximo es 2 + 254·10 = 2542 B, lejos de los 192 KB de la partición.
- `apos_addr_plan()`:
  - un bitmap de direcciones reservadas por el mapa y otro de tomadas por la
    salida;
  - la regla 3 (el EUI menor gana) por ordenamiento por (addr, EUI) en vez del
    doble lazo;
  - la regla 4 en orden de EUI, con un ordenamiento único y no un "busca el
    menor" en cada vuelta.
- Las funciones `find_*` siguen lineales (n ≤ 254, sin presupuesto crítico).

**Pruebas:**
- las de hoy siguen pasando;
- caso de 254 peers sin mapa, todos con dirección distinta → adopt;
- 254 UNASSIGNED → direcciones `0x0001..0x00FE` en orden de EUI;
- 255 peers → `-EINVAL`;
- **benchmark host** que registre el tiempo de 254 peers en el log de la
  suite;
- mutaciones: las 4 de hoy más "bitmap sin limpiar entre llamadas".

**Cierre:** el benchmark host queda en un orden de magnitud bajo 1 ms (el
target es más lento, pero el ESP32-S3 a 240 MHz no está 100× por debajo de un PC
en este tipo de código). La medición real en el target es parte de H-M.

---

## Fase B — protocolo nuevo de la celda grande

### T3. Lista de assign de 254 y enumeración por cubetas (D2)

**Archivos:** `src/apos_frame.{c,h}`, `src/apos_node.c`, `src/apos_gw.{c,h}`,
`src/apos_shell.c`, `tests/apos_frame/`, `tests/apos_gw/`.

- **`SURVEY_BEGIN`** pasa a `session, window_s, bucket(1), n_buckets(1),
  flags(1)` (`APOS_SB_UNASSIGNED_ONLY`). La longitud nueva es exacta, así que un
  `SURVEY_BEGIN` viejo deja de parsear. El parser viejo de cada ancla se
  reemplaza; el gateway solo emite el nuevo.
- **Ancla:** contesta solo si la regla de cubeta o la bandera la incluyen. El
  hash es FNV-1a del EUI con una sal **distinta** de la del slot de
  `enum_slot()`, para que cubeta y slot no se correlacionen.
- **Gateway:** `assign` usa su propia lista `struct apos_assign_peer
  { eui[8]; addr; conflict; }` de 254 entradas, estática, separada de
  `apos_table` (que sigue en 8 para el survey chico). `apos_table_needs_assign()`
  tiene su equivalente en la lista.
- **Cubetas:** `n_buckets = max(1, ceil(n_esperadas / 16))`, con `n_esperadas`
  = entradas del mapa + 16 de margen. Las rondas recorren las cubetas; cada
  cubeta lleva sus `APOS_GW_ENUM_ROUNDS`.
- **`apos assign`:** por defecto `unassigned_only` más una pasada de las
  cubetas cuyas anclas del mapa no han aparecido en el status. `apos assign
  all` hace la auditoría completa.
- **`apos run`** (survey chico) sigue con una sola cubeta y su tope de 8.

**Pruebas:**
- `tests/apos_frame/`: round-trip del frame nuevo y rechazo del viejo.
- `tests/apos_gw/` con **60 anclas falsas** que implementan la regla de cubeta y
  el stagger por slot:
  - enumeración completa;
  - `unassigned_only` encuentra solo las nuevas;
  - un assign de 3 placas nuevas sobre 57 existentes cuesta ≤ 2 cubetas;
  - la escritura del mapa sigue bajo presupuesto.
- Mutaciones: ignorar la bandera, la cubeta fuera de rango, la sal compartida
  con el slot.

**Cierre:** H-E en banco (≥ 8 anclas, `n_buckets` forzado a 2 con un comando de
debug).

### T4. Rondas de sondeo y tablas de vecinos (D3)

**Archivos:** `src/apos_frame.{c,h}` (`PROBE_ROUND` 0x0C, `PROBE` 0x0D),
`src/apos_node.{c,h}`, `src/apos_gw.c`, `src/net_uplink.c`, `src/pos_json.{c,h}`
(formato de la tabla), `tests/apos_frame/`, `tests/pos_json/` (o una suite
nueva `tests/neigh_json/`), `tests/apos_gw/`.

- **Frames:**
  - `PROBE_ROUND` = sesión, ronda, `slot_us`, k;
  - `PROBE` = sesión, ronda, EUI.
- **Ancla, al recibir `PROBE_ROUND`** (gates: ventana abierta, sesión, src
  `0x0000`):
  1. calcula su slot `((id + ronda·k) mod 254) · slot_us`;
  2. lo duerme;
  3. pide permiso a `beacon_guard`; si cae sobre el beacon, lo **salta** sin
     reintentar (la ronda siguiente lo mueve);
  4. emite su `PROBE` por la ruta `tx_now()`, una vez.
- **Ancla, al oír un `PROBE`** de una sesión abierta: actualiza su tabla
  `{addr → count, max_rx_level_x10}` (254 × 4 B, estática).
- **Límite de rango:** `slot_us · 254` ≤ el tope del stagger de enumeración,
  para no bloquear el lazo SLAVE más que hoy. Si no cabe, la ronda se reparte
  por cubetas (mismo mecanismo de T3).
- **Publicación:** al cerrar la ventana, o a los N s sin `PROBE` nuevos, el
  ancla publica `uwb/anchor/neigh/<zone>/<EUI16>`, retained:
  `{"s":<sesión>,"n":[[addr,count,dbm10],...]}`, partido en mensajes de ≤ 200 B
  (`"part"`/`"of"`).
- **Gateway:** el comando `apos probe [rounds]` (y luego el trabajo de T5)
  emite las rondas, una por step, con su margen.

**Pruebas:**
- frames (round-trip y longitudes);
- JSON de la tabla (partición, longitud máxima, contra el buffer real);
- en `tests/apos_gw/`, el gateway emite las rondas con el espaciado correcto y
  nunca dos frames por step.
- La lógica del slot es una función pura `apos_probe_slot_us(id, ronda, k,
  slot_us)` con host test: k coprimo con 254, y ningún id cae en el mismo
  desfase en 3 rondas consecutivas.

**Cierre:** H-P en banco.

### T5. Canal de trabajos firmado y publicación de mediciones (D4)

**Archivos:** `src/cmd_proto.{c,h}` (verificación de trabajos), `src/apos_gw.c`
(cola), `src/net_uplink.c`, `src/pos_json.{c,h}` (formato de `meas`),
`edge/tdoa_server/nodecmd.py` (firmar trabajos), `tests/cmd_proto/`,
`tests/apos_gw/`, `edge/tests/`.

- **Formato:**
  `{"job":<u32>,"op":"range|probe|local","x":"<hex>","sig":"<hex>"}`, con la
  firma HMAC-SHA256 sobre `zone|gw|job|op|x`.
- **Anti-replay:** `job` > el último aceptado, persistido (`apos/jobseq`). El
  persist va bajo presupuesto y **por lote**, no por trabajo: se escribe cada 16
  trabajos o al vaciarse la cola. Un reinicio en medio puede re-aceptar a lo más
  16; eso se documenta, porque solo repite mediciones idempotentes.
- **`x`:**
  - para `range`, pares `ffff tttt` en hex, ≤ 24 por trabajo;
  - para `local`, direcciones;
  - para `probe`, el número de rondas.
- **Cola de 64 trabajos.** Si está llena, se rechaza con un ack
  `{"job":n,"ok":0,"err":"full"}`; el edge reintenta.
- **Ejecución:** cada par usa el camino `RANGE_CMD` de hoy (reintentos, ventana
  de sordera). Cada `local` usa `step_local_range()`.
- **Resultados:** `uwb/anchor/meas/<zone>` con
  `{"job":n,"from":EUI,"to":EUI,"mean":mm,"sd":mm,"n":k}`, o un hueco con
  `"n":0` y la razón.
- **El survey chico no cambia:** `apos run` sigue usando su tabla y su solve. La
  cola de trabajos es otro modo (`APOS_GW_JOBS`) que no persiste geometría.
- **Refresco de ventanas durante la cola** (rescatado de `896cac3`,
  reimplementado): mientras haya trabajos, el gateway re-emite `SURVEY_BEGIN`
  con la misma sesión cada `APOS_GW_WINDOW_REFRESH_MS` (30 s), un frame por step
  y respetando el margen de asentamiento. Sin esto, un ancla que pasa más de
  `APOS_NODE_REFRESH_S` (60 s) sin recibir un `RANGE_CMD` cierra su ventana, y
  sus pares posteriores salen como huecos con el radio aparentemente sano. Con
  una cola de ~17 min eso es el caso normal, no un borde. La cola nunca se
  "salta" pares de a uno por step: si no hay frame que mandar, avanza dentro del
  mismo step.
- **`SURVEY_BEGIN` que no sale** (rescatado de `e9b06aa`): `send_survey_begin()`
  deja de descartar el resultado de `tx_now()`. Un fallo se avisa y nombra la
  consecuencia (ventanas que vencen a mitad de la cola); el refresco se registra
  con su posición en la cola. Sin reintento propio: cada llamador re-emite en su
  propio calendario.

**Pruebas:**
- vector HMAC de trabajo compartido C y Python;
- una cola de más de 60 s simulados re-emite `SURVEY_BEGIN` antes de que venza
  la ventana de cualquier ancla falsa (las anclas falsas modelan el cierre de
  ventana, como en las pruebas de assign);
- replay rechazado y fuera de orden rechazado;
- cola llena;
- en `tests/apos_gw/`, un lote de 24 pares ejecutado contra las anclas falsas
  publica 24 mediciones o huecos y nunca dos frames por step.
- Mutaciones: aceptar `job` repetido, firmar sin `op`, persistir sin
  presupuesto.

**Cierre:** en banco, el edge manda un lote de 6 pares y llegan 6 mediciones
coherentes con `apos run` en las mismas anclas.

### T6. Comando `link_set` (D7)

**Archivos:** `src/cmd_proto.{c,h}` (campo opcional `"p"`, 16 hex),
`src/node_cmd.c`, `tests/cmd_proto/`, `edge/tdoa_server/nodecmd.py`,
`edge/tests/`.

- `link_set a=tau b=gen p=EUI16`: valida rangos, persiste tau → modo → enlace en
  el mismo orden que `handle_settau()`, aplica en vivo con
  `ccp_slave_set_tau()` y hace ack con el tau y la generación que **quedaron**.
- La cadena canónica de la firma agrega `|p` **solo cuando el campo existe**,
  para que las firmas viejas sigan siendo válidas.
- Gateway: `not_supported`, como los demás comandos de ancla.

**Pruebas:** vector cruzado C/Python con `p`; firma vieja sin `p` sigue válida;
`p` mal formado → `bad_arg`.

**Cierre:** en banco, `nodecmd.py <EUI> link_set 300 7 <gwEUI>`, y luego
`sync tau show` en el ancla da link `installed`, gen 7 y tau 300.

---

## Fase C — edge

### T7. Registro del sitio (D1)

**Archivos:** `edge/tdoa_server/registry.py` (nuevo), `service.py`,
`edge/tests/test_registry.py` (nuevo), `docker-compose.yml` (volumen `data/`).

- Modelo de §D1 del spec, versión `1` en el archivo y escritura atómica.
- Alimentado por status (EUI, id, assigned), `neigh`, `meas` y las acciones del
  survey. Estados `seen → assigned → measured → placed → active`; cualquier cosa
  inesperada **retrocede** el estado, nunca lo adelanta.
- `active` requiere `placed` y un ack de `link_set` con la generación del
  registro.
- **Integración con TDoA:** el `Pipeline` recibe las anclas `active`,
  resueltas por EUI. Se elimina la dependencia de `parse_setup` cuando hay
  registro; sin registro, queda el comportamiento de hoy.

**Pruebas:**
- transiciones válidas e inválidas;
- persistencia (cargar, guardar y recargar idéntico);
- versión desconocida → no se sobrescribe;
- un ancla que vuelve UNASSIGNED retrocede a `seen`.

### T8. Planificador y solver incremental (D4, D5, D6)

**Archivos:** `edge/tdoa_server/survey_plan.py`, `survey_solve.py` (nuevos),
`edge/tools/xcheck_survey.py` (nuevo, contra `apos_geom.c`), `edge/tests/`.

- **Planificador:**
  - pares candidatos por ancla nueva, ≤ D = 8 vecinos más fuertes, priorizando
    las ya ubicadas;
  - siempre lo suficiente para la rigidez por ancla;
  - genera los lotes de trabajos de ≤ 24 pares.
  - **Reglas rescatadas de `b87a767`** (reimplementadas sobre las tablas de
    sondeo en lugar del bitmap de 32 bits):
    - la evidencia de un solo sentido basta para que un par sea candidato (la
      ronda no garantiza que cada ancla oiga a todas las que la oyen);
    - un ancla con tabla **vacía** reportó no oír a nadie, lo cual es falta de
      evidencia y no aislamiento, así que conserva como candidatos todos sus
      pares con las anclas que sí la oyeron;
    - los huecos se cuentan solo sobre pares candidatos, para que "se pidió y
      falló" no quede enterrado bajo "nunca se pidió".
- **Rigidez global del sitio, rescatada de `7563263`** (portada a Python, como
  reporte y no como bloqueo; el bloqueo es la rigidez por ancla):
  - conexión por union-find, grado mínimo d y conteo de aristas contra
    2N−3 / 3N−6, sobre la lista de aristas;
  - pares repetidos contados **una sola vez**, porque una arista duplicada haría
    pasar por redundante una malla isostática;
  - el reporte dice cuál de las tres condiciones falla, porque el remedio es
    distinto en cada caso;
  - la advertencia del original se conserva literal: las tres condiciones son
    **necesarias, no suficientes**. Dos grupos densos unidos por una sola ancla
    pasan las tres y aun así giran alrededor de ella. La prueba rigurosa (rango
    de la matriz de rigidez) queda fuera, igual que en el original, hasta tener
    una malla real medida.
- **Solver:**
  - forma cerrada del gauge (port de `apos_geom_seed()`);
  - crecimiento con Gauss-Newton de 3 incógnitas y las ubicadas fijas;
  - pasadas Gauss-Seidel solo sobre las nuevas;
  - rigidez por ancla con el mismo criterio de condicionamiento que
    `apos_geom_gauge_conditioning()`;
  - puntos de control fijos;
  - residual por ancla y peor arista.
- **2D y 3D,** con el mismo `dim` que el survey del sitio.

**Pruebas:**
- sintéticas con 50 y 254 anclas en malla con ruido de ±20 mm: todas ubicadas,
  error RMS < 50 mm sin deriva medible a 100 m **con** 3 puntos de control, y la
  deriva sin ellos **registrada** como número (es la evidencia de R1);
- un ancla con 2 vecinas en 2D queda `measured` con la razón;
- las ubicadas no se mueven en un segundo incremento;
- `xcheck_survey.py`: el gauge del port coincide con `apos_geom.c` en 1000 casos
  aleatorios.
- rigidez global: dos grupos unidos por una sola ancla pasan el chequeo (se
  prueba que la limitación documentada existe); una arista duplicada no cuenta
  dos veces; un ancla de tabla vacía conserva sus pares candidatos.

### T9. TDoA con las mejores K y hull local (D8)

**Archivos:** `edge/tdoa_server/tdoa.py`, `service.py`, `edge/tests/`.

- `Collector`: libera al vencer la ventana o al llegar a `K_MAX = 8`, en lugar
  de `expected = len(anchors)`.
- Selección: anclas `active`, filtro por σ, orden por σ, tomar K = 6 y ampliar
  si el determinante de la normal no pasa el umbral.
- El hull se calcula con las anclas usadas.
- Variables de entorno `TDOA_K` y `TDOA_K_MAX`.

**Pruebas:**
- grupo con 12 observaciones → se usan 6, las de menor σ;
- un grupo geométricamente degenerado con las 6 primeras amplía a 7 y resuelve;
- el hull local rechaza un salto que el hull global habría dejado pasar;
- con 4 anclas, el comportamiento es idéntico al de hoy (regresión).

### T10. `setup` desde el registro y bandera `apos edge` (D9)

**Archivos:** `edge/tdoa_server/service.py`, `src/net_uplink.c`,
`src/apos_store.{c,h}` (bandera persistida), `src/apos_shell.c`,
`edge/tests/`, `tests/pos_json/`.

- El edge arma el JSON de setup con el **mismo schema** que `pos_json_anchors()`.
  Un test lo compara campo por campo contra la salida del C para las mismas
  coordenadas.
- El gateway, con `apos edge on`, deja de publicar `setup/<zone>`. Con `off`, o
  sin la bandera, sigue igual que hoy.

**Pruebas:** paridad de schema C/Python y la bandera en las dos posiciones.

### T11. CLI `tools/survey.py` y procedimiento (D10)

**Archivos:** `edge/tools/survey.py`, `docs/large-cell-commissioning.md`
(nuevo), `edge/README.md`.

- Los subcomandos de §D10. Cada uno escribe al registro e imprime lo que hizo.
- `apply` pide confirmación, instala `link_set` en paralelo y espera los acks
  con un plazo.
- Procedimiento de operador: primera puesta en marcha, agregar anclas,
  reemplazar una ancla, y lectura del reporte de `place`.

**Pruebas:** la CLI contra un broker falso en memoria: cada subcomando produce
los mensajes esperados.

---

## Fase D — cierre

### T12. Documentación

- `CLAUDE.md`: sección de estado, consola (`apos assign all`, `apos probe`,
  `apos edge`), Layout de los módulos nuevos, hard-won facts que hayan salido
  en la implementación y host tests.
- El spec cambia su estado a "implementado, NO corrido en hardware".

### T13. Gates de hardware

H-E, H-P, H-I, H-T y H-M en ese orden. H-B y H-S los corre el operador en
paralelo desde el inicio. Cada gate queda como `NO EJECUTADO` / `PASA` / `FALLA`,
con su número, en el spec.

---

## Dependencias

```
T0 (independiente, en cualquier momento antes de T5)
T1 ─┬─ T2 ─ T3 ─ T4 ─ T5 ─┐
    └─ T6 ─────────────────┤
T7 ─ T8 ──────────────────┼─ T11 ─ T12 ─ T13
T7 ─ T9 ──────────────────┤
T7 + T1 ─ T10 ─────────────┘
```

- T1, T6 y T7 no comparten archivos y pueden empezar en paralelo.
- T8 y T9 dependen de T7 y son independientes entre sí.
- T3, T4 y T5 tocan `apos_frame`/`apos_gw`/`apos_node`, así que van en serie.
- T10 toca `net_uplink.c`, igual que T4 y T5: coordinar o serializar.

## Riesgos de implementación (además de los del spec)

- **Stack del gateway:** la lista de assign, la cola de trabajos y los nuevos
  JSON van en estático, nunca como automáticos (ver el hard-won fact del
  `gw_core_ctx`). H-M lo mide.
- **Bloqueo del lazo SLAVE** en las rondas de sondeo: el slot más largo de una
  ronda no puede exceder el stagger de enumeración de hoy (756 ms) sin
  repartirse por cubetas. T4 lo fija con una aserción.
- **Edge sin dependencias nuevas:** el solver es Python puro. Si T8 demuestra
  que hace falta numpy, se para y se reporta antes de agregarlo.
