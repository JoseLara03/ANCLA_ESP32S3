# Identidad de ancla por EUI y dirección corta asignada por el gateway — diseño

**Estado (2026-10-21): IMPLEMENTADO en `feat/anchor-eui-identity`, probado con
pruebas host y compilado (producción, cal, poll). NO corrido en hardware: los
gates H1-H8 de §4.2 siguen `NO EJECUTADO`.** Registro de lo construido en el
plan (`docs/superpowers/plans/2026-10-21-anchor-eui-identity.md`, §"Estado de
implementación").

**Contexto:** producción en masa. Hoy cada ancla necesita que alguien la
conecte al shell y le teclee `anchor id <0..3>`, y dos anclas con el mismo id
no se detectan hasta el survey (`apos_gw.c:481`, `-EADDRINUSE`, que aborta). La
comparación con sistemas comerciales (Pozyx, Sewio, Zebra) dejó el patrón: la
identidad viene de fábrica, un servidor central es la autoridad, y en TDoA el
ancla casi no necesita un id corto en el aire porque sólo recibe.

**Objetivo:** que un ancla recién salida de fábrica se integre a una celda sin
tocar su consola. El instalador la monta y la enciende; el operador corre
**un** comando en el gateway (`apos assign`).

**Relación con otros documentos.** Éste es el "paso 1". El "paso 2", subir el
límite de 4 anclas en modo BLINK, ya está diseñado en
`docs/superpowers/specs/2026-09-02-blink-anchor-scale-design.md` y **no se
re-litiga aquí**. Este documento no toca `UWB_MAX_ANCHORS`, `POS_MAX_ANCHORS`,
`APOS_MAX_NODES` ni ningún cap de capacidad; sólo cambia **quién** decide el id
y **qué** identifica a un ancla fuera del aire. Los dos documentos componen: el
asignador de aquí reparte direcciones dentro del cap que fije el paso 2.

---

## 0. Lo que este documento NO reabre

- **SS-TWR entre anclas para el survey** (`2026-08-14-...-design.md` §4).
- **Una celda es TWR o BLINK, nunca ambas** (`2026-08-30-...-design.md`).
- **El contrato de payload de posición** `{"Tid","x","y","z"}` y el schema del
  topic retained de anclas (`uwb/anchor/setup/<zone>`). Ninguno gana campos.
- **Cero cambios en `tag_testting`** y cero cambios en frames que el tag parsea.
  Lo único nuevo en el aire son dos subtipos APOS, tráfico gateway↔ancla.

---

## 1. Hechos verificados en código (2026-10-21)

1. **El EUI ya existe y ya es la llave del survey.** `hwinfo_get_device_id()`
   se llama en tres sitios por separado: `apos_node.c:95`, `apos_gw.c:185`,
   `ccp_master.c:150`. `struct apos_table` (`apos_table.h:45`) y
   `struct apos_survey_node` (`apos_store.h:28`) ya guardan `eui[8]` junto a
   `short_addr`. `apos_links` ya busca por EUI.
2. **En el ESP32-S3 el "EUI-64" es la MAC de 6 bytes.**
   `zephyr/drivers/hwinfo/hwinfo_esp32.c:42-52` copia 6 bytes de eFuse;
   `apos_node.c:93` rellena los bytes 6-7 con cero. Los bytes 0-2 son el OUI de
   Espressif, constante. Consecuencia útil: **el EUI que imprime el firmware es
   la MAC de la etiqueta del módulo seguida de `0000`**, y un instalador puede
   leerlo sin cable.
3. **`anchor_id` vale 0 por defecto y sólo se persiste si alguien lo fija**
   (`uwb_config.c:34`, `uwb_store.c:274`). Una placa nunca configurada y una
   configurada como 0 son indistinguibles hoy.
4. **La dirección corta es `0x0001 + anchor_id`** (`uwb_config.c:209`); el pool
   de tags empieza en `GW_TAG_ADDR_BASE` = `0x0100` (`gw_core.h:79`). **Trampa:
   `UWB_ANCHOR_ADDR_BASE + 0xFF` es `0x0100`, la primera dirección de tag.** Un
   id centinela `0xFF` NO puede pasar por esa suma.
5. **El MQTT client id sale de la dirección corta**: `"uwb-gw-%04X"`
   (`net_uplink.c:918`), en un buffer de 16 bytes. Dos anclas con el mismo
   `anchor id` presentan el mismo client id, y MQTT 3.1.1 §3.1.4 obliga al
   broker a desconectar la sesión vieja cuando llega la nueva: **las dos se
   tiran mutuamente en un ciclo**, hoy, sin que nada lo reporte. Además todo
   gateway se llama `uwb-gw-0000`: dos gateways en un mismo broker tienen el
   mismo problema aunque sus zonas difieran.
6. **Status, comandos y su firma dependen del id.**
   `uwb/anchor/status/<zone>/<id>` es retained (`net_uplink.c:657`), los topics
   cmd/ack usan `<id>` (`:665`), y `node_cmd.c:157` firma contra el string del
   id (`canonical = "<zone>|<node>|..."`, `edge/tdoa_server/nodecmd.py`). Con id
   duplicado, dos anclas se pisan el mismo retained y **ambas aceptan el mismo
   comando firmado**.
7. **La observación BLINK identifica al ancla sólo por `"a"`**, un `uint8`
   (`pos_json.h:65`, `blink_rx.c:98`). El gateway la ubica con
   `UWB_ANCHOR_ADDR_BASE + anchor_id` contra `node[].short_addr` del survey
   (`tdoa_gw.c:203-221`). El edge deriva el id **del nombre**
   `"ANC-LOBBY-00N"` del topic de setup (`edge/tdoa_server/service.py:64-75`),
   lo cual es frágil por construcción.
8. **Peor caso medido de `pos_json_blink()`: 84 B** contra
   `POS_JSON_BLINK_MAX_LEN` = 128 (`pos_json.h:126-132`).
9. **`APOS_ENUM_SLOTS` ya es 64** (`apos_node.h:84`), con 3 rondas y slot
   re-sorteado por ronda vía hash del EUI con sal (`apos_node.c:235-250`). La
   enumeración ya distingue anclas por EUI aunque compartan dirección.

---

## 2. Decisiones

### D1. La identidad es el EUI; la dirección corta es un handle de radio

| contexto | qué identifica al ancla |
|---|---|
| survey guardado, links, mapa de direcciones | EUI (ya es así) |
| observación BLINK por MQTT | EUI (`"u"`), con `"a"` como respaldo |
| topics status / cmd / ack, firma de comandos | EUI |
| MQTT client id | EUI |
| frames UWB (APOS, TWR, CCP) | dirección corta |
| consola humana, `apos gauge`, logs | id corto (el que el gateway asignó) |

**Precisión sobre "sólo las anclas que transmiten":** en este sistema toda
ancla transmite por lo menos durante el survey (`ENUM_RSP`, ranging,
`RANGE_RSP`, ACKs), así que en la práctica **toda ancla termina con dirección**.
Lo que cambia es su papel: deja de ser identidad y queda como un handle de
radio. Por eso basta con que sea única **dentro de la celda y mientras
transmite**, y por eso un error en ella ya no puede atribuir una observación al
ancla equivocada.

**Formato canónico del EUI fuera del aire:** 16 dígitos hex en MAYÚSCULAS, sin
separadores, los 8 bytes en orden (`"7CDFA1B2C3D40000"`). Es el mismo formato
que ya imprimen `apos_node.c:105` y `apos_gw.c:502`. Nunca se trunca a la MAC
de 12 dígitos: si un día el hwinfo entrega 8 bytes reales, el formato no
cambia.

### D2. Un solo módulo dueño del EUI: `src/node_eui.{c,h}`

- `node_eui_init()` se llama una vez en `main()`, antes de `net_uplink` y del
  dispatch de modo. Lee hwinfo, rellena con cero y formatea el hex una vez.
- `const uint8_t *node_eui(void)`, `const char *node_eui_hex(void)` y
  `bool node_eui_valid(void)`.
- Sustituye las tres lecturas de §1.1. **Sin cambio de comportamiento:** es la
  primera tarea precisamente porque se puede verificar aislada.
- **Si hwinfo falla, o el EUI sale todo en ceros:** `node_eui_valid()` es
  false, se registra `LOG_ERR` una vez y el gateway rechaza ese EUI en
  `apos assign`. Es el mismo agujero del EUI todo-cero que `CLAUDE.md`
  documenta para los tags. Aquí no se tapa, pero se vuelve visible.

### D3. Estado UNASSIGNED

- `#define UWB_ANCHOR_ID_UNASSIGNED 0xFFu` y
  `#define UWB_ANCHOR_ADDR_UNASSIGNED 0x00FFu` en `uwb_config.h`.
- **El default de fábrica pasa de 0 a UNASSIGNED.** Lo mismo `anchor reset`.
- `uwb_config_short_addr()` devuelve `UWB_ANCHOR_ADDR_UNASSIGNED` de forma
  **explícita** para ese id, nunca `BASE + id` (trampa de §1.4). Con esto el
  rango de anclas asignables queda en `0x0001..0x00FE`, holgado frente a
  cualquier cap futuro.
- `uwb_store.c` acepta el valor persistido `0xFF`, que hoy se rechazaría como
  "stored id invalid".
- **Qué hace un ancla UNASSIGNED**, cada punto en un sitio que ya rehúsa cosas
  hoy:
  - TWR: `anchor_respond_discovery()` y el responder WAVE **rehúsan**, con un
    log una vez por boot. Mismo criterio que la negativa por falta de posición:
    una negativa registrada se diagnostica, una respuesta con dirección
    compartida no.
  - APOS: contesta `SURVEY_BEGIN` con `ENUM_RSP` desde `0x00FF` (su EUI va en
    el payload) y acepta `SETADDR` (D5). **Rehúsa `RANGE_CMD`**: rangear con una
    dirección compartida mezclaría respuestas de varias anclas.
  - BLINK: sigue recibiendo y publicando, con `"u"` y `"a":255`. Un gateway
    viejo descarta `a=255` en la cota de `anchor_xyz()` (lo cuenta como
    `no_anchor`, no falla). Uno nuevo busca por EUI y tampoco la encuentra en
    el survey (ningún ancla sin asignar puede estar surveyada).
  - CCP esclavo: sin cambio, sólo recibe.
- **Migración de la flota actual:** una placa con `anchor/id` persistido
  conserva ese id. Una que nunca lo fijó (estaba en 0 por defecto) arranca
  UNASSIGNED después del reflash. `apos assign` la recupera, normalmente con el
  mismo id 0 si está libre (D6, regla 2). Esto se dice en el procedimiento,
  porque una placa así deja de contestar TWR hasta ese paso.

### D4. El gateway guarda un mapa EUI → dirección: `src/apos_addrmap.{c,h}`

- C puro y host-testeado. Codec versionado y con offsets fijos, mismo patrón y
  mismas reglas que `apos_links.c`: una versión desconocida, una longitud
  inconsistente, bytes de cola o un EUI todo cero **se rechazan y no se
  borran**.
- Formato: `ver(1) n(1)` y luego `n × (eui[8], addr_le[2])`. Clave de settings
  `apos/addrmap`.
- Capacidad `APOS_ADDRMAP_MAX = APOS_MAX_NODES` (8). Las entradas de anclas que
  no aparecen en una enumeración **se conservan**: es una reserva permanente,
  como una reserva DHCP. Sólo `apos forget <eui>` libera una dirección.
- Por qué permanente: un ancla apagada el día del `assign` no debe perder su id,
  ni su dirección pasar a otra placa mientras siga instalada. Es la misma razón
  de la cuarentena del pool de tags, en versión más simple: aquí el reemplazo es
  un acto explícito del operador, no un timeout.
- **La escritura del mapa en flash va budget-gated**, con el mismo gate que el
  persist del survey (`APOS_GW_SOLVE_BUDGET_UUS`). Es una escritura NVS en el
  lazo `K_PRIO_COOP(0)` (ver el hard-won fact sobre XIP). Se escribe **una sola
  vez por `assign`**, al final, no una por ancla.

### D5. Dos subtipos APOS nuevos: `SETADDR` (0x0A) y `SETADDR_ACK` (0x0B)

```
SETADDR      dst=0xFFFF  src=0x0000   payload: session(2) eui(8) addr(2)        len = 11+12 = 23
SETADDR_ACK  dst=0x0000  src=<addr>   payload: session(2) eui(8) addr(2) st(1)  len = 11+13 = 24
```

- **El destino va en broadcast y el EUI viaja en el payload.** Es la única forma
  de direccionar un ancla que todavía no tiene dirección propia, o que comparte
  la suya con otra.
- `APOS_LEN_MAX` no cambia (34 sigue siendo el máximo).
- **Gates en el ancla, todos obligatorios** (los mismos que ya protegen
  `RANGE_CMD`): ventana de survey abierta, `src == 0x0000`, sesión que
  coincide, `eui == node_eui()`, y `addr` dentro de
  `[UWB_ANCHOR_ADDR_BASE, UWB_ANCHOR_ADDR_BASE + UWB_MAX_ANCHORS)`.
- **Efecto:**
  1. Si `addr` es la dirección actual, se manda `ACK ok` y no pasa nada más
     (idempotente).
  2. Si no, se persiste `anchor/id` y se manda el ACK con estado `ok`,
     `not_persisted` o `bad_addr`.
  3. Después de un `ok` con cambio, se reinicia en frío unos 500 ms después de
     que el TXFRS del ACK se confirme.
- El ACK sale **con la dirección vieja**, que el gateway ignora porque empareja
  por EUI.
- **Se aplica reiniciando, no en vivo, a propósito.** La dirección se fotografía
  en muchos sitios al entrar al modo (`uwb_slave_run()`'s `cfg_snapshot`,
  `blink_rx.c:34`, `ccp_slave`'s `own_short_addr`, el filtro de trama). Un
  reinicio es lo que `anchor id` ya exige hoy. Aplicarla en vivo obligaría a
  auditar cada snapshot, y uno olvidado sería el bug de `sync_tau_mode` de
  2026-09-25 otra vez.
- **Un ancla con firmware viejo** descarta el subtipo y nunca manda ACK. El
  gateway la reporta por EUI, mismo precedente que `SETTAU`.

### D6. Política de asignación, pura y determinista: `apos_addr_plan()`

Recibe el mapa y los peers enumerados (EUI, dirección actual). Devuelve, por
peer, uno de dos resultados: "adoptar" (sólo agregarla al mapa, sin frame) o
"asignar X". Las reglas, en orden:

1. **El mapa manda.** Un peer que ya está en el mapa recibe su dirección del
   mapa. Si trae otra, se le asigna la del mapa.
2. **Adoptar lo que ya funciona.** Un peer que no está en el mapa, con una
   dirección válida que nadie más reserva en el mapa ni reclama en esta
   enumeración, se **adopta** tal cual. Así una flota numerada a mano pasa por
   `assign` sin un solo reinicio.
3. **Desempate de duplicados** que el mapa no resuelve: se queda la dirección el
   peer de EUI menor (orden lexicográfico de bytes), y el otro entra a la regla
   4.
4. **Lo que queda** (UNASSIGNED o perdedores de la regla 3) recibe la dirección
   libre más baja, que no esté en el mapa ni la tenga otro peer. Se asigna en
   orden de EUI, para que dos corridas con los mismos peers den el mismo
   resultado.
5. **Sin dirección libre:** el `assign` se rehúsa entero y nombra los EUIs que
   no caben. No se asigna a medias.

El EUI del propio gateway nunca entra al mapa (él es `0x0000`).

### D7. Flujo del operador: `apos assign` es explícito y `apos run` lo exige

```
apos assign       enumerar -> planear -> SETADDR a cada "asignar" -> esperar ACKs
                  -> persistir mapa (gated) -> re-enumerar hasta que cada EUI
                  conteste desde su dirección nueva (o vencer el plazo)
                  -> verificar que el plan quedó vacío
apos addr         el mapa como JSON: eui, addr, id, visto en la última enum (sí/no)
apos forget <eui> libera una reserva; rehúsa mientras haya survey corriendo
```

- `assign` es una fase nueva del step machine de `apos_gw` (`APOS_GW_ASSIGN`):
  **un frame por step**, con reintentos como en `APPLY`
  (`APOS_GW_APPLY_RETRIES`).
- **No hay espera fija de reinicio: se re-enumera hasta converger.** Según el
  operador (2026-10-21), el arranque de un ancla varía con el tipo de reset: ~5
  s si el driver WiFi del ESP32 arranca bien al primer intento, y 10-20 s más
  si se queda colgado y el sistema tiene que detectarlo, apagarlo y reintentar.
  Una espera fija sería demasiado corta en el peor caso o un desperdicio en el
  mejor. En su lugar, el gateway re-emite `SURVEY_BEGIN` (cada ronda de enum ya
  lo es, y reabre la ventana de survey que el reinicio cerró) cada
  `APOS_GW_ENUM_SETTLE_MS`. Cada EUI reasignado cuenta como convergido cuando
  contesta `ENUM_RSP` **desde su dirección nueva**. El plazo total es
  `APOS_GW_ASSIGN_CONVERGE_MS` = 30000: los 20 s del peor caso reportado más
  margen, y holgado dentro de `APOS_GW_WINDOW_S` (120 s).
- **Hecho de código que juega a favor, pero aún sin medir:** el lazo UWB **no
  espera al WiFi**. `net_uplink_start()` sólo hace `k_thread_create()`
  (`net_uplink.c:1281`) y `main()` entra a `uwb_slave_run()` justo después
  (`main.c:103-104`), así que un ancla debería contestar la enumeración mucho
  antes de terminar su conexión WiFi. Que el driver colgado no retrase también
  el lazo UWB (por CPU, o por un `SYS_INIT` del driver antes de `main`) **no
  está comprobado**; el gate H8 lo mide.
- El reinicio usa `sys_reboot(SYS_REBOOT_COLD)`, el mismo que el comando
  remoto `reboot` (`net_uplink.c:897`), para no introducir un tercer tipo de
  reset con su propio comportamiento del driver.
- **Si al vencer el plazo el plan no quedó vacío, se aborta.** Nunca se
  re-asigna en bucle; se reporta qué EUIs no convergieron.
- **`apos run` rehúsa si la enumeración encuentra un UNASSIGNED o un
  duplicado**, con el mensaje "corre `apos assign`". **No re-direcciona por su
  cuenta**, porque `apos gauge` se fija con ids **antes** de `run`: si `run`
  cambiara direcciones por debajo, el gauge apuntaría en silencio a otra placa
  física. Es el mismo defecto de "las coordenadas no siguieron al id" que costó
  una sesión en 2026-08. Por eso el orden es assign → gauge → run.
- **Por qué no es automático al arrancar el gateway:** el gauge necesita de
  todos modos un humano (qué ancla es el origen es una decisión física), así que
  hacer automático el assign ahorra un comando y abre el riesgo de arriba. Queda
  como mejora posible para después (§6).

### D8. Cambios MQTT (identidad = EUI)

| qué | hoy | nuevo |
|---|---|---|
| client id | `uwb-gw-%04X` (addr), buffer 16 | `anc-<EUI16>` / `gw-<EUI16>`, buffer 24 |
| status (retained) | `uwb/anchor/status/<zone>/<id>` | `.../<EUI16>` para anclas; `gw` sin cambio |
| cmd / ack | `.../cmd/<zone>/<id>` | `.../cmd/<zone>/<EUI16>` |
| firma de comandos (`node`) | `"<id>"` | `"<EUI16>"` |
| payload de status y last-will | `"id"` | `"eui"`, `"addr"`, `"assigned"`, y `"id"` sólo si está asignado |
| observación BLINK | `{"a":2,...}` | `{"a":2,"u":"<EUI16>",...}` |
| mapa de anclas | `setup/<zone>` (contrato de plataforma) | **sin cambio**, ni topic ni schema |

Notas:

- **Tamaño de la observación:** `,"u":"<16 hex>"` son 22 B, así que el peor caso
  pasa de 84 a ~106 B contra 128. Cabe, pero el margen baja a ~21 B.
  `tests/pos_json_blink/` ya imprime la cifra medida; esa cifra, no esta
  cuenta, es la que se registra.
- **El parser** acepta `"u"` opcional (anclas viejas). Si llega, el gateway
  busca el nodo del survey **por EUI**; si no, cae a `BASE + a` como hoy. El
  `aid` que se le pasa a `tdoa_collect` sigue siendo
  `short_addr(nodo) - BASE`, así que `tdoa_collect` no se toca.
- **No hay topic de mapa nuevo** (decisión del operador, 2026-10-21: la
  plataforma ya define en qué topic va el mapa, `setup/<zone>`). El edge
  resuelve EUI → id con el **status retained** de cada ancla, que ahora lleva
  `"eui"`, `"id"` y `"addr"`; `service.py:127` ya se suscribe a
  `uwb/anchor/status/<zone>/#`, así que no necesita una suscripción nueva. Las
  posiciones las sigue sacando de `setup/<zone>` por nombre, como hoy. El
  parseo por `"ANC-LOBBY-00N"` sigue siendo frágil, pero no empeora, y deja de
  ser la única fuente de verdad: una observación con `"u"` cuyo EUI no tenga
  status conocido se cuenta y se descarta, en vez de atribuirse al id
  equivocado.
- **Longitud de topics:** `uwb/anchor/status/<zone>/` más 16 hex son 41 B con
  la zona actual de 6 caracteres, contra `status_topic[48]`. Se agrega un
  `BUILD_ASSERT` que ate el tamaño del buffer a `sizeof(prefijo) + 16`, porque
  una zona más larga lo truncaría en silencio.
- **Retained huérfanos:** los status viejos `.../status/<zone>/0..3` siguen en
  el broker después del cambio. El procedimiento indica borrarlos (publicar un
  retained vacío en cada uno).

### D9. Consola

- `anchor id <0..3>` se queda como override manual. **El mapa del gateway lo
  respeta sólo si no choca** (regla 2 de D6); si choca, gana el mapa en el
  próximo `assign`.
- `anchor id auto` deja el ancla UNASSIGNED y lo persiste.
- `anchor show` y el banner de boot imprimen `eui` y `assigned`.
- **La imagen de calibración no tiene gateway**, así que sigue exigiendo
  `anchor id` manual. `cal peer` rehúsa si este ancla, o el peer pedido, está
  UNASSIGNED, con un mensaje que lo dice.

---

## 3. Orden de despliegue y rollback

**Cambio directo** de los topics status/cmd/ack (decisión del operador,
2026-10-21): no hay periodo publicando en los dos. Lo único que se mantiene
dual es lo que sale gratis: `"a"` sigue en la observación como respaldo.

1. **Edge y tools primero**, ya con el esquema nuevo: topics por EUI, `--node
   <EUI16>|gw` en `nodecmd.py`, y llaves por EUI en `ANCHORS_JSON` y
   `OFFSETS_DTU`. Mantienen el respaldo a `"a"` para observaciones sin `"u"`.
2. **Gateway después:** parser con `"u"` opcional y `assign`.
3. **Anclas al final.** Mientras una ancla siga en firmware viejo, el edge no
   ve su status (sigue publicando en `.../<id>`) y no le puede mandar
   comandos. Esa es la ventana que el cambio directo acepta.

- Ancla vieja con gateway nuevo: no hay `"u"`, se cae al id, funciona; no
  contesta `SETADDR`, se reporta.
- Ancla nueva con gateway viejo: el campo `"u"` extra se tolera, y 106 B < 128
  B. Los topics de status por EUI le dan igual al gateway.
- **Rollback de un ancla a firmware viejo:** conserva su `anchor/id` persistido.
  Si estaba UNASSIGNED (valor `0xFF`), el firmware viejo lo rechaza al cargar
  (`"stored id invalid"`) y cae a 0. Puede chocar con otra, igual que hoy.

---

## 4. Verificación

### 4.1 Host (sin Zephyr)

- `tests/apos_frame/`: round-trip de `SETADDR`/`SETADDR_ACK`, longitudes, y que
  un `SETADDR` con EUI ajeno se ignore.
- `tests/apos_addrmap/` (nuevo):
  - codec: cada truncamiento, versión desconocida, EUI todo cero, bytes de cola;
  - plan: flota adoptada sin frames; dos duplicados con y sin entrada en el
    mapa; UNASSIGNED múltiples; mapa lleno; determinismo (mismo input permutado
    da el mismo plan).
- `tests/apos_gw/` (el `apos_gw.c` real contra radio, anclas y NVS falsas):
  - assign completo;
  - un ACK perdido;
  - un ancla con firmware viejo (nunca hace ACK);
  - fallo de escritura NVS del mapa;
  - `run` que rehúsa con un UNASSIGNED presente;
  - y que en todos esos casos el plan reportado coincida con lo persistido.
  - Seguir la disciplina de mutación que ya tiene la suite.
- `tests/uwb_config/`: default UNASSIGNED; `short_addr` de UNASSIGNED es
  `0x00FF` y **no** `0x0100`; `set_id` acepta 0..3 y el centinela.
- `tests/pos_json_blink/`: `"u"` presente y ausente, peor caso medido impreso.
- `tests/node_status/`: payload con y sin id.
- `tests/cmd_proto/` y `edge/tests/test_tdoa.py`: un vector nuevo con `node`
  igual a un EUI, el mismo en ambos lados.

### 4.2 Hardware (todos NO EJECUTADO)

| gate | qué | criterio |
|---|---|---|
| H1 | 4 placas con `anchor reset` → `apos assign` | 4 ids distintos, persisten tras `kernel reboot cold` en ambos lados, plan vacío al re-enumerar |
| H2 | 2 placas forzadas al mismo id | la del mapa (o la de EUI menor) conserva el id, la otra se reasigna y reinicia |
| H3 | flota actual numerada a mano | `assign` adopta todo, **cero** reinicios, cero frames `SETADDR` |
| H4 | beacon durante `assign` | ningún `"beacon started but TXFRS never completed"`; heartbeat `gw_sf` en 200.0 ms |
| H5 | MQTT | el log del broker no muestra desconexiones por client id duplicado; un status retained por EUI; el edge resuelve EUI → id desde el status |
| H6 | TDoA intacto | fixes con `"u"` iguales a la línea base con `"a"` (misma posición estática, misma dispersión) |
| H7 | ancla con firmware viejo | reportada por EUI, assign abortado sin persistir la entrada |
| H8 | arranque tras `SETADDR` | tiempo desde el ACK hasta el primer `ENUM_RSP` con la dirección nueva, por separado con el WiFi arrancando limpio y con el driver colgado; debe quedar bajo `APOS_GW_ASSIGN_CONVERGE_MS` con margen. Si el driver colgado también retrasa el lazo UWB, se registra como hard-won fact |

---

## 5. Tareas

1. `node_eui` y sustituir las 3 lecturas de hwinfo. Sin cambio de
   comportamiento; compilan ambas imágenes.
2. Estado UNASSIGNED: `uwb_config`, `uwb_store`, shell, negativas TWR/APOS,
   `cal`. Host test de `uwb_config`.
3. Codec `SETADDR`/`ACK` en `apos_frame`. Host test.
4. `apos_addrmap`: codec + `apos_addr_plan()`. Host test.
5. `apos_table` acepta UNASSIGNED y duplicados como peers marcados; fase
   `ASSIGN` en `apos_gw`; handler en `apos_node`; `apos assign/addr/forget`;
   ampliar `tests/apos_gw/`.
6. Identidad MQTT: client id, topics status/cmd/ack, `node_cmd`, last-will.
   Host test de `node_status`.
7. `"u"` en la observación, el parser y la búsqueda por EUI en `tdoa_gw`. Host
   test de `pos_json_blink`.
8. Edge y tools: `service.py` (EUI → id desde el status, observaciones con
   `"u"`), `nodecmd.py`, `calibrate.py`, `rx_cal.py`, `xcheck.py`; pytest.
9. Docs: `CLAUDE.md` (Console, Layout, el hecho de §1.5 como hard-won fact) y
   un procedimiento de puesta en marcha (encender → `apos assign` → `apos
   addr` → `apos gauge` → `apos run` → `apos apply`).
10. Gates H1-H8.

Las tareas 1-4 son independientes entre sí. La 5 depende de 2-4, la 7 de la 1,
y la 8 se puede empezar en cuanto D8 esté fijo.

---

## 6. Abierto / fuera de alcance

- **Identificar físicamente un ancla:** un `apos identify <id|eui>` que haga
  parpadear el WS2812 de esa placa, para que el instalador sepa cuál es cuál al
  elegir el gauge. Barato (otro subtipo APOS), pero no es necesario para el
  objetivo de este documento.
- **QR con el EUI en la carcasa**, el equivalente a la importación por MAC de
  Sewio. Es de manufactura, no de firmware.
- **`assign` automático en arranque o en ocio del gateway.** Ver D7 para la
  razón de no hacerlo ahora.
- **Varios gateways en un sitio:** el mapa es por gateway. Dos celdas que se
  oigan podrían repartir la misma dirección a anclas distintas. Hoy no hay
  despliegue multi-celda. Cuando lo haya, la autoridad probablemente tenga que
  subir al edge (mapa por sitio y gateways que lo consulten), y eso es otro
  diseño.
- **Aprovisionamiento WiFi:** `net ssid/pass` sigue siendo por shell en cada
  ancla. Para cero contacto de verdad hace falta resolverlo (BLE o softAP), y
  es el mismo problema de producción que éste, pero con otro mecanismo.
- **Preguntas resueltas por el operador (2026-10-21):**
  1. No hay topic de mapa nuevo: `setup/<zone>` es el de la plataforma y no
     cambia; el edge saca EUI → id del status (D8).
  2. Cambio directo de los topics status/cmd/ack (§3).
  3. El arranque varía de ~5 s a 10-20 s según el driver WiFi, así que no hay
     espera fija: se re-enumera hasta converger, con plazo de 30 s (D7), y H8
     mide el tiempo real.
