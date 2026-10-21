# Identidad de ancla por EUI y asignación de dirección desde el gateway — plan

Fecha: 2026-10-21.
Estado: **T0-T10 implementadas en `feat/anchor-eui-identity`, con pruebas host
y compilación de las tres imágenes. T11 (hardware) NO EJECUTADO.** Ver
"Estado de implementación" al final.
Spec: `docs/superpowers/specs/2026-10-21-anchor-eui-identity-design.md`. Las
referencias D1–D9 y H1–H8 son de ese documento. Este plan no re-decide nada de
lo que el spec decide; si una tarea descubre que una decisión del spec es
inviable, se **para y se reporta**, no se improvisa.

Decisiones del operador ya incorporadas al spec (2026-10-21):

- no hay topic de mapa nuevo; `setup/<zone>` no cambia;
- los topics status/cmd/ack cambian de forma directa;
- no hay espera fija de reinicio: el gateway re-enumera hasta converger, con
  un plazo de 30 s.

## 0. Reglas que aplican a todas las tareas

- **Rama:** `feat/anchor-eui-identity`, cortada de `codex/b14-static-survey`.
  Esa es la línea principal: trae el servidor edge y los static survey links.
  Antes de cortar, b14 se adelantó por fast-forward hasta `2c2f574`, el commit
  de `feat/dw-irq-poll` que agrega el RX por polling que necesitan las anclas
  viejas. Esas anclas son justo la flota que esto tiene que asignar. El spec y
  este plan son el primer commit de la rama (2026-10-21).
- **Pendiente aparte, y sin resolver:** el `master` local tiene 32 commits sin
  publicar ("network-scaling-v3": cap de 4 a 32 anclas, survey a 32 nodos,
  DISCOVERY agrupado, ANNOUNCE/MULTI-POLL) que no están en b14 y que tocan
  `uwb_config`, `apos_*` y `disc_schedule`. Antes de mergear esta rama hay que
  decidir si esa línea sigue viva. Si sigue, choca con T2 y T4-T6.
- **Cada tarea cierra con las tres imágenes compilando**: producción, `cal.conf`
  y `poll.conf`. Hay que registrar el delta de `dram0_0_seg` de producción en el
  mensaje del commit. `net_uplink.c` y `blink_rx.c` compilan en la imagen de cal
  con `CONFIG_NETWORKING=n`, así que **todo símbolo público nuevo que toquen
  necesita su stub en la mitad `#else`**, o la imagen de cal no linkea.

```powershell
$env:ZEPHYR_BASE = "C:\Users\Menay\zephyrproject\zephyr"
$env:ZEPHYR_SDK_INSTALL_DIR = "C:\Users\Menay\zephyrproject\zephyr-sdk-1.0.1"
west build -b ancla_esp32s3/esp32s3/procpu
west build -b ancla_esp32s3/esp32s3/procpu --pristine -d build_cal -- "-DEXTRA_CONF_FILE=cal.conf"
west build -b ancla_esp32s3/esp32s3/procpu --pristine -d build_poll -- "-DEXTRA_CONF_FILE=poll.conf"
```

- **Cada tarea corre la suite host completa**, no sólo las suites que tocó.
  Las líneas de build están en `CLAUDE.md` §Host tests. Las suites nuevas o
  ampliadas de este plan se agregan ahí en la misma tarea que las crea. En la
  máquina con mingw.org de 32 bits, agregar `-Itests/ccp_frame/shim` a las
  suites que usan códigos errno ausentes.
- **Nada de flash ni de transmisión en el lazo `K_PRIO_COOP(0)` sin pasar por
  sus gates actuales:** un frame por step, y escrituras NVS budget-gated
  (`APOS_GW_SOLVE_BUDGET_UUS`).
- **Ciclo por tarea:** implementar, revisar, ronda de fixes, re-revisión acotada
  (convención de `CLAUDE.md` §Reporting progress). **Estimado:** ~35-43 min de
  agente por tarea de código, ~6-7 h para T1-T10. Es un orden de magnitud, no
  un compromiso, y no dice nada de cuándo funciona en hardware.

---

## T0. Base y línea de comparación

1. ~~Crear la rama (§0) y hacer commit del spec y de este plan.~~ Hecho
   2026-10-21.
2. Por cada placa (gateway + 4 anclas), registrar la salida de `anchor show` y
   `apos show`, y si tiene `anchor/id` persistido o está en 0 por defecto. Una
   placa en default va a arrancar UNASSIGNED después de T2 (spec D3,
   migración); hay que saber cuáles son **antes** de flashear.
3. Capturar 5 min de MQTT con `tools/mqtt_capture.py`: status retained,
   observaciones y posiciones. Es la línea base de H5 y H6.
4. Si se puede, guardar un extracto del log del broker. Sirve como evidencia de
   las desconexiones por client id duplicado que se sospechan hoy (spec §1.5).

**Cierre:** las capturas quedan guardadas en `tools/` o en el scratch del
operador, con nombre y fecha, y referenciadas en el commit de T0.

---

## T1. `node_eui`: un solo dueño del EUI (spec D2)

**Archivos:** `src/node_eui.{c,h}` (nuevos), `src/main.c`, `src/apos_node.c`,
`src/apos_gw.c`, `src/ccp_master.c`, `CMakeLists.txt` (en el bloque
incondicional), `tests/apos_gw/` (su línea de build).

- API:
  - `void node_eui_init(void)`
  - `bool node_eui_valid(void)`
  - `const uint8_t *node_eui(void)`, 8 bytes, con cola en cero
  - `const char *node_eui_hex(void)`, 16 hex en mayúsculas
  - `static inline void node_eui_fmt(const uint8_t eui[8], char out[17])`,
    puro, para que los consumidores formateen EUIs ajenos con el mismo formato.
- `valid` es false si hwinfo devuelve ≤ 0 **o** si los 8 bytes son cero.
  `LOG_ERR` una vez.
- `main()`: `node_eui_init()` como **primera** sentencia, antes de
  `net_config_init()`.
- Sustituir las tres lecturas de hwinfo:
  - `apos_node.c:88-107`: se conserva el log `{"apos":"ready","eui":...}`, que
    ahora sale de `node_eui_hex()`.
  - `apos_gw.c:185`.
  - `ccp_master.c:146-167`: **se conserva el fallback actual** de `root_id`
    cuando no hay EUI. Cambiar ese fallback no es parte de esta tarea.
- **Sin cambio de comportamiento.** Es la tarea de entrada precisamente porque se
  verifica sola.

**Pruebas:** `tests/node_eui/` (nueva) prueba `node_eui_fmt()` (orden de
bytes, mayúsculas, NUL). `tests/apos_gw/` agrega `src/node_eui.c` a su build;
el shim de hwinfo ya existe en `tests/apos_gw/shim/zephyr/drivers/hwinfo.h`.

**Cierre:** en una placa, el EUI del log de boot es **idéntico byte a byte** al
de antes del cambio (comparar contra T0). Compilan las tres imágenes.

---

## T2. Estado UNASSIGNED (spec D3, D9)

**Archivos:** `src/uwb_config.{c,h}`, `src/uwb_store.c`, `src/anchor_shell.c`,
`src/main.c`, `src/anchor_respond.c`, `src/uwb_slave.c`, `src/cal_shell.c`,
`src/cal_run.c`, `tests/uwb_config/`.

- En `uwb_config.h`: `UWB_ANCHOR_ID_UNASSIGNED` (`0xFFu`),
  `UWB_ANCHOR_ADDR_UNASSIGNED` (`0x00FFu`) y
  `bool uwb_config_is_assigned(const uwb_config_t *)`.
- El default de `uwb_config.c:34` pasa a UNASSIGNED. `uwb_config_set_id()`
  acepta `0..UWB_MAX_ANCHORS-1` **o** el centinela. Por esa vía
  `uwb_store.c:93` acepta un `0xFF` persistido sin tocar su lógica.
- `uwb_config_short_addr()`: rama explícita UNASSIGNED → `0x00FF`, **antes** de
  la suma `BASE + id`. Es la trampa del spec §1.4: `0x0001 + 0xFF` es `0x0100`,
  la primera dirección de tag.
- **Revisar cada consumidor de `cfg->anchor_id`**:
  - `main.c:48` y `log_config()` no deben llamar `disc_resp_delay_uus(0xFF)`;
    imprimen `"disc_delay_uus":null` y `"assigned":0`.
  - `uwb_slave.c:533` y `cal_run.c:277`: logs.
  - `anchor_respond.c:302`: la negativa va antes del cálculo del retardo.
- `anchor_respond_discovery()` y el responder WAVE **rehúsan** si el ancla no
  está asignada, con un log rate-limited como el de `anchor_respond.c:49`. El
  texto dice el síntoma: *"sin dirección asignada — corre `apos assign` en el
  gateway"*.
- Shell:
  - `anchor id auto` guarda el centinela.
  - `anchor id <n>` sigue igual.
  - `anchor show` agrega `"eui"` (de `node_eui_hex()`) y `"assigned"`.
  - `anchor reset` deja el ancla UNASSIGNED.
- `cal peer <id>` rehúsa si **este** ancla no está asignada, con un mensaje que
  explica que la imagen de cal exige `anchor id` manual.
- **Sin tocar en esta tarea:** `blink_rx` (publica `"a":255` por sí solo, porque
  copia `cfg->anchor_id`), `node_status` y `net_uplink` (son de T7).

**Pruebas (`tests/uwb_config/`):**
- el default es UNASSIGNED;
- `short_addr(UNASSIGNED) == 0x00FF` y `!= 0x0100`;
- `set_id` acepta 0..3 y `0xFF`, y rechaza 4..0xFE;
- `is_assigned` en los dos estados;
- un gateway sigue dando `0x0000` sin importar el id.

**Cierre:** en el banco, `anchor id auto` + `kernel reboot cold` →
- el banner dice `short_addr 0x00FF`, `assigned:0`;
- un tag en celda TWR no recibe `0xE4` de esa placa y la placa registra la
  negativa;
- sus observaciones salen con `"a":255` y el gateway las cuenta como
  `no_anchor`.

**Hasta que cierre T6, esta rama no va a campo:** una placa que nunca persistió
su id queda muda para TWR y todavía no existe `apos assign` para recuperarla.

---

## T3. Codec `SETADDR` / `SETADDR_ACK` (spec D5)

**Archivos:** `src/apos_frame.{c,h}`, `tests/apos_frame/`.

- `APOS_SUB_SETADDR 0x0Au` y `APOS_SUB_SETADDR_ACK 0x0Bu`;
  `APOS_LEN_SETADDR` = `APOS_HDR_LEN + 12u` (23) y `APOS_LEN_SETADDR_ACK` =
  `APOS_HDR_LEN + 13u` (24). `APOS_LEN_MAX` no cambia: agregar un
  `BUILD_ASSERT`/`_Static_assert` de que los dos quedan ≤ `APOS_LEN_MAX`.
- Estados: `APOS_SETADDR_OK 0`, `APOS_SETADDR_PERSIST_ERR 1`,
  `APOS_SETADDR_BAD_ADDR 2`, con el mismo patrón de nombres que
  `APOS_SETTAU_*`.
- `apos_frame_setaddr_build/parse` y `apos_frame_setaddr_ack_build/parse`,
  misma forma que el par `settau`.
- **Helper puro de decisión**, para que la lógica del ancla sea host-testeable
  aunque `apos_node.c` no tenga suite:
  `enum apos_setaddr_verdict apos_setaddr_check(const uint8_t my_eui[8],
  uint16_t my_addr, const uint8_t msg_eui[8], uint16_t msg_addr, uint16_t
  addr_lo, uint16_t addr_hi)`. Veredictos: `IGNORE` (EUI ajeno), `BAD_ADDR`,
  `SAME`, `CHANGE`.
  - Los gates de sesión, ventana y `src == 0x0000` **no** van aquí; se quedan
    en `apos_node.c`, igual que los de `RANGE_CMD`.

**Pruebas:** round-trip de los dos frames; cada truncamiento; un subtipo
desconocido no parsea como `SETADDR`; los cuatro veredictos del helper,
incluidas las cotas `addr_lo-1`, `addr_lo`, `addr_hi-1`, `addr_hi`, `0x00FF`.

**Cierre:** suite completa en verde y las tres imágenes compilan. Es código
muerto hasta T5/T6, así que `dram0_0_seg` debería moverse poco o nada.

---

## T4. `apos_addrmap`: codec y política de asignación (spec D4, D6)

**Archivos:** `src/apos_addrmap.{c,h}` (nuevos), `tests/apos_addrmap/`
(nueva), `CMakeLists.txt`.

- `APOS_ADDRMAP_MAX = APOS_MAX_NODES` y `APOS_ADDRMAP_VERSION 1`.
- `struct apos_addrmap { uint8_t n; struct { uint8_t eui[8]; uint16_t addr; }
  e[APOS_ADDRMAP_MAX]; }`.
- Codec con offsets fijos, little-endian, mismo estilo y mismas negativas que
  `apos_links.c`: versión, `len == 2 + 10·n`, `n ≤ MAX`, ningún EUI todo cero,
  ningún EUI ni addr repetido. Un blob inválido **se rechaza y no se borra**.
- `apos_addrmap_find_eui()`, `apos_addrmap_find_addr()`,
  `apos_addrmap_put()`, `apos_addrmap_forget()`.
- `int apos_addr_plan(const struct apos_addrmap *map, const struct
  apos_plan_peer *peers, uint8_t n, uint16_t addr_lo, uint16_t addr_hi,
  struct apos_plan_out *out)`.
  - Por peer devuelve `ADOPT` (sin frame), `KEEP` (ya en el mapa con la addr
    correcta) o `ASSIGN addr`.
  - Implementa las reglas 1-5 de D6 en ese orden.
  - Devuelve `-ENOSPC` **sin plan parcial** si no caben todos, y nombra los
    EUIs que sobran.
  - El EUI del gateway nunca entra. El caller no lo pasa, y la función además
    lo rechaza si llega con `addr == 0x0000`.
- C puro, sin Zephyr. La persistencia en settings es de T6.

**Pruebas:**
- codec: round-trip a 0, 1 y `MAX`; cada truncamiento; versión desconocida;
  bytes de cola; EUI cero; duplicados.
- plan:
  - flota numerada a mano y sin mapa → todos `ADOPT`, cero `ASSIGN` (es H3 en
    host);
  - dos peers con la misma addr, uno en el mapa → gana el del mapa;
  - dos peers con la misma addr, ninguno en el mapa → gana el EUI menor;
  - tres UNASSIGNED → addrs más bajas libres, en orden de EUI;
  - una reserva de un EUI ausente **no** se re-asigna a otro;
  - mapa lleno → `-ENOSPC`, sin efecto;
  - **determinismo**: la misma entrada en otro orden da el mismo plan.
- **Revisión de mutación**, como `tests/apos_gw/`: invertir el desempate,
  quitar la regla 2 o permitir el plan parcial; cada mutación tiene que romper
  al menos un caso.

**Cierre:** suite nueva en verde y en `CLAUDE.md` §Host tests.

---

## T5. Lado ancla del protocolo (spec D3 APOS, D5)

**Archivos:** `src/apos_node.{c,h}`.

- `ENUM_RSP` sale desde `uwb_config_short_addr()`, que después de T2 ya es
  `0x00FF` para una no asignada. Verificar que **ningún gate de `apos_node.c`
  rechaza** responder `SURVEY_BEGIN` estando UNASSIGNED.
- `RANGE_CMD` rehúsa si el ancla no está asignada, en el bloque de gates de
  `apos_node.c:~531-545`, con su propio log.
- Handler de `SETADDR`:
  1. Pasan los gates existentes: ventana abierta, `src == 0x0000`, sesión.
  2. `apos_setaddr_check()` (T3) con
     `[UWB_ANCHOR_ADDR_BASE, UWB_ANCHOR_ADDR_BASE + UWB_MAX_ANCHORS)`.
  3. `IGNORE` → nada. `BAD_ADDR` → ACK con ese estado. `SAME` → ACK OK, sin
     reinicio.
  4. `CHANGE` → `uwb_config_set_id()` y `uwb_store_save_id()`, y ACK OK o
     PERSIST_ERR según el resultado.
  5. Sólo tras un OK con cambio y **después** de que el TXFRS del ACK se
     confirme: `k_work_reschedule(&reboot_work, K_MSEC(500))` →
     `sys_reboot(SYS_REBOOT_COLD)`.
- El ACK pasa por el mismo `gw_rx_wait()` que los demás frames
  ancla→gateway, para no caer en la ventana sorda del beacon (el hard-won fact
  de PLEN_256 de 2026-10-16). **No crear una ruta TX nueva.**
- La escritura NVS ocurre en el lazo SLAVE, que es preemptible, igual que la
  del `SETTAU` existente. No necesita gate de presupuesto.

**Pruebas:** no hay suite de `apos_node.c`, por la misma razón que hoy; la
decisión está en el helper de T3. El ancla falsa de `tests/apos_gw/`
(`tests/apos_gw/shim`) se extiende en T6 para responder `SETADDR` con la misma
semántica, **llamando al mismo `apos_setaddr_check()`**, para que el fake no se
aparte del real.

**Cierre:** compilan las tres imágenes. La verificación en banco va junto con
T6, porque sin el gateway no hay quién mande el frame.

---

## T6. Lado gateway: tabla, fase ASSIGN y comandos (spec D4, D6, D7)

**Archivos:** `src/apos_table.{c,h}`, `src/apos_gw.{c,h}`,
`src/apos_store.{c,h}`, `src/apos_shell.c`, `tests/apos_gw/`,
`tests/apos_table/`.

- **`apos_table_add_peer()` deja de devolver `-EADDRINUSE`.** Ahora acepta el
  peer, keyed por EUI como antes, y lo marca `addr_conflict` si su addr la
  comparte otro EUI o es `0x00FF`. Se agrega `apos_table_needs_assign()`.
  - `apos_table_find_addr()` es ambigua con duplicados, y se documenta: sólo es
    válida con `!needs_assign()`, y RUN lo garantiza.
  - Actualizar `tests/apos_table/`: el caso que hoy espera `-EADDRINUSE` pasa a
    esperar el flag.
- `apos_gw_enum` (`apos_gw.c:~470-508`): el `LOG_ERR` de duplicado y el aborto
  se reemplazan:
  - **en RUN**, al cerrar la enumeración, `needs_assign()` → aborta con
    `{"apos_error":"... corre apos assign","eui":[...]}`;
  - **en ASSIGN**, se acepta.
- **`APOS_GW_ASSIGN`**, como sub-step-machine (un frame por step):
  1. `ENUM`, las mismas 3 rondas.
  2. `PLAN` con `apos_addr_plan()`, rango `[BASE, BASE + UWB_MAX_ANCHORS)`, y
     log JSON del plan.
  3. `SEND`: un `SETADDR` por step a cada `ASSIGN`; espera de ACK con
     `APOS_GW_APPLY_TIMEOUT_MS` y `APOS_GW_APPLY_RETRIES`; un ACK se acepta sólo
     si su EUI coincide.
  4. `PERSIST`: guarda en el mapa las entradas `ADOPT` y las `ASSIGN`
     confirmadas, **una** escritura budget-gated en `apos/addrmap`. Si falla →
     `apos_error` y abort, sin re-asignar.
  5. `CONVERGE`: re-emite `SURVEY_BEGIN` cada `APOS_GW_ENUM_SETTLE_MS` hasta
     que cada EUI asignado conteste `ENUM_RSP` desde su addr nueva, o hasta que
     venzan `APOS_GW_ASSIGN_CONVERGE_MS` (30000).
  6. `DONE` si el plan re-calculado queda vacío; si no, `ABORT` con la lista de
     EUIs. **Nunca un segundo ciclo de SEND.**
- `APOS_GW_ASSIGN_CONVERGE_MS` va en `apos_gw.h` con su derivación: el peor
  caso reportado de arranque con el driver WiFi colgado (10-20 s) más margen,
  y un `BUILD_ASSERT` de que es menor que `APOS_GW_WINDOW_S * 1000`.
- `apos_store`: `apos_store_addrmap_get()` y `apos_store_save_addrmap()` sobre
  la clave `apos/addrmap`, con el codec de T4. **Un blob rechazado se registra y
  se deja en flash**, como `apos/links`.
- Shell (`apos_shell.c`, registrado junto a los demás en `:625`), todo
  GATEWAY-only:
  - `apos assign`;
  - `apos addr` (JSON con eui, addr, id y `seen` de la última enumeración);
  - `apos forget <EUI16>`, que rehúsa si la fase no es IDLE y persiste
    inmediatamente, con el mismo precedente que `apos ref`.
  - `apos run` ya aborta por la regla de arriba; el mensaje de la shell lo
    repite.

**Pruebas (`tests/apos_gw/`, el `apos_gw.c` real contra radio, anclas y NVS
falsas):** la build agrega `src/apos_addrmap.c src/node_eui.c`. Escenarios:
- assign completo con 4 UNASSIGNED;
- flota adoptada: **cero frames `SETADDR`** y el mapa persistido igual;
- duplicado resuelto;
- ACK perdido una vez y recuperado por reintento;
- ancla con firmware viejo que nunca hace ACK → abort, su entrada no se
  persiste;
- escritura NVS fallida → abort;
- un ancla que nunca reaparece en `CONVERGE` → abort al vencer el plazo, con el
  reloj falso;
- `apos run` con un UNASSIGNED presente → rehúsa.
- En todos los casos, **lo que el log reporta coincide con lo persistido**.
- Mutación: quitar el gate de presupuesto del persist, aceptar un ACK con EUI
  ajeno, permitir un segundo SEND. Cada mutación tiene que romper un caso.

**Cierre:** suites en verde y compilan las tres imágenes. **En banco (adelanto
de H1/H3):** con la flota de T0 y `apos assign`, las placas numeradas a mano se
adoptan sin reiniciar, y las de default quedan con ids distintos tras reiniciar
solas.

---

## T7. Identidad MQTT por EUI (spec D8)

**Archivos:** `src/net_uplink.c`, `src/node_cmd.{c,h}`,
`src/node_status.{c,h}`, `src/pos_json.h` (los comentarios de topics),
`tests/node_status/`, `tests/cmd_proto/`.

- Client id: `client_id[24]`, `"anc-%s"` / `"gw-%s"` con `node_eui_hex()`. Si
  `!node_eui_valid()`, `"anc-a%04X"` con la short addr y un `LOG_ERR` que diga
  que puede chocar.
- Topics: `status_topic`, `cmd_topic` y `ack_topic` llevan `<EUI16>` para
  anclas y siguen en `gw` para el gateway (`net_uplink.c:648-668`). Cada buffer
  lleva un `BUILD_ASSERT(sizeof(buf) >= sizeof(PREFIJO) + 16)`.
- `node_cmd_init(bool is_gateway, const char *node)`. El string firmado es el
  EUI16 o `"gw"`. `node[8]` (`node_cmd.c:133`) pasa a `node[17]`.
  `cmd_verify` no cambia.
- `struct node_status`: agrega `const char *eui` y `bool assigned`, y `id` sólo
  se emite si está asignada. `node_status_offline_json(buf, len, is_gateway,
  eui, assigned, id)`. El gateway también publica su `eui`.
- **Sin publicación doble.** Los retained viejos en `.../status/<zone>/<id>` los
  borra el operador (T10, procedimiento).

**Pruebas:**
- `tests/node_status/`: payload asignado y sin asignar (sin `id`), last-will con
  `eui`, y longitud máxima contra el buffer real.
- `tests/cmd_proto/`: un vector nuevo con `node = "7CDFA1B2C3D40000"`. **El
  mismo vector** entra a `edge/tests/test_tdoa.py` en T9.

**Cierre:** en banco, dos anclas forzadas al mismo `anchor id` (antes de
reasignarlas) **ya no se tiran del broker**: cada una conecta con su propio
client id y publica su propio status. Es un adelanto de H5.

---

## T8. EUI en la observación BLINK y búsqueda por EUI en `tdoa_gw` (spec D8)

**Archivos:** `src/pos_json.{c,h}`, `src/blink_rx.c`, `src/tdoa_gw.c`,
`tests/pos_json_blink/`.

- `struct pos_blink_obs` agrega `uint8_t eui[8]` y `bool has_eui`.
  `pos_json_blink()` emite `"u":"<EUI16>"` cuando `has_eui`.
  `pos_json_blink_parse()` acepta `"u"` **opcional**: exactamente 16 hex; si
  está mal formado, se rechaza la observación entera. No se ignora, porque una
  identidad corrupta es peor que ninguna.
- `blink_rx.c` llena `eui` con `node_eui()` cuando `node_eui_valid()`.
- `tdoa_gw.c`: `anchor_xyz()` se vuelve `anchor_resolve(const struct
  pos_blink_obs *, ...)`:
  - con `has_eui`, busca `node[i].eui` en el survey;
  - sin él, se queda el camino actual `BASE + anchor_id`.
  - El `aid` que se pasa a `tdoa_collect` sigue siendo
    `node.short_addr - UWB_ANCHOR_ADDR_BASE`, con la misma cota
    `< UWB_MAX_ANCHORS`. **`tdoa_collect` no se toca.**
  - EUI sin nodo → `no_anchor`, igual que hoy.
- Corregir el comentario de `pos_json.h:65` ("0..UWB_MAX_ANCHORS-1"), que ya
  mentía (spec 2026-09-02 §5) y ahora incluye 255.

**Pruebas (`tests/pos_json_blink/`):** `"u"` presente, ausente, con 15 o 17
hex, o con caracteres no hex; round-trip; y **el peor caso medido impreso** que
quede < `POS_JSON_BLINK_MAX_LEN`. En el commit se registra la cifra medida, no
el ~106 estimado.

**Cierre:**
- el delta de `dram0_0_seg` queda registrado (`OBS_QUEUE_DEPTH` × ~9 B
  esperados);
- un gateway nuevo con anclas de T0 sigue resolviendo por `"a"`;
- con anclas nuevas resuelve por `"u"`, y los fixes no cambian (adelanto de
  H6).

---

## T9. Edge y herramientas (spec D8, §3)

**Archivos:** `edge/tdoa_server/service.py`, `edge/tdoa_server/nodecmd.py`,
`edge/tools/nodecmd.py`, `edge/tools/calibrate.py`, `edge/tools/xcheck.py`,
`tools/rx_cal.py`, `edge/tests/test_tdoa.py`, `edge/README.md`.

- `service.py`:
  - `parse_obs()` lee `"u"` opcional;
  - un mapa EUI → id construido desde el payload de status, que ya llega por
    `status/<zone>/#`;
  - las observaciones con `"u"` se resuelven por ese mapa a un id, y de ahí a
    posición con `parse_setup()`, **que no cambia**;
  - `"u"` desconocido → contador `unknown_eui`, que se reporta en el log de
    stats; no se atribuye a ningún id;
  - `status_events()` nombra al ancla por `eui` (e `id` si existe).
- `ANCHORS_JSON` y `OFFSETS_DTU` aceptan como llave un entero (id) o un string
  EUI16.
- `nodecmd.py` (los dos): `--node <EUI16|gw>`; `challenge_from_status()` lee
  el topic `.../status/<zone>/<EUI16>`.
- `rx_cal.py`, `calibrate.py` y `xcheck.py`: aceptan `"u"`, y por defecto
  reportan por EUI e id.

**Pruebas:** `python -m pytest edge/tests` con:
- el vector de firma de T7;
- el parseo de una observación con y sin `"u"`;
- el mapa EUI → id desde el status;
- `"u"` desconocido contado y descartado.

**Cierre:** pytest en verde. Este componente **se despliega primero** (spec
§3), así que también se prueba contra el firmware de T0 para confirmar que
sigue funcionando sin `"u"`.

---

## T10. Documentación

- `CLAUDE.md`:
  - Console: `anchor id auto`, `apos assign`, `apos addr`, `apos forget`.
  - Layout: `node_eui`, `apos_addrmap`.
  - Host tests: las líneas nuevas.
  - Hard-won facts:
    - el client id duplicado de MQTT y sus desconexiones;
    - la trampa `0x0001 + 0xFF = 0x0100`;
    - que el EUI del ESP32-S3 es la MAC + `0000`.
  - El "start here" de status apunta a este plan.
- **`docs/anchor-commissioning.md` (nuevo)**, procedimiento de operador:
  1. montar y encender;
  2. `apos assign`;
  3. `apos addr` y relacionar cada EUI con su placa física usando la MAC de la
     etiqueta;
  4. `apos gauge`;
  5. `apos run`;
  6. `apos apply`.
  Además:
  - cómo reemplazar un ancla (`apos forget` + `assign`);
  - cómo borrar los retained huérfanos `uwb/anchor/status/<zone>/0..3`;
  - qué esperar en el primer flasheo de una placa que estaba en default (la
    migración de D3).
- El spec cambia su estado a "implementado, NO corrido en hardware", con el
  commit.

---

## T11. Gates de hardware (spec §4.2)

Orden de menor a mayor riesgo, para que un fallo temprano no gaste lo demás:

1. **H3**: flota actual numerada a mano → `assign` adopta todo, con cero
   reinicios.
2. **H1**: 4 placas con `anchor reset` → 4 ids distintos que sobreviven a
   `kernel reboot cold` en ambos lados.
3. **H2**: dos placas forzadas al mismo id.
4. **H8**: tiempo desde el ACK hasta el `ENUM_RSP` en la addr nueva, en cada
   caso de arranque del WiFi. **Se registra el número**, y si el driver colgado
   retrasa el lazo UWB, se escribe como hard-won fact.
5. **H7**: un ancla con firmware viejo.
6. **H5 / H6**: MQTT y TDoA contra la línea base de T0.
7. **H4** se vigila durante todos los anteriores: ningún `"beacon started but
   TXFRS never completed"` y el heartbeat `gw_sf` en 200.0 ms.

Cada gate queda como `NO EJECUTADO` / `PASA` / `FALLA` con su número en el
spec. Un gate que falla no se "arregla" relajando su criterio.

---

## Dependencias

```
T0 ─┬─ T1 ──────────────┬─ T7
    ├─ T2 ──┐           └─ T8
    ├─ T3 ──┼─ T5 ─┐
    └─ T4 ──┘      └─ T6
T7 + T8 ─ T9        (T9 puede empezar en cuanto D8 esté fijo; su cierre espera los vectores de T7)
T1..T9 ─ T10 ─ T11
```

T1, T2, T3 y T4 no comparten archivos y pueden ir en paralelo. T5 y T6 tocan
`apos_*` y van en serie. T7 y T8 comparten `pos_json.h` sólo en comentarios;
si corren en paralelo, se coordinan ese archivo.

---

## Estado de implementación (2026-10-21)

Commits (sobre el spec/plan `ab5ca42`): T1 `37f5251`, T2 `e4a3169`, T3
`0187837`, T4 `f9edaf1`, T5+T6 `65f850b`, T7 `76b3f07`, T8 `28be4b8`, T9
`be98ffb`, T10 `7cff005`, más un commit de correcciones de la revisión
independiente (el siguiente a este texto).

Verificación: 38 suites host, `edge` 52/52 (`python -m unittest discover -s
tests`). Producción, cal y poll compilan sin warnings nuevos. Producción
`dram0_0_seg` pasó de 274 320 a 275 824 B: unos 1 KB son las dos colas de
observaciones, que ahora llevan el EUI. Mutaciones deliberadas, todas
detectadas: `apos_addrmap` 4/4 y la lógica de assign en `apos_gw` 11/11.

**Desviaciones respecto al texto de arriba, todas deliberadas:**

1. **El assign converge ANTES de persistir**, no después (T6 decía SEND →
   PERSIST → CONVERGE). Un ancla se reinicia 500 ms después de que su ACK SALE
   de su radio, sin saber si llegó. Con un ACK perdido en el aire, los
   reintentos encuentran un ancla reiniciando y con la ventana cerrada. El
   orden original la contaba como fallida y no la registraba, aunque ya estaba
   en su dirección nueva. Ahora un ACK ausente no es un fallo: el ancla queda
   "pendiente" y la re-enumeración lo decide. Se persiste lo ADOPTADO más lo
   ASIGNADO que se confirmó con ACK **o se vio** en su dirección nueva. Sigue
   siendo fallo una negativa explícita del ancla, o un TX que nunca salió.
   Lo encontró la revisión independiente.
2. **El assign también reescribe las direcciones del survey guardado** (por
   EUI) para las anclas que se movieron. Además, net_uplink ahora republica el
   topic retained `setup/<zone>` cada vez que cambia el survey; antes solo lo
   publicaba al reconectar, también después de `apos apply`. Sin esto, el edge
   (que ubica por id el payload de setup, cuyo schema congelado no lleva EUI)
   le pondría a un ancla movida las coordenadas de otra. Si el resultado
   dejaría dos nodos con una misma dirección, el survey no se toca y se pide
   re-survey. Lo encontró la revisión.
3. **`apos forget` y `apos addr clear` quedan en cola** (fase
   `APOS_GW_FORGET`): el gateway escribe el mapa desde su lazo con el
   presupuesto de solve, no desde el hilo del shell. Lo encontró la revisión.
4. **Ancla: si el ACK no pudo salir, se deshace el id guardado**, para que
   config viva, status y NVS sigan diciendo lo que corre el radio. Lo encontró
   la revisión.
5. **Agregados que no estaban en el plan:**
   - `apos addr clear`, para un mapa ilegible que el operador decide descartar.
   - Un ancla sin asignar también rechaza SETPOS y SETTAU.
   - Un assign sin ninguna ancla que conteste es error.
   - El edge borra el EUI → id de un ancla que reporta sin id, y las llaves
     numéricas se leen en base 10 como antes.
6. Las líneas de prueba nuevas de `CLAUDE.md` se agregaron juntas en T10, no en
   cada tarea.

**Hallazgo de la revisión NO corregido:** `AS_PERSIST`, igual que `AP_MARK` y
`AP_COMMIT` de apply, espera sin plazo un paso con `APOS_GW_SOLVE_BUDGET_UUS`
disponible. Por diseño ese paso llega una vez por superframe (el primero
después del beacon), así que se dejó igual que el código existente en vez de
inventar un plazo solo para assign.

**Abierto:** T11 completo (H1-H8), y decidir qué pasa con los 32 commits sin
subir de `master` ("network-scaling-v3") antes de mergear esta rama.
