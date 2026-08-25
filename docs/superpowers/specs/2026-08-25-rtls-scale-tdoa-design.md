# RTLS a escala: 100 tags como paquete base, migración a TDoA — diseño

**Fecha:** 2026-08-25
**Estado:** Diseño aprobado, pre-implementación
**Alcance:** La arquitectura que lleva este RTLS de 11 tags / 4 anclas a 100 tags
como paquete base de producto, con camino a ~100 anclas. Cubre los dos
repositorios: `ANCLA_ESP32S3` (anclas + gateway) y `tag_testting` (tag).
**Reemplaza en parte:** `tag_testting/spec/2026-06-17-uwb-mac-protocol-contract.md`
§1 (la premisa anti-TDoA) y §5.1 (la afirmación de que los tiers devuelven
airtime). Ver §11.

---

## 1. Objetivo y restricciones

**Objetivo de producto:** soportar **100 tags como mínimo** en el paquete base,
incluyendo el caso de carga que un cliente va a crear sin avisar — *cambio de
turno*, 100 personas caminando a la vez, todas exigiendo 5 Hz.

**Objetivo de despliegue:** ~100 anclas por sitio, precisión 10–30 cm.

**Restricciones fijadas antes de este diseño:**

| Restricción | Valor | Origen |
|---|---|---|
| PHY | **congelado en canal 5 / PLEN_1024 / PAC32 / código 9 / 850 kbps / SFD 4z / STS off** | decisión 2026-08-25 |
| Backhaul de anclas | **WiFi en todas las anclas** (el ESP32-S3 ya lo tiene; el stack ya está en el firmware, solo compilado en modo gateway) | decisión 2026-08-25 |
| Cálculo de posición | **en el gateway**, no en el tag | decisión 2026-08-25 |
| Configuración de tags | debe ser posible **por UWB**, originada en la plataforma vía MQTT | requisito 2026-08-25 |
| Robustez contra metal | prioridad alta | requisito 2026-08-25 |

El congelamiento del PHY es la decisión de mayor apalancamiento en el
calendario: **no invalida ninguna calibración de retardo de antena**, así que
los gates de hardware pendientes (§10) se desbloquean de inmediato y corren en
paralelo al software.

---

## 2. Estado actual: los límites, verificados

Todo lo de esta sección se leyó del código, no se estimó.

### 2.1 El techo de tags es 11, y es duro

`gw_core_join()` recorre los asientos y **devuelve `false`** en el tag 12
(`src/gw_core.c:75-77`). `GW_N_CFP = UWB_FRAME_N_CFP = 11`.

### 2.2 Los tiers no devuelven capacidad — el contrato afirma algo no implementado

El contrato MAC §5.1 dice: *"Bajar un tier devuelve airtime — el mecanismo que
permite que ~12 slots CFP sirvan a 100 tags cuando solo unos pocos se mueven."*

**Eso no está implementado.** `gw_core.c` guarda `tier` en el asiento y nunca lo
lee para programar. `gw_core_build_slotmap()` (`src/gw_core.c:22-28`) escribe la
dirección del asiento en su slot **cada superframe, sin condición**. Un tag en
IDLE que despierta cada 25 superframes sigue siendo dueño de su slot el 100% del
tiempo y lo desperdicia 24/25.

**Los tiers ahorran batería del tag, no capacidad de red.**

### 2.3 El tag interpreta "no me toca" como "me reclamaron el asiento"

`uwb_net.c:282` y `:320`: `if (!ev->in_map) { c->state = UWB_ST_SCAN; return
UWB_ACT_TO_SCAN; }`. Cualquier multiplexado ingenuo de slots rebotaría a cada
tag a SCAN en cada superframe que no le toque. **Esto es lo que obliga al bump
de `proto_ver`** — ver §4.1.

### 2.4 El discovery de anclas es O(N) en airtime

`disc_resp_delay_uus(id) = DISC_BASE_UUS + id * DISC_SLOT_UUS` = `2000 + id*3500`
uus (`src/disc_schedule.h`), contra `DISCOVERY_WINDOW_MS = 15` en el tag
(`src/uwb_net_runner.c:65`):

| Anclas | Retardo del último | Veredicto |
|---|---|---|
| 4 (hoy) | 12 500 uus ≈ 12.8 ms | cabe apenas en 15 ms |
| 8 | 26 500 uus ≈ 27.2 ms | rebasa la ventana *y* `TX_COMPLETE_TIMEOUT_MS`=18 |
| 10 | 33 500 uus ≈ 34.4 ms | 17% del superframe por ronda |
| 100 | 348 500 uus ≈ **357 ms** | **1.8 superframes** — imposible |

Ninguna constante salva esto. Ver §7: TDoA **borra el rol** en vez de optimizarlo.

### 2.5 El cap de 4 anclas está en tres lugares que se refuerzan

| Límite | Dónde |
|---|---|
| `UWB_MAX_ANCHORS 4` | `src/uwb_config.h:21` |
| `UWB_FRAME_MAX_ANCHORS 4` | `src/uwb_frame_802_15_4z.h:31` — **archivo byte-idéntico con el tag** |
| `APOS_MAX_NODES 8` | `src/apos_geom.h:33` |

### 2.6 El slot map broadcast no es un eje escalable

`UWB_FRAME_LEN_BEACON = 15 + 2*11 = 37 = UWB_FRAME_MAX_LEN`. A 56 slots serían
127 bytes = **exactamente el PSDU máximo de 802.15.4**. El techo absoluto de un
slot map broadcast es ~55 entradas, y su airtime crece linealmente comiéndose el
superframe.

### 2.7 Un gateway = un beacon = un dominio de colisión = una celda

Nada en el firmware coordina dos gateways. `POS_JSON_ZONE_NAME "852541"`
(`src/pos_json.h:24`) y `UWB_FRAME_PANID 0xCADE` son constantes de compilación,
así que dos celdas no se distinguen ni en el aire ni en MQTT sin builds
separados.

Esta es **literalmente** la limitación que Ciholas documenta haber eliminado en
Bernoulli: *"en revisiones previas el dispositivo master servía como
sincronizador de tiempo de la red, lo que limitaba el tamaño de las
instalaciones al alcance UWB del master."*

### 2.8 Divergencia real en el contrato PHY entre los dos repos

| | Ancla `src/uwb_phy.h` | Tag `src/phy_config.c:170` |
|---|---|---|
| SFD timeout | `(1025 + 8 - 32)` = **1001** | `(1024 + 1 + 8 - 8)` = **1025** |

La fórmula es `PLEN + 1 + SFD − PAC` y **ambos corren `DWT_PAC32`**, así que 1001
es el correcto. El comentario del tag dice "PAC 8", que está obsoleto respecto a
su propio código. No rompe nada funcionalmente — solo deja el RX abierto más de
lo necesario tras un falso detect de preámbulo, y por eso nadie lo notó. Pero
`uwb_phy.h` dice *"every node must match these values exactly"*. Se corrige en
§9 Martes.

---

## 3. Presupuesto de airtime: el modelo

Toda cifra de capacidad de este documento sale de aquí. Duración de símbolo de
preámbulo a 64 MHz PRF = **1017.63 ns**; payload a 850 kbps = **9.41 µs/byte**;
SFD 4z 8 símbolos = 8.1 µs; PHR estándar = 22.4 µs.

### 3.1 Airtime de frame a PLEN_1024 (el PHY congelado)

| Frame | Bytes (+FCS) | Airtime |
|---|---|---|
| Poll `0xE0` | 13 | 1194 µs |
| Respuesta `0xE4` | 22 | 1279 µs |
| Beacon `0xE5` | 39 | **1439 µs** |
| Keepalive `0xE8` | 14 | 1204 µs |
| Blink TDoA (nuevo) | 16 | **1223 µs** |

**Validación del modelo contra el código existente**, que es lo que lo hace
confiable y no autoconsistente:

- Beacon 1439 µs vs `BEACON_OCCUPANCY_UUS` = 1500 uus (1538 µs) ✓
- `POLL_RX_TO_RESP_TX_DLY_UUS` = 2000 uus (2051 µs): el piso derivable es
  `122 µs (payload RX) + ~200 µs (SPI/cómputo medido) + 1042 µs (preámbulo de la
  respuesta, que debe transmitirse antes del RMARKER)` = **1364 µs**, dejando
  687 µs de margen ✓
- Slot TWR de 4 anclas = `4 × (1.194 + 2.051 + 1.279)` = **18.1 ms**; CFP usable
  = `200 − 1.44 − 3.08 − 6.15` = 189.3 ms → `189.3/18.1` = **10.5 slots** vs el
  `N_CFP = 11` real ✓ (el 11 es ligeramente optimista)

### 3.2 Capacidad comparada, 100 tags

Presupuesto en *slot-superframes por ventana de 25 SF*. Costo por tag:
FAST (5 Hz) = 25, SLOW (1 Hz) = 5, IDLE (0.2 Hz) = 1.

TWR: `N_CFP = 11` → 275 slot-superframes.
TDoA: slot de blink = `1223 µs + 100 µs` de guarda ≈ 1.32 ms; presupuesto
`200 − 1.44 (beacon) − 6 (CAP) − 3.1 (guardas)` = 189.5 ms → **143 slots**.

| Caso de carga, 100 tags | TWR (hoy) | **TDoA** |
|---|---|---|
| Todos IDLE (0.2 Hz) | 36% ✓ | 6% ✓ |
| Todos a 1 Hz | 182% ✗ | 14% ✓ |
| 20% en movimiento | 211% ✗ | 30% ✓ |
| **Cambio de turno: 100 a 5 Hz** | **✗ 9× corto** | **✓ 70%** |
| Movedores simultáneos máx a 5 Hz | **11** | **143** |
| Capacidad máx a 0.2 Hz | 275 | **~3 575** |

**Conclusión: TDoA es requisito de producto, no optimización.** TWR a PLEN_1024
es permanentemente 9× corto en el caso que decide, y ninguna cantidad de celdas
lo arregla — el cambio de turno ocurre *dentro* de una celda.

### 3.3 Por qué el PHY se congela en PLEN_1024

Bajar PLEN cuesta ganancia de integración de preámbulo:
`10·log₁₀(1024/256)` = **−6.0 dB** de sensibilidad (1024→512 = −3.0 dB). Contra
metal y NLOS se quiere **más** preámbulo, no menos. El preámbulo corto compra
**airtime**, no robustez.

Como TDoA ya resuelve la capacidad (§3.2) a PLEN_1024 con 30% de margen, **no
hay razón para gastar esos 6 dB**. Se conserva el preámbulo largo por rango y
robustez contra metal, que es la prioridad declarada.

**Corolario de calendario:** cambiar el PHY invalida toda calibración de retardo
de antena de la flota (nota propia del tag: *"a phy_option change invalidates the
record for a whole fleet at once"*). Congelarlo ahora significa **calibrar una
sola vez**, empezando hoy.

### 3.4 La defensa real contra metal ya está escrita

Más rendidor que cualquier ajuste de PLEN, y ya existe host-testeado:

- `0xE4` ya carga `cir_power` / `cir_quality` → detector de NLOS
- `pos_ekf.c` ya hace *per-range innovation gating* → rechazo de rangos inflados

En TDoA estas dos piezas se mueven al gateway junto con el solver (§4.3) y ahí
valen más, porque el gateway ve **todas** las anclas que oyeron el blink, no las
4 que el tag eligió.

---

## 4. Arquitectura objetivo

### 4.1 El scheduler: separar "dueño del asiento" de "le toca este superframe"

Es el cambio que rompe el cap de 11 tags, y **es arquitectura-neutral**: en TDoA
el gateway sigue asignando slot y tasa por tag con leases y downlink. Solo cambia
qué pasa *dentro* del slot. No es trabajo puente — es la Fase 1 de la
arquitectura final.

**Hoy:** `slot_index` del GRANT es a la vez identidad de asiento y horario, y el
slot map los confunde.

**Objetivo:** dos conceptos separados.

- **Asiento** (`seat_id`): identidad estable, gobernada por lease + keepalive.
  Concedida en GRANT, sobrevive superframes en que el tag no transmite.
- **Horario** (slot map del beacon): a quién le toca **este** superframe. El
  gateway lo construye desde `(seat, tier, frame_counter)`.

El gateway rota los asientos de tier bajo por los slots disponibles, de modo que
un tag IDLE consume 1 slot-superframe de 25 en vez de 25 de 25.

**Cambio en el tag (`uwb_net.c`):** `!in_map` deja de significar "me reclamaron"
y pasa a significar "no me toca, duermo". La pérdida de asiento se detecta por lo
que **ya existe y ya es correcto**: `lease_remaining` envejecido desde
`frame_counter` (`lease_age()`, `uwb_net.c:162`) más el keepalive. El chequeo de
`in_map` era redundante con el lease; el lease es el que tiene la semántica
correcta.

**Esto es un bump `proto_ver` 2 → 3** y por tanto un día-bandera en ambos
firmwares. Es aceptable porque los dos son nuestros y ambos se reflashean; **no
es aceptable dejarlo implícito** — un tag v2 contra un gateway v3 no se
degradaría, se quedaría sordo en SCAN (el tag descarta beacons con `proto_ver`
distinto, `UWB_NET_PROTO_VER`), lo cual es el modo de falla correcto pero hay que
saberlo antes de flashear.

El formato de aire del slot map **no cambia** — solo su significado. La longitud,
el orden de bytes y `UWB_FRAME_N_CFP` quedan idénticos.

### 4.2 Sincronía de anclas: el riesgo del proyecto

El contrato MAC §1 descartó TDoA porque *"distribuir inalámbricamente el reloj
sub-nanosegundo que TDoA necesita no es viable"* a 100 m. **Esa premisa es falsa
en este hardware**, y los dos primitivos necesarios ya están presentes y uno ya
está en producción:

- `dwt_readrxtimestamp()` → timestamps de 40 bits a **15.65 ps** (256 DTU =
  4.006 ns), 64× más fino que el nanosegundo objetivo
- `dwt_readclockoffset()` → ya usado en `src/ss_initiator.c:244`, normalizado por
  `2^26` → **~14.9 ppb por LSB**

**El presupuesto:** dos cristales a ±20 ppm derivan 40 ppm relativos = **8 µs
sobre un intervalo CCP de 200 ms**. Para sub-ns hace falta la *tasa* de deriva a
~5 ppb.

Dos caminos, y el bueno no es el obvio:

| Vía | Resolución | Rol |
|---|---|---|
| CFO (`dwt_readclockoffset()`) | ~15 ppb/LSB → 5 ppb promediando ~10 CCPs (2 s) | adquisición rápida, cross-check |
| **Timestamps CCP consecutivos** | `15.65 ps / 0.2 s` = **78 ppt** | **el estimador principal** |

**El estimador ya existe como patrón probado en este código:**
`tag_testting/src/beacon_sched_core.c` es un estimador de periodo de largo
baseline en Q16.16 sin float, host-testeado. `sync_model.c` se construye copiando
esa forma, no inventándola. Es la misma matemática con otro observable.

**Coasteo:** un ancla que pierde CCPs coastea en su modelo de deriva. El residual
es de segundo orden — cambio de la tasa de deriva por temperatura, ~8 ppb/s con
un XO común, o **1.6 ppt por intervalo de 200 ms** — así que coastear segundos es
seguro. La temperatura no es el enemigo que parece.

**Topología del árbol:** cada salto suma ruido. Para ~100 anclas en un sitio,
dos niveles alcanzan: gateway → anclas *master* → anclas hoja. **Con WiFi en
todas las anclas el árbol se coordina fuera de banda**, que es exactamente lo que
compra la decisión de backhaul.

**Por qué WiFi basta donde parecería que hace falta Ethernet:** en Sewio y
Ciholas el cable existe para *distribuir sincronía*. Aquí la sincronía va por
UWB, así que WiFi solo carga datos — su jitter afecta la **latencia** del fix,
nunca la **precisión**, porque el timestamp ya viene estampado en tiempo DW3220.
Esto es lo que hace viable el hardware actual sin rediseño de PCB.

### 4.3 Cálculo de posición en el gateway

Obligatorio en TDoA (solo la infraestructura tiene los timestamps) y valioso por
sí solo: es un hito **desacoplado** que puede aterrizar en TWR antes de TDoA.

**El porte es un cambio de modelo de medición, no un rewrite:**

| Módulo | Hoy | En TDoA |
|---|---|---|
| `pos_solver.c` | Gauss-Newton sobre modelo de **rango**, inversa 2×2 cerrada, **ya sin CMSIS** | Gauss-Newton sobre **diferencia de rangos** — misma maquinaria, otro residual y jacobiano |
| `pos_residual.c` | residual RMS de rango, **ya separado justo para este reuso** | residual de diferencia de rangos |
| `pos_ekf.c` | EKF acoplado, updates escalares secuenciales, sin inversión de matriz | misma estructura, medición de diferencia |

Los tres son **C puro host-testeado** y portan al ESP32-S3 del gateway casi
verbatim.

**Flujo:** las anclas estampan blinks → `(tag_eui, anchor_id, rx_ts, cfo)` por
MQTT → el gateway convierte al tiempo común con `sync_model` → multilatera.
Mínimo 3 anclas para 2D, 4+ para sobredeterminado.

**Costo en el uplink:** rangos/timestamps crudos en vez de un fix resuelto.
En TWR serían `4 × (id 1 + rango 2)` = 12 B contra los 10 B actuales de
`x/y/residual` — prácticamente gratis.

### 4.4 Downlink de configuración por UWB

**La tensión a nombrar:** el ahorro de energía de TDoA viene de que el tag es
*solo TX*. Un downlink lo obliga a escuchar, lo que parece contradecir el
beneficio.

**No lo contradice, porque el tag ya escucha.** `beacon_track_core.c` (predictor
EMA + ventana RX angosta) y `beacon_sched_core.c` (planificador de despertares)
ya lo hacen despertar en horario predicho a oír el beacon.

**Diseño:** un bit de *config pendiente* en el beacon; solo el tag apuntado
extiende su ventana RX. Frames nuevos:

| Código | Nombre | Dirección |
|---|---|---|
| `0xEC` | CONFIG_SET | gateway → tag, direccionado |
| `0xED` | CONFIG_ACK | tag → gateway, en CAP |

(`0xEB` está tomado por ALERT — no reusar.)

**Ruta:** plataforma → MQTT → gateway → UWB → tag.

**El gateway lleva el downlink**, no las anclas: ya es dueño del beacon y del
horario, y un solo escritor del schedule evita conflictos. Anclas como relevo
para cobertura es Fase 5, no Fase 1 — aunque el backhaul ya lo permitiría.

### 4.5 Direccionamiento, zonas y multi-celda

- `POS_JSON_ZONE_NAME` y `UWB_FRAME_PANID` pasan de constantes de compilación a
  **configuración en NVS**, para que una celda se provisione sin recompilar.
  Sin esto no hay despliegue multi-celda con un solo binario.
- El pool de direcciones cortas (`alloc_short_addr()`, contador monotónico sin
  reuso) es la raíz documentada del bug de `Tid` y hay que darle reuso. La
  identidad de plataforma sigue siendo `tag_id_from_eui()` enmascarado a
  `0x7FFFFFFF` — **ese contrato no se toca** (ver el registro de `Tid` en
  `CLAUDE.md`).

---

## 5. Migración por fases

| Fase | Contenido | Estado tras la fase |
|---|---|---|
| **1** | Scheduler asiento/horario (§4.1), `proto_ver` v3 | TWR sirve ~275 tags IDLE / 11 movedores. Cap de 11 roto. |
| **2** | `sync_model` + CCP entre anclas, medido en hardware | Sincronía sub-ns demostrada o refutada. **Gate de decisión.** |
| **3** | Blink TDoA + solve en gateway (§4.3) + backhaul de timestamps | 100 tags a 5 Hz. Objetivo de producto alcanzado. |
| **4** | Downlink `0xEC`/`0xED` (§4.4) + zonas en NVS (§4.5) | Configurable desde plataforma, multi-celda. |
| **5** | Anclas como relevo de downlink; TWR conservado en el perímetro | Cobertura fuera del casco convexo. |

La Fase 2 es el **gate real del proyecto**. Si la sincronía sub-ns no se
demuestra en hardware, la Fase 3 no procede y el producto queda en TWR con el
scheduler de Fase 1 — 9× corto del objetivo. Por eso la Fase 2 se de-riskea en
C puro host-testeado **antes** de tocar radio.

---

## 6. Costos y riesgos aceptados

| Riesgo | Magnitud | Mitigación |
|---|---|---|
| Precisión TDoA 10–30 cm vs TWR 5–10 cm | dentro del requisito, pero es degradación real | depende directo de la calidad de sincronía; medir en Fase 2 |
| TDoA degrada fuera del casco convexo de anclas | perímetro impreciso | TWR conservado ahí (Fase 5) o densidad extra de anclas |
| El tag pierde la elección de anclas | en TWR elegía sus mejores 4 por CIR; en TDoA lo oye quien lo oiga | la densidad de instalación pasa a ser **requisito**, no recomendación |
| `proto_ver` v3 es día-bandera | un tag v2 contra gateway v3 queda **sordo en SCAN** | ambos firmwares son nuestros; reflasheo coordinado |
| La sincronía no alcanza sub-ns en hardware | mata la Fase 3 | Fase 2 es un gate explícito, no un supuesto |

---

## 7. Lo que **no** se hace, y por qué

Comprometer TDoA mata trabajo que parecía necesario. Dejarlo explícito evita que
se re-litigue:

- **No se arregla el discovery O(N) (§2.4).** En TDoA el tag no descubre anclas:
  emite y las anclas escuchan. **No hay binding tag↔ancla**, así que el cap de 4
  anclas y el stagger lineal de 3.5 ms dejan de ser el problema. Se **borra el
  rol**, no se optimiza el mecanismo.
- **No se adopta MULTI-POLL `0xE3`.** Optimiza un slot TWR que TDoA reemplaza.
- **No se cambia PLEN / PAC / turnaround.** Congelado (§3.3), y por eso la
  calibración arranca ya.
- **No se crece `N_CFP` ni el slot map broadcast.** Tope estructural en ~55
  entradas por el PSDU de 127 bytes (§2.6). El eje de escala es el scheduler
  (§4.1), no la longitud del mapa.
- **No se añade una quinta ancla de ranging TWR.** Es trabajo de ingeniería en
  `UWB_MAX_ANCHORS`, el stagger de `disc_schedule`, el `TX_COMPLETE_TIMEOUT_MS`
  re-derivado y `UWB_FRAME_MAX_ANCHORS` del tag — y TDoA lo vuelve innecesario.

---

## 8. División de trabajo: dos agentes

El trabajo vive en **dos repositorios separados** con archivos compartidos que
deben permanecer byte-idénticos. Esa es la fuente principal de conflicto, así que
la regla va primero.

### 8.0 Regla de archivos compartidos — no negociable

| Archivo | Dueño (fuente de verdad) | El otro lado |
|---|---|---|
| `src/uwb_frame_802_15_4z.{c,h}` | **Agente-ANCLA** | copia byte-por-byte, corre sus propios host tests |
| `src/cal_math.{c,h}` | **Agente-TAG** | copia byte-por-byte |

**El Agente-TAG no edita el codec de frames.** Si necesita un cambio en el
formato de aire, lo pide al Agente-ANCLA, y luego copia el archivo completo.
Un `diff` que no sea vacío entre las dos copias es un bug de formato de aire
esperando ocurrir, y ya está documentado como tal en ambos `CLAUDE.md`.

**Verificación obligatoria antes de cerrar cualquier tarea que toque un
compartido** — debe salir vacío:

```bash
diff "ANCLA_ESP32S3/src/uwb_frame_802_15_4z.c" "tag_testting/src/uwb_frame_802_15_4z.c"
diff "ANCLA_ESP32S3/src/uwb_frame_802_15_4z.h" "tag_testting/src/uwb_frame_802_15_4z.h"
```

### 8.1 Agente-ANCLA — repo `ANCLA_ESP32S3`

Dueño del MAC, del gateway, del formato de aire y de la sincronía.

| # | Tarea | Fase | Host-testeable |
|---|---|---|---|
| A1 | `src/mac_budget.{c,h}` + `tests/mac_budget/` — modelo de airtime y capacidad. Calcula airtime de frame, piso de turnaround, guardas, minislot, el **máximo factible** de slots CFP en TWR y de slots de blink en TDoA, desde PLEN/PAC/rate/payload. **Debe pinear las constantes actuales como regresión** (beacon 1439 µs vs `BEACON_OCCUPANCY_UUS`, máximo factible ≈ 10–11 contra el `N_CFP` real de 11) para que el modelo se valide contra la realidad y no solo consigo mismo. | 1 | sí |
| A2 | `BUILD_ASSERT` en **`src/uwb_mac.h`** (ANCLA-only) que verifica que `UWB_FRAME_N_CFP` no exceda el máximo factible de `mac_budget`. **`N_CFP` sigue siendo un literal en el aire** congelado por `proto_ver`: el modelo *verifica* que el literal sea factible, nunca lo calcula — calcularlo en dos repos independientes es frágil. **El assert NO va en `uwb_frame_802_15_4z.h`**: ese archivo es byte-idéntico con el tag y el tag no tiene `mac_budget.h` (regla §8.0). | 1 | sí |
| A3 | Scheduler asiento/horario en `gw_core.{c,h}`: `seat_id` separado del slot del beacon, rotación tier-aware desde `(seat, tier, frame_counter)`. Ampliar `tests/gw_core/`: 100 asientos lógicos sobre 11 slots, mezcla de tiers, expiración de lease bajo rotación. | 1 | sí |
| A4 | `UWB_PROTO_VER` 2 → 3 en `uwb_frame_802_15_4z.h` + nota de día-bandera. Avisar al Agente-TAG para que copie. | 1 | sí |
| A5 | Reuso de direcciones cortas en `alloc_short_addr()` sin romper el contrato de `Tid` (`tag_id_from_eui()` enmascarado a `0x7FFFFFFF` **no se toca**). | 1 | sí |
| A6 | `src/sync_model.{c,h}` + `tests/sync_model/` — estimador de offset y deriva en Q16.16 sin float, copiando la forma de `tag_testting/src/beacon_sched_core.c`. Observable: timestamps CCP consecutivos (78 ppt). CFO como adquisición rápida. Cubrir coasteo y el wrap de 17.2 s de hi32 con aritmética de diferencia firmada. | 2 | sí |
| A7 | Frame CCP + emisión/recepción entre anclas; plan de medición en hardware para el gate de Fase 2. | 2 | parcial |
| A8 | Blink TDoA en el codec + backhaul de timestamps por MQTT desde las anclas (compilar `net_uplink` en modo slave). | 3 | parcial |
| A9 | Portar `pos_solver` / `pos_residual` / `pos_ekf` al gateway con modelo de diferencia de rangos. Host tests propios. | 3 | sí |
| A10 | Frames `0xEC` CONFIG_SET / `0xED` CONFIG_ACK + bit de config pendiente en el beacon. | 4 | sí |
| A11 | `POS_JSON_ZONE_NAME` y `UWB_FRAME_PANID` a NVS vía `uwb_store` / `net_store`. | 4 | sí |

### 8.2 Agente-TAG — repo `tag_testting`

Dueño de la FSM del tag, del ahorro de energía y del consumo del downlink.

| # | Tarea | Fase | Host-testeable |
|---|---|---|---|
| T1 | **Corregir la divergencia SFD (§2.8):** `phy_config.c:170` de `(1024 + 1 + 8 - 8)` a `(1024 + 1 + 8 - 32)` = 1001, y arreglar el comentario obsoleto "PAC 8" → PAC 32 en las opciones que usan `DWT_PAC32`. Cambio mínimo y seguro; el PHY queda congelado. | 1 | no (constante) |
| T2 | `uwb_net.c`: `!in_map` deja de significar "asiento reclamado" y pasa a "no me toca, duermo". La pérdida de asiento se detecta por `lease_remaining` + keepalive, que **ya existe y ya es correcto** (`lease_age()`, `uwb_net.c:162`). Quitar el salto a `UWB_ST_SCAN` en `:282` y `:320`. | 1 | sí |
| T3 | Ampliar `tests/uwb_net/`: un tag que pasa muchos superframes fuera del slot map **conserva** su asiento; lo pierde solo por expiración de lease. Pinear `UWB_NET_PROTO_VER` = 3 contra el codec (el test `test_proto_ver_matches_frame_module` ya existe y debe seguir pasando). | 1 | sí |
| T4 | Copiar `uwb_frame_802_15_4z.{c,h}` desde ANCLA tras A4 y correr `tests/uwb_frame/`. Verificar `diff` vacío. | 1 | sí |
| T5 | Revisar la interacción de `UWB_LISTEN_SKIP_CAP` (25) con el scheduler nuevo. Hoy el cap existe porque el lease se pierde al dormir; con asiento separado del horario **la razón del cap desaparece** y los `listen_skip` de diseño (300/75/1) pasan a ser alcanzables. Levantar el cap es cambio de una línea *más* los tests que lo prueben. | 1 | sí |
| T6 | Consumo del downlink: extender la ventana RX cuando el beacon trae el bit de config pendiente, parsear `0xEC`, aplicar, responder `0xED` en CAP. Reusar `beacon_track_core` / `beacon_sched_core` en vez de un temporizador nuevo. | 4 | parcial |
| T7 | Retirar el solver del tag una vez que el gateway resuelva (Fase 3): el tag pasa a emitir blink y ya. `pos_solver` / `pos_ekf` **no se borran** — se quedan como la fuente del porte del gateway y de sus host tests. | 3 | sí |
| T8 | Borrar `pos_dbg.c` según el checklist de remoción de su spec, una vez que el EKF viva en el gateway. | 3 | sí |

### 8.3 Puntos de sincronización entre agentes

| Cuándo | Qué |
|---|---|
| Tras A4 | El Agente-TAG copia el codec (T4). Nada del lado del tag compila contra v3 antes. |
| Antes de flashear Fase 1 | Ambos firmwares al mismo `proto_ver`. Un tag v2 contra gateway v3 queda **sordo en SCAN**, no degradado. |
| Gate de Fase 2 | A7 mide en hardware. Si falla, la Fase 3 no procede y T7/T8 no se ejecutan. |

---

## 9. Plan de la semana (martes 2026-08-25 → viernes 2026-08-28)

| Día | Agente-ANCLA | Agente-TAG |
|---|---|---|
| **Mar** | A1, A2 | T1 |
| **Mié** | A3, A4, A5 | T2, T3, T4 |
| **Jue** | A6 | T5 |
| **Vie** | A7 (frame CCP + plan de medición) y actualizar este spec con lo aprendido en A1/A3/A6 | cerrar host tests de T2/T3/T5 |

**En paralelo, desbloqueado desde hoy porque el PHY quedó congelado:**
calibración de retardo de antena en las tres anclas y el survey `apos`
(`docs/antenna-delay-calibration.md`, `docs/anchor-auto-positioning.md`).

Ninguna tarea de software de esta semana depende de hardware. Los gates de
hardware (§10) **no caben en la semana** y quedan secuenciados después del
congelamiento de PHY — decirlo así es más útil que meterlos y no terminarlos.

---

## 10. Gates de hardware, en orden

Heredados de `CLAUDE.md` y ahora desbloqueados por el congelamiento de PHY.
Ninguno se ha ejecutado.

1. Calibración de retardo de antena: `cal ref` por ancla hasta `|error_mm| < 15`,
   sobrevive `kernel reboot cold`, las tres anclas.
2. Cross-check `cal peer`: `|error_mm| < 30` en todo par.
3. `apos enum` → `apos run` con `missing_pairs:0` → `apos apply`.
4. Las distancias nodo-a-nodo del survey contra **cinta métrica** — en un arreglo
   de 4 anclas resuelto en 3D es la única verificación real de la geometría.
5. `residual` del `0xEA` por debajo de ~0.1 m con `(x, y)` estable entre fixes.
6. **Nuevo, gate de Fase 2:** sincronía sub-ns entre dos anclas sostenida sobre
   CCPs, medida contra una distancia conocida.

---

## 11. Correcciones al registro

- **`spec/2026-06-17-uwb-mac-protocol-contract.md` §1** descarta TDoA porque el
  reloj sub-ns no sería distribuible inalámbricamente. **Falso en este
  hardware** — §4.2 lo cuantifica con los primitivos que ya existen. Es también
  el enfoque estándar de la industria (Sewio: *"Master Anchors broadcast a Sync
  signal"*; Ciholas Bernoulli: sincronía distribuida sin master).
- **Contrato MAC §5.1** afirma que bajar tiers devuelve airtime. **No está
  implementado** (§2.2). §4.1 lo implementa de verdad.
- **`CLAUDE.md`, déficit de TX de ~25 dB:** era mal soldado del PA, **resuelto**;
  la nota ya se actualizó el 2026-08-25. El fix de
  `dwt_setfinegraintxseq(0)` sigue siendo requerido pero no era la causa y su
  contribución nunca se midió por separado — no citarlo como evidencia.
- **Divergencia SFD entre repos** (§2.8): se corrige en T1.
