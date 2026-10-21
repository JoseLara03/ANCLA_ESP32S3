# Puesta en marcha de anclas sin consola (`apos assign`)

Procedimiento de operador para integrar anclas recién salidas de fábrica a una
celda **sin conectarlas al shell**. Diseño:
`docs/superpowers/specs/2026-10-21-anchor-eui-identity-design.md`. Plan y
estado: `docs/superpowers/plans/2026-10-21-anchor-eui-identity.md`.

**Estado:** implementado y probado con pruebas host. **Ningún paso de este
documento se ha corrido en hardware** (gates H1-H8 del spec: `NO EJECUTADO`).

---

## 0. Qué hay que saber antes

- **La identidad de un ancla es su EUI**: 16 dígitos hex que son **la MAC de
  la etiqueta del módulo seguida de `0000`** (`anchor show`, `apos addr` y el
  banner de arranque la imprimen). El `anchor id` (0..3) es solo un handle de
  radio que reparte el gateway. Lo puede cambiar con `apos assign`.
- **Un ancla de fábrica arranca sin id (UNASSIGNED)**, en la dirección
  compartida `0x00FF`. En ese estado no contesta TWR (DISCOVERY/WAVE), no se
  deja rangear y no acepta coordenadas. Sí contesta la enumeración del survey y
  sigue publicando sus observaciones BLINK (con `"a":255` y su EUI).
- **El WiFi sigue siendo por consola**: `net ssid`/`net pass` y la clave de
  comandos (`net cmdkey`) se graban una vez en banco. Esto no lo resuelve
  `apos assign` (ver §6 del spec).

---

## 1. Flujo normal

En la consola del **gateway**:

```
apos assign
```

1. El gateway enumera (3 rondas de `SURVEY_BEGIN`).
2. Calcula un plan, que imprime como líneas `apos_assign_plan` con la acción
   de cada ancla:
   - `keep`: ya tiene la dirección que le corresponde;
   - `adopt`: se queda con la que ya usa;
   - `assign`: recibe una dirección nueva.
3. Manda un `SETADDR` a cada ancla que lo necesita. Cada una guarda su id, lo
   confirma y **se reinicia** sola.
4. Guarda el mapa EUI → dirección (`apos/addrmap`).
5. Re-enumera hasta que cada ancla reasignada contesta desde su dirección
   nueva, con un plazo de 30 s. El arranque de un ancla tarda ~5 s si el WiFi
   levanta bien y 10-20 s más si el driver se cuelga; el lazo UWB no espera al
   WiFi, pero eso todavía no está medido (gate H8).
6. Cierra con una línea `apos_assign_done`. **`"converged":1` es lo que hay
   que ver.**

Después:

```
apos addr                      # quién es quién: eui, addr, id, seen
apos gauge origin=<id> xaxis=<id> plane=<id> [up=<id>]
apos run
apos apply
```

Usa `apos addr` y la MAC de la etiqueta para relacionar cada id con su caja
física **antes** de elegir el gauge.

**Si `apos assign` movió alguna dirección, el gauge se borra** (aparece
`"addresses changed — the gauge was cleared"`): el gauge se guarda como
direcciones y podría estar apuntando a otra caja. Vuelve a fijarlo.

**`apos run` se niega a arrancar** si encuentra un ancla sin asignar o dos
anclas en la misma dirección (`apos_needs_assign` por cada una, más el error).
Nunca re-direcciona por su cuenta.

---

## 2. Flota que ya estaba numerada a mano

Un `apos assign` sobre anclas que ya tienen ids distintos las **adopta**: no
manda ningún frame y no reinicia ninguna placa. Solo aprende el mapa. Es lo
primero que hay que correr después de actualizar el firmware de un sitio
existente (gate H3).

**Ojo con la migración:** una placa que **nunca guardó** un `anchor id` (estaba
en 0 por defecto) arranca UNASSIGNED con este firmware. Deja de contestar TWR
hasta el primer `apos assign`, que normalmente le da el id 0 si está libre.
Revisa con `anchor show` qué placas tenían id guardado antes de flashear.

---

## 3. Reemplazar o retirar un ancla

El mapa es una **reserva permanente**: un ancla apagada el día del assign
conserva su id, y nadie más lo recibe.

- **Para reemplazar una placa**:
  ```
  apos addr                    # localiza el EUI de la placa vieja
  apos forget <EUI16>
  apos assign                  # la nueva recibe la dirección liberada
  ```
- **Si el mapa no cabe** (más anclas que direcciones o que entradas): el
  assign se niega entero (`not every anchor fits`) y no cambia nada. Libera
  reservas viejas con `apos forget`.
- **Si el mapa guardado no se puede leer** (por ejemplo, tras un rollback de
  firmware): `apos assign` se niega para no sobrescribirlo. Si decides
  descartarlo, usa `apos addr clear`.

---

## 4. Lectura de errores

| línea | significa | qué hacer |
|---|---|---|
| `... never acknowledged SETADDR after 4 attempts` | firmware anterior a esta función, o fuera de alcance | flashear o acercar; esa placa queda sin reserva |
| `... answered SETADDR with status 1` | no pudo guardar en NVS | revisar la placa; queda en su dirección vieja |
| `apos_assign_unconverged` | la placa aceptó, pero no reapareció en su dirección nueva antes de 30 s | revisar si arrancó; volver a correr `apos assign` |
| `address map NOT persisted` | falló la escritura del mapa en el gateway | volver a correr; las placas que ya cambiaron se adoptan |

Un ancla sin asignar también se nota en su propia consola:
`DISCOVERY refused — this board has no assigned address`.

---

## 5. MQTT después del cambio

- **Status, comandos y acks** están en `uwb/anchor/{status,cmd,ack}/<zone>/<EUI16>`
  (antes era `<id>`). El gateway sigue en `.../gw`.
- **Los retained viejos** `uwb/anchor/status/<zone>/0..3` quedan en el broker.
  Bórralos publicando un retained vacío en cada uno, por ejemplo:
  ```bash
  mosquitto_pub -h HOST -t "uwb/anchor/status/852541/0" -r -n
  ```
- **El client id** pasó a `anc-<EUI16>` / `gw-<EUI16>`. Antes, dos anclas con
  el mismo id (o dos gateways) se desconectaban mutuamente del broker en un
  ciclo.
- **Comandos firmados**, desde el edge:
  `python -I edge/tools/nodecmd.py <EUI16> status`.
- **Orden de despliegue** (cambio directo, sin publicar en ambos esquemas):
  edge → gateway → anclas.
