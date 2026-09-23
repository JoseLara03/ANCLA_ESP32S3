# PLAN DE IMPLEMENTACIÓN — Fix B14: compensación de vuelo master→ancla en el modelo de sync

**Rama:** `fix/b14-flight-compensation` (partir de `diag/tdoa-precision` mergeado a main, o de main + cherry-pick de T6).
**Alcance:** SOLO este fix. Prohibido tocar TDMA/superframe, PHY, B10, o implementar multi-hop (solo hooks de diseño).
**Autoridad:** este documento. Si el código contradice una premisa aquí escrita, PARAR y reportar antes de improvisar.

---

## 0. Contexto (evidencia que motiva el fix)

B14, medido en hardware: el sync CCP convierte los ToA a la base de **transmisión** del gateway. Cada ancla $i$ queda adelantada por su propio vuelo $d_{Mi}/c$, y nada lo resta. Evidencia: sesgo por ancla constante entre dos puntos de prueba (+2.64/+2.62/−0.44 m); posición del gateway predicha desde el sesgo de radio a 228 mm de la cinta; mover el gateway al centro redujo el error 2.141 → 0.634 m. Fracción explicada: 80–90 %.

Álgebra del defecto: el offset entrenado es $b_i = T^{tx}_{ref} - R_i$, pero el correcto es

$$b_i = T^{tx}_{ref} + \tau_i - R_i, \qquad \tau_i = d_{Mi}/c$$

Sin $\tau_i$, cada ToA convertido arrastra $-d_{Mi}/c$ y cada par TDoA arrastra $-(d_{Mi}-d_{Mj})/c$.

**Hechos del sistema que el fix debe respetar (verificados en T0/T1):**
- T0-A: el DW3220 resta `ant_rx` de TODOS los timestamps (CCP incluido) → el retardo RX del esclavo se auto-cancela entre entrenamiento y evaluación. Un cambio de `ant_rx` en esclavo es invisible al solver. Esta invarianza es un **invariante de regresión** del fix.
- El gateway NO aporta observaciones al solver (solo es referencia de tiempo). Su $\tau_0 \equiv 0$ por definición.
- El skew se entrena por diferencias entre CCPs sucesivos → una $\tau_i$ **constante** se cancela en el entrenamiento de skew. El fix toca SOLO el término de fase/offset.
- Maquinaria T6 (gate `SYNC_BASELINE_USEFUL`, sigma dinámica, contadores) ya existe y no debe romperse.

---

## 1. Objetivo y predicción verificable

Convertir los ToA a la base de tiempo **verdadera** del tier-0 añadiendo $\tau_i$ en el entrenamiento del offset del modelo de sync.

| Métrica | Antes | Predicción post-fix | Gate |
|---|---|---|---|
| Error punto de prueba 1 | 2.141 m | ~0.24 m | ≤ 0.35 m |
| Error punto de prueba 2 | 1.885 m | ~0.24 m | ≤ 0.35 m |
| Sesgo constante por ancla | +2.6 / −0.44 m | \|sesgo\| < 0.3 m | sí |
| `rx_cal.py` (T3) | no corrida | $b \approx 0$ (ver §9) | \|b\| < 0.5 ns/ancla |
| Sonda T0 (repetir) | invarianza | invarianza intacta | sí |

Si la Fase A no cumple el gate, el modelo de B14 está incompleto → PARAR y reportar. No pasar a Fase B.

---

## 2. Requerimientos

### Funcionales

- **FR-1** Toda ancla esclava convierte ToA a la base verdadera del tier-0 con error sistemático residual ≤ 0.5 ns.
- **FR-2** $\tau_i$ es un parámetro por ancla, en DTU con signo (`int32`; regla: $\tau_{dtu} \approx 213 \times d_{m}$), persistido en flash, editable por consola.
- **FR-3** Fuentes de $\tau_i$ con prioridad: `TWR` (si fresco) > `SURVEY` > `LEGACY` ($\tau=0$, comportamiento actual con B14). Modo seleccionable: `LEGACY | SURVEY | TWR | AUTO`.
- **FR-4** $\tau_i$ se aplica en UN solo punto: el entrenamiento del offset del modelo de sync. Ningún consumidor aguas abajo (solver, logs, rx_cal) cambia.
- **FR-5** Al actualizar $\tau_i$ (consola o TWR), el offset publicado se corrige **en feed-forward** ($b_i \mathrel{+}= \Delta\tau_i$) en el mismo instante. PROHIBIDO dejar que el filtro de offset re-converja lentamente ante un cambio de $\tau$ — eso introduciría un transitorio de segundos.
- **FR-6** (Fase B) Medición on-air de $\tau_i$ por SS-TWR master→ancla, iniciada por el master, en slots reservados del superframe, round-robin entre anclas, tasa ≥ 0.5 Hz por ancla.
- **FR-7** (Fase B) El resultado TWR viaja del master a la ancla **piggybacked en el CCP** (payload v2, versionado). La ancla lo filtra (ver FR-8) antes de adoptarlo.
- **FR-8** (Fase B) Filtro de $\tau_{twr}$: EMA lento ($\alpha \approx 0.05$), gate NLOS sobre los diagnósticos del intercambio TWR, y **detección de salto**: si $|\tau_{nuevo} - \tau_{usado}| > 2$ ns sostenido por ≥ 5 mediciones consecutivas → transición controlada + evento de log (implica movimiento físico del gateway o de la ancla, o cambio de padre de sync en el futuro multi-hop).
- **FR-9** Freshness: si el TWR de sync lleva > 60 s sin actualizar, la fuente degrada a `SURVEY` (o `LEGACY` si no hay survey) y se loguea.
- **FR-10** Telemetría por ancla en el log periódico existente: `tau_used_dtu, tau_source, tau_twr_dtu, tau_twr_age_s, twr_quality, tau_jump_events`.
- **FR-11** Rollback: modo `LEGACY` debe reproducir el comportamiento actual bit a bit (A/B en el mismo binario).

### No funcionales

- **NFR-1** Costo de airtime del TWR de sync < 1 % del superframe (2 tramas ≈ 3 ms por intercambio; con 4 anclas en round-robin a superframe de 200 ms: ~0.4 %).
- **NFR-2** Cero `float` en caminos de timestamp; toda resta 40-bit wrap-safe (helpers existentes `sdelta40`/two-step fold).
- **NFR-3** Persistencia: $\tau_{survey}$ en flash (config); $\tau_{twr}$ en RAM con persistencia opcional solo si cambió > 1 ns desde la última escritura (desgaste de flash).
- **NFR-4** Tests unitarios en CI para toda la lógica nueva (§6.1).
- **NFR-5** Sin regresiones: TWR tag↔ancla existente intacto (sumas de antena preservadas — este fix NO toca `ant_tx`/`ant_rx`), enumeración/stagger intacto, T6 intacto.

---

## 3. Decisiones de diseño y su justificación (incluye el futuro multi-hop)

**D1 — El fix vive en el modelo de sync de la ancla, no en el solver ni en `rx_delay`.**
`rx_delay` es calibración *por enlace*: si el gateway se mueve hay que recalibrar todo, y en multi-hop sería una calibración por cada par (padre, hijo) — combinatoria e inmantenible. Solver-side requeriría que el solver conozca la ruta de sync de cada ancla — estado frágil. Anchor-side arregla en la fuente: **todo consumidor hereda una base limpia**, incluidos hops y failover futuros.

**D2 — $\tau_i$ es un parámetro NUEVO, separado de `ant_delay`.**
`ant_delay` es propiedad de la *placa* (viaja con ella); $\tau_i$ es propiedad del *despliegue/enlace*. Mezclarlos: (a) rompería las sumas TWR ya calibradas, (b) rompería la invarianza T0-A, (c) haría la calibración de antena dependiente de dónde está el master.

**D3 — Las anclas NO conocen coordenadas; $\tau_i$ se configura/mide en DTU.**
En Fase A, una herramienta host calcula $\tau_i$ desde la survey y se empuja por consola. En Fase B, el TWR lo mide directamente en DTU. Mantener a las anclas *position-agnostic* es una decisión de escalabilidad: no hay que distribuir ni versionar coordenadas en el fleet, y en multi-hop cada enlace se auto-mide sin saber dónde está nada.

**D4 — Fase A con survey, Fase B con TWR. Por qué TWR es la fuente correcta a largo plazo:**
El TWR mide el retardo de extremo a extremo *en el dominio de timestamps*, que es exactamente la cantidad que sesga $b_i$. Sus términos de error se comportan bien:
- Residual de antena del **master** ($e_{tx,M}$): entra igual en todos los $\tau_i$ → **common-mode, se cancela en las diferencias TDoA**.
- Residual RX del **esclavo**: auto-cancelado por T0-A.
- Residual neto: $\approx s_i/2$ (mitad del split TX/RX del esclavo), estimado < 0.5 ns. Palanca futura: la calibración de split T2 (banco, reciprocidad) — que además vuelve a ser válida tal como se derivó originalmente una vez corregido B14.
Conclusión: TWR da $\sigma_\tau \approx 0.3$ ns sin calibración adicional; survey da ~23 ps pero no se auto-mantiene. Survey = bootstrap; TWR = régimen permanente.

**D5 — El master inicia el TWR y distribuye el resultado en el CCP.**
Mantiene la autoridad del schedule centralizada (coherente con la arquitectura TDMA actual), la ancla no necesita slot propio de transmisión (solo responde con offset fijo), y el CCP ya existe como canal periódico master→anclas.

**D6 — Feed-forward en cambios de $\tau$ (FR-5).**
Un cambio de $\tau$ es información cierta, no ruido: aplicarlo instantáneamente. Dejarlo al filtro convertiría cada actualización en un transitorio de posición.

**D7 — Payload CCP versionado.**
Permite flashing rodante del fleet sin mezclar formatos. Byte de versión; receptores v1 ignoran campos v2.

**D8 — Reglas de diseño multi-hop (HOOKS SOLAMENTE, no implementar):**
1. Cada nodo compensa **su propio enlace** con su padre de sync.
2. Un master tier-1 re-transmite tiempo **ya compensado** (referenciado a tier-0, no a su transmisión cruda).
3. Re-parenting = re-medir el nuevo enlace (TWR con el nuevo padre) + gate de baseline T6 durante la convergencia.
Con estas reglas los sesgos **no se acumulan** por hop; solo el ruido, en cuadratura: $\sigma_{total} \approx \sqrt{N_{hops}} \times 0.33$ ns (3 hops ≈ 0.6 ns ≈ 17 cm). Sin ellas: $\text{bias}_i = \sum_{hops} d_h/c$ — metros por hop, saltos discontinuos en failover, y fronteras de celda con discontinuidad de posición.
Estructuras a dejar preparadas: `sync_parent_id`, `tau` ya es por-enlace (eso ES el hook), campo de tier en CCP v2 (reservado, default 0).

**D9 — (Futuro, no implementar) CCP two-way estilo PTP.**
Intercambio simétrico que cancela el vuelo sin survey ni TWR dedicado: $\text{offset} = [(t_2-t_1)-(t_4-t_3)]/2$. Costos: 2× airtime de sync y residual = asimetría de split/2 → requiere T2. Candidato para la fase de escalado si el airtime sobra; con el PHY de preámbulo largo actual, D4 llega antes con menos riesgo.

---

## 4. Diseño detallado

### 4.1 Punto de aplicación (Fase A)

En el modelo de sync (según audit: `sync_model.c`, entrenamiento del offset en la recepción del CCP; conversión/extrapolación ~`sync_model.c:176`):

```c
/* Entrenamiento del offset, por CCP: */
b_meas = (int64_t)T_tx_ref + sync.tau_dtu - (int64_t)R_i;   /* + tau_dtu es EL fix */

/* La conversión de blinks NO cambia: usa b como antes.        */
/* El entrenamiento de skew NO cambia: tau es constante y      */
/* se cancela en las diferencias entre CCPs. Test que lo prueba: §6.1-T4 */

/* FR-5: cuando tau cambia: */
void sync_set_tau(sync_state_t *S, int32_t new_tau_dtu) {
    int64_t d = (int64_t)new_tau_dtu - S->tau_dtu;
    S->offset_dt += (double)d;        /* feed-forward, sin re-convergencia */
    S->tau_dtu   = new_tau_dtu;
    log_tau_change(S, d);
}
```

Ancla master (raíz): `tau_dtu = 0` siempre.

### 4.2 Configuración y consola

```
sync tau mode <legacy|survey|twr|auto>     ; default: survey tras Fase A, auto tras Fase B
sync tau set <dtu>                          ; escribe tau_survey, persiste
sync tau show                               ; tau_used, fuente, edad twr, jumps
```

Config persistida: `{ tau_survey_dtu: i32, mode: u8 }`. Runtime: `{ tau_twr_dtu: i32, tau_twr_age_s: u32, twr_quality: u8, jump_events: u32 }`.

Herramienta host: extender el tooling de survey para emitir `sync tau set` por ancla desde las coordenadas (distancia 3D master→ancla × 213 DTU/m). La survey actual tiene rms 7 mm → error en τ ≈ 23 ps, despreciable. Implementado en `tools/sync_tau_from_survey.py`.

**Adenda 2026-09-23 — alternativa on-air, evita medir la posición del gateway a mano.** En campo, medir con cinta la posición del gateway en el MISMO frame del survey resultó poco práctico (anclas montadas en techo/paredes lejanas, el gateway no es parte del survey en sí) y propenso a error humano — un desalineamiento de ~0.8 m entre la posición usada para `sync_tau_from_survey.py` y la real fue justo lo que se detectó en la primera corrida de campo. `src/gw_selftau.{c,h}` (ver su entrada en el Layout de CLAUDE.md) evita esa conversión de coordenadas por completo: el gateway rangea DIRECTO a cada ancla ya posicionada por SS-TWR (mismo `ss_initiator.c` que ya usa el survey ancla↔ancla, mismo responder WAVE que cada ancla ya contesta — sin frame nuevo, sin protocolo nuevo) y calcula `tau_dtu` de la distancia medida, sin geometría de por medio. Consola: `sync tau measure <id> [n]` / `sync tau result`. Sigue siendo Fase A (manual, one-shot, el operador copia el número a mano) — NO es la Fase B de este plan (FR-6..FR-9: round-robin, filtro, CCP v2, auto-tracking). El operador elige la fuente que le convenga: `sync_tau_from_survey.py` (si tiene cómo medir bien la posición del gateway) o `sync tau measure` (si no). **No corrido en hardware.**

### 4.3 TWR de sync (Fase B)

- **Slots:** 2 slots reservados por superframe (poll + response). Round-robin: superframe $k$ mide la ancla $k \bmod N_{anchors}$. Con 4 anclas y superframe 200 ms → 1.25 Hz por ancla.
- **Roles:** master = initiator (reutiliza stack SS-TWR existente); ancla = responder (reutiliza el responder de tags). El master calcula ToF en DTU.
- **Distribución:** CCP v2 añade `{ version: u8, tier: u8 (reservado=0), tau_meas_dtu: i32, tau_meas_anchor: u16, twr_quality: u8 }`. Solo la ancla direccionada adopta la medición.
- **Filtro en la ancla (FR-8):** gate NLOS desde diagnósticos del exchange → EMA $\alpha=0.05$ → jump detection → `sync_set_tau()` (feed-forward).
- **Interacción con B13/T6:** un salto de τ adoptado NO es un re-baseline (el skew sigue válido); solo la fase cambia en feed-forward. Test que lo prueba: §6.1-T6.

### 4.4 Telemetría

Extender el log periódico (L3-equivalente) con los campos de FR-10. Formato CSV, mismas reglas de no-bloqueo que la instrumentación DIAG.

---

## 5. Fases y gates

**Fase A — τ estático desde survey** (FR-1..FR-5, FR-10, FR-11)
Entregables: código + tests §6.1 + tool host + aceptación §6.2-A.
**GATE A:** tabla de §1 cumplida. Si no → PARAR, reportar, no continuar.

**Fase B — τ auto-medido por TWR** (FR-6..FR-9)
Entregables: código + tests §6.1 + aceptación §6.2-B (incluye la prueba del gateway móvil).
**GATE B:** concordancia τ_twr vs τ_survey < 0.5 ns sostenida; prueba del gateway móvil superada.

**Fase C — hooks multi-hop** (D8)
Entregables: solo estructuras + documentación en `docs/sync-multihop-design.md`. Sin lógica de hops.

---

## 6. Tests

### 6.1 Unitarios (CI, obligatorios)

- **T1** Conversión con τ conocido: ToA convertido se desplaza EXACTAMENTE τ_dtu respecto a τ=0.
- **T2** τ en entrenamiento = τ en conversión (no doble aplicación ni omisión): inyectar CCP sintético con vuelo conocido → el ToA de un blink sintético sale en base verdadera ±1 DTU.
- **T3** Wrap-safety: repetir T1/T2 con timestamps cruzando el wrap de 40-bit y con R_i cerca de 0.
- **T4** Invariancia del skew: entrenar skew con τ=A, cambiar a τ=B vía `sync_set_tau` → el skew estimado no cambia (la constante se cancela en diferencias).
- **T5** Feed-forward: cambio de τ se refleja en el offset publicado en la MISMA llamada (sin esperar al siguiente CCP).
- **T6** Cambio de τ NO dispara re-baseline ni invalida `SYNC_BASELINE_USEFUL`.
- **T7** Prioridad de fuentes: TWR fresco > survey > legacy; expiración a 60 s degrada correctamente.
- **T8** Filtro TWR: ráfaga de mediciones NLOS es rechazada; salto sostenido ≥5 muestras sí transita y loguea `tau_jump`.
- **T9** Round-trip de persistencia de config.
- **T10** Modo LEGACY bit-compatible: mismos ToA convertidos que el build pre-fix (golden log).

### 6.2 Aceptación en hardware

**A. Estática (post Fase A):**
1. Tag estático en los dos puntos históricos de prueba. Medir error y sesgo por ancla. Comparar contra tabla §1.
2. **Test negativo:** el patrón de sesgo por distancia-al-gateway debe DESAPARECER. Re-correr el ajuste "predecir la posición del gateway desde el sesgo de radio": ahora debe fallar sin solución consistente (el término $d_{Mk}-d_{M0}$ ya no existe). Si aún predice el gateway → el fix no está aplicándose en el camino real.
3. Repetir la sonda T0 (+400 DTU en `ant_rx` de un esclavo): la posición debe seguir invariante.
4. Correr `rx_cal.py`: expectativa $|b| < 0.5$ ns por ancla. Si algún $|b|$ es 0.1–0.5 ns sistemático, NO aplicar ciegamente: es candidato a group-delay angular (§9), reportar valores.
5. Regresión TWR tag↔ancla: mismas distancias que pre-fix (±10 cm).
6. Regresión T6: reboot del gateway → transitorio acotado como en la aceptación previa de T6; contadores íntegros.

**B. Auto-medición (post Fase B):**
1. Concordancia τ_twr vs τ_survey: < 0.5 ns por ancla, sostenido 10 min.
2. **Prueba del gateway móvil (la que demuestra el requerimiento original):** con modo `AUTO`, mover físicamente el gateway ~0.5 m. Esperado: `tau_jump` se loguea, τ converge al nuevo valor en < 60 s, y el error de posición NO crece (> 0.3 m). Repetir con modo `SURVEY` (τ congelado): el error DEBE crecer (~la proyección del movimiento sobre las diferencias de distancia). Este par de corridas demuestra que el mecanismo existe y que la auto-medición lo neutraliza.
3. Matar el enlace TWR de sync (atenuador/absorbente): a los 60 s la fuente degrada a SURVEY y el error se mantiene acotado.

---

## 7. Rollback

`sync tau mode legacy` en caliente (sin reflash) restaura el comportamiento actual. Si el bug está en el filtro TWR: modo `survey`. Nada en este plan toca la calibración de antena existente, así que el TWR de tags no puede degradarse por rollback.

---

## 8. Fuera de alcance (explícito)

- TDMA/superframe (el cambio 200→100 ms sigue pospuesto).
- B10 (gating NLOS de blinks en el solver) — solo se gatea el TWR de sync, no los blinks.
- Multi-hop real (retransmisión tier-1, re-parenting) — solo hooks D8.
- Calibración de split T2.
- Cambios de PHY o de data rate.

---

## 9. Residuos conocidos post-fix (documentar, no perseguir)

1. **$s_i/2$** (mitad del split del esclavo) cuando τ viene de TWR — palanca futura: T2.
2. **Group delay dependiente del ángulo de llegada**: el CCP llega desde la dirección del master y los blinks desde otras; diferencia típica 0.1–0.5 ns. Se manifestará como $b \neq 0$ pequeño y sistemático en rx_cal. Es física de antena, no bug.
3. **NLOS en el camino del CCP** (correlacionado, no promediable): mitigación = siting/elevación del master; detección = gate de innovación ya existente en el sync model.
4. Jitter de sync medido (516 ps fleet): ruido, promedia en el tracker de posición.

---

## 10. Reporte de entrega (plantilla)

```
Fase A: [código OK / tests CI pass] 
  Aceptación: punto1 [m], punto2 [m], sesgos por ancla [m],
              test negativo gateway [pass/fail], T0 repeat [pass/fail],
              rx_cal b por ancla [ns], regresión TWR [pass/fail], T6 [pass/fail]
  GATE A: [PASS/FAIL + evidencia]
Fase B: τ_twr vs τ_survey [ns/ancla], gateway-móvil AUTO [m] vs SURVEY [m],
        degradación por pérdida de enlace [pass/fail]
  GATE B: [PASS/FAIL]
Fase C: doc + estructuras [links]
Desviaciones del plan: [ninguna / lista con justificación]
```
