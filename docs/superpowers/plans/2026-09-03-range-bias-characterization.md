# Caracterización del range bias SS-TWR (dependiente de distancia)

> **For agentic workers:** this is a bench-measurement plan, not a code plan.
> No task here should touch firmware until Task 3 makes an explicit decision
> to. Steps use checkbox (`- [ ]`) syntax for tracking.

**Origen:** sesión 2026-09-03, campaña de calibración RX antenna-delay
(`docs/rx-antenna-delay-calibration.md`). El fit de `tools/rx_cal.py` sobre
tres marcas dio `INCONSISTENT` en dos anclas de cuatro, con spreads de
1.4-1.6 m entre marcas — muy por encima del propio offset que se intentaba
medir. El primer diagnóstico (geometría del survey sin validar) se descartó:
el operador reporta que ningún ancla se movió, y que la discrepancia
survey-vs-láser ya se había notado antes, en **cientos de calibraciones**
contra un DWM3001CDK ya calibrado — con un patrón consistente: **el error
entre la distancia real y la medida crece o decrece con la distancia**.

**Hipótesis de trabajo:** esto no es un problema de antenna delay (que es un
offset CONSTANTE, independiente de la distancia) ni de geometría de survey.
Coincide con el efecto documentado por Qorvo para el DW3000/DW1000 —
"range bias" dependiente del nivel de señal recibida en la detección del
leading-edge del CIR, que se manifiesta como dependiente de la distancia
porque la potencia recibida cae con ella. Se revisó el driver vendorizado
(`modules/dw3000-decadriver`) y **no implementa ninguna tabla de corrección
de range bias** — solo hay bias trims de RF/PLL/LDO, nada relacionado a rango
vs nivel de señal. Esto nunca se ha corregido en este proyecto.

**Por qué esto es más fundamental que el antenna delay o la calibración RX,
y las bloquea a ambas:** `cal ref` calibra el antenna delay contra el
DWM3001CDK a **una sola distancia de referencia**, así que absorbe el range
bias de ESA distancia dentro del `ant_tx` calculado — y a cualquier otra
distancia el bias real es distinto y queda sin corregir. `apos run` resuelve
la geometría del survey a partir de distancias SS-TWR ancla-a-ancla, así que
**hereda el mismo sesgo dependiente de distancia** en cada arista, con una
magnitud distinta según la longitud de cada arista. Y `rx_cal.py` usa esa
geometría del survey como "verdad" para calcular la distancia esperada a
cada ancla desde cada marca — si la geometría está distorsionada por el
range bias, el residual que el script atribuye a "RX delay" es en realidad
una mezcla de RX delay real más error de geometría heredado. Ninguna de las
dos campañas produce un número confiable mientras esto no se mida y, si
aplica, se corrija.

**Alcance de este plan:** solo medir y caracterizar la curva error-vs-distancia.
No incluye implementar una corrección en firmware (eso es una decisión de
Task 3, y una tarea de código aparte si se decide seguir adelante), ni repetir
el survey o la campaña RX (ambas quedan bloqueadas hasta que esto se
resuelva).

---

## Task 1: Captura de rangos SS-TWR anchor-a-anchor a múltiples distancias

- [ ] Elegir un par de anclas ya calibradas con `cal ref` (cualquiera; el
      efecto buscado es del PHY, no específico de un par).
- [ ] Marcar con cinta métrica (o láser) al menos **6-8 distancias** entre esas
      dos anclas, cubriendo el rango real de despliegue — desde la separación
      mínima esperada (~0.5-1 m) hasta la máxima (~3-5 m, según el sitio).
      Más puntos que en la campaña RX a propósito: aquí se busca la FORMA de
      una curva, no solo dos extremos.
- [ ] Para cada distancia: reposicionar físicamente una de las anclas (o usar
      dos anclas portátiles), y correr varias repeticiones de `cal peer` (o el
      mecanismo de ranging anchor-a-anchor que ya expone la consola) para
      obtener una media estable por distancia — no una sola lectura, el ruido
      de una sola medición no debe confundirse con el bias sistemático.
- [ ] Registrar por cada distancia: la distancia real (láser), la distancia
      media reportada, el `error_mm` resultante, y el nivel de señal recibida
      si el firmware lo expone (`quality`/`ipatovAccumCount` vía el diagnóstico
      CIR — ya existe en el driver, ver `dwt_configciadiag()`).
- [ ] Repetir con un segundo par de anclas si el tiempo lo permite, como
      chequeo de que el efecto es del PHY y no idiosincrático de un board.

**Nota de disciplina de banco:** no re-flashear ni re-calibrar nada entre
mediciones de esta tarea — el objetivo es aislar el bias del PHY, no
introducir una variable de calibración a medio camino.

## Task 2: Analizar la curva error-vs-distancia

- [ ] Graficar `error_mm` (medida − real) contra la distancia real, por par.
- [ ] Comparar la forma contra la curva de range-bias que Qorvo publica para
      el canal/PRF/PLEN que usa este proyecto (canal 5, PLEN_1024, PAC32, código
      9, 850 kbps, SFD_IEEE_4Z — ver `src/uwb_phy.h`). Si Qorvo no publica esa
      combinación exacta, usar la más cercana disponible como referencia de
      forma, no de magnitud exacta.
- [ ] Si la curva correlaciona con el nivel de señal recibida (`quality`) en
      vez de con la distancia directamente, decirlo explícitamente — son la
      misma variable en la práctica (la señal cae con la distancia) pero la
      distinción importa para decidir qué corregir: una tabla contra `quality`
      generaliza mejor que una contra distancia nominal, que este firmware ni
      siquiera conoce en producción.
- [ ] Registrar el hallazgo con los números reales, sea positivo o negativo —
      si la curva NO muestra un patrón sistemático claro (por ejemplo, si el
      error resulta dominado por ruido en vez de por una tendencia), es un
      hallazgo negativo tan válido como uno positivo, y cierra esta hipótesis
      en vez de la geometría del survey.

## Task 3: Decisión — corregir o no, y cómo

- [ ] Con la curva en mano, decidir: (a) el efecto es despreciable frente al
      objetivo de exactitud (~45 cm) y no amerita corrección, (b) el efecto es
      significativo y se corrige con una tabla/polinomio aplicado antes de
      alimentar `apos_geom` y `rx_cal.py`, o (c) el efecto es significativo pero
      su magnitud depende de variables que este firmware no mide hoy (potencia
      de transmisión real, ganancia de antena por board), y hace falta más
      instrumentación antes de poder corregirlo.
- [ ] Si la decisión es (b): abrir una tarea de código separada — no forma
      parte de este plan — para aplicar la corrección en el punto correcto de
      la cadena (antes de que `apos_geom` resuelva la geometría, y antes de que
      `rx_cal.py` calcule la distancia esperada).
- [ ] Documentar la decisión en `CLAUDE.md`, con los números de esta
      caracterización, para que no se repita el diagnóstico incorrecto
      ("el survey está mal medido") la próxima vez que alguien vea una
      discrepancia survey-vs-láser.

## Solo después de cerrar este plan

- Re-validar (o re-correr) el survey `apos run`, ahora con el range bias
  entendido y, si aplica, corregido.
- Repetir la campaña de calibración RX (`docs/rx-antenna-delay-calibration.md`)
  con una geometría de survey en la que se pueda confiar.
