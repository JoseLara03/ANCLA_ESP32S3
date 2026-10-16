# B14: survey con gateway, compensación estática y preparación para multi-hop

Fecha: 2026-10-12.
Estado: plan de implementación actualizado con decisiones D1–D3 confirmadas por el operador.

**Actualización 2026-10-12 (implementación):** T0–T8 implementadas en la rama `codex/b14-static-survey` (desde `da3bfab`), con pruebas host y compilación de ambas imágenes; **gates de hardware de la sección 7: NO EJECUTADO**. Registro as-built en `docs/sync-multihop-design.md` y `docs/static-survey-install.md`. Desviaciones respecto al texto de este plan, todas deliberadas y justificadas allí:

1. **El enlace gateway↔ancla se mide en un solo sentido (gateway → ancla).** El sentido inverso exige una ruta de respuesta con TX retardado en el loop que posee el beacon, y el error de antena en SS-TWR es exactamente recíproco, así que el segundo sentido no cancelaría nada. 10 aristas frente a 9 parámetros libres: la redundancia no cambia. Por eso las "20 pares dirigidos" de §4.1/§5-T3 son 16 aquí (12 ancla–ancla + 4 gateway→ancla).
2. El rol "gateway" de un nodo se deriva de `short_addr == 0x0000` en vez de un campo nuevo, para no cambiar el blob del survey (§3: no ampliar la estructura guardada sin migración). Los registros nuevos (`apos/links`, `apos/pending`) sí son versionados.
3. T5 no restaura la generación anterior en las anclas: no se afirma atomicidad distribuida; la recuperación es reaplicar (idempotente, generación monótona nueva) con las salidas suspendidas mientras tanto.
4. Los fuentes de `tau_filter` se conservan sin compilar (el plan pide eliminar la *dependencia operativa*, no el historial).
Base técnica revisada: `fix/b14-flight-compensation`, commit `da3bfab`.
El documento se redacta en el checkout actual `feat/rtls-scale-tdoa`; esa rama antigua NO es la base de implementación.

## 1. Objetivo y decisiones firmes

Incluir el gateway en el autoposicionamiento, mantener el gauge definido explícitamente por el operador y generar una compensación de vuelo estática para cada enlace de sincronización. Desactivar el TWR automático durante la operación normal. Conservar el CCP periódico, la estimación de deriva y fase, y la corrección B14 en su punto de aplicación actual.

El gateway y las anclas son infraestructura fija. Una modificación física requiere recalibración explícita. La ausencia de movimiento no convierte sus osciladores en relojes sincronizados.

Entregable inicial: una celda de un gateway y cuatro anclas, con survey completo de cinco equipos, cuatro receptores de BLINK y compensación estática persistente. La participación geométrica del gateway NO añade una observación TDoA.

Preparar los datos y contratos para enlaces padre–hijo futuros. No implementar aún retransmisión CCP, selección automática de padre, routing ni survey remoto multi-hop.

## 2. Decisiones confirmadas por el operador

| ID | Decisión | Consecuencia |
|---|---|---|
| D1 | Resolver también las alturas automáticamente mediante survey 3D. | Resolver XYZ de los cinco equipos. No incorporar un nuevo modo de Z introducida manualmente ni sustituir el survey por 2D ante un fallo. |
| D2 | Gauge exclusivamente en las anclas. | `origin`, `xaxis`, `plane` y `up` son cuatro anclas distintas. El gateway se resuelve como quinto nodo y no puede ocupar ninguna referencia del gauge. |
| D3 | Solo survey y mediciones manuales; sin TWR automático en operación. | Retirar el disparo periódico de ranging. No ofrecer una opción para reactivarlo en el firmware nuevo; conservar las órdenes de medición manual. |

Estas decisiones cierran las tres preguntas de alcance. Condición física de instalación: las cuatro anclas del gauge deben formar una geometría 3D no degenerada; `origin`, `xaxis` y `plane` no pueden ser colineales, y `up` debe separarse suficientemente de su plano. Cuatro anclas a la misma altura no satisfacen ese gauge. Un gateway fuera de su plano no sustituye a `up`, porque el operador ha excluido al gateway del gauge. Ante geometría insuficiente se rechaza el candidato con diagnóstico; no se inventa altura ni se modifica el gauge automáticamente.

## 3. Hechos del código que condicionan el cambio

- `gw_selftau.c`: `auto_enabled = true` al arrancar. Cambiar solo el valor por defecto no resuelve la migración de anclas en modo TWR/AUTO ni garantiza un `tau_survey` válido.
- `apos_gw.c`: excluye `0x0000`; cero también significa gauge no configurado. Permitir el gateway como nodo local del survey, manteniendo su prohibición en el gauge. No confundir dirección cero válida en la tabla con referencia no configurada.
- `apos_node.c`: rechaza explícitamente RANGE_CMD hacia el gateway. `uwb_gateway.c` debe adquirir una ruta de respuesta de survey si se miden ambas direcciones.
- `gw_selftau` ya inicia SS-TWR gateway→ancla. Reutilizar la primitiva, no crear otro protocolo de ranging sin necesidad.
- `apos_geom.h`: admite ocho nodos, suficientes para cinco equipos. No implica que todos los demás arrays o campos soporten más anclas.
- `apos_store`: persiste un blob cuya compatibilidad se comprueba por tamaño, sin una versión explícita de esquema. No ampliar la estructura guardada sin migración.
- `tdoa_gw_step()`: obtiene el número esperado de observaciones de `survey.n_nodes`. Con cinco equipos pero cuatro receptores, ese criterio deja de ser correcto.
- `pos_json` y la referencia geográfica asumen relaciones con el origen del survey. Conservar el ancla de origen y su asociación geográfica aunque se inserte el gateway en la tabla de nodos.
- `check_twr_freshness()` degrada el origen dinámico después de 60 s; esa caducidad NO debe aplicarse a una calibración estática de instalación.

No repetir como hecho los comentarios históricos que atribuyen a la reciprocidad cancelación del sesgo de antena: la comparación de direcciones ayuda con error residual de reloj; no valida la escala absoluta ni elimina el sesgo común de antena.

## 4. Contrato funcional

### 4.1 Survey y gauge

1. El operador declara un gauge 3D completo antes de `apos run`: `apos gauge origin=<ancla> xaxis=<ancla> plane=<ancla> up=<ancla>`. Las cuatro referencias son distintas y ninguna puede ser el gateway. Omitir `up` no satisface este flujo de instalación.
2. El gateway se incorpora localmente con su EUI-64 real; no se enumera ni se envía comandos a sí mismo por radio.
3. Se enumeran las anclas y se detectan duplicados de EUI/dirección.
4. Se miden pares en ambas direcciones, incluido gateway↔ancla, reutilizando SS-TWR. Cinco nodos completos suponen 20 pares dirigidos, con el número configurado de muestras por par.
5. Se conservan mediciones originales, dispersión y número de muestras; se resuelve la geometría y se reportan calidad, aristas faltantes y ambigüedad.
6. `apos apply` instala coordenadas y compensaciones como una misma generación de configuración. `apos run` no modifica la configuración activa.

El gauge fija un sistema de coordenadas cuyo origen es un ancla; no corrige por sí solo un rango sesgado. Una instalación incompleta o geométricamente ambigua no puede convertirse silenciosamente en un survey válido. El modo 2D histórico puede mantenerse para compatibilidad, pero no permite aceptar ni aplicar la instalación 3D definida por este plan.

### 4.2 Alturas y distancia física

La fuente propuesta de compensación es la geometría aceptada:

`tau(parent, child) = round(distance_3d(parent, child) / c / DTU_seconds)`.

Usar constantes físicas y conversión coherentes con B14. El cálculo geométrico puede usar float fuera del camino crítico; guardar y aplicar `tau` en DTU enteros. Mantener las operaciones temporales de 40 bits wrap-safe.

Las distancias TWR medidas son evidencia para comprobar la geometría, no una fuente dinámica que reescriba `tau` en ejecución. Registrar la discrepancia por enlace entre distancia medida y distancia geométrica. No elegir automáticamente la que produzca menor residual del tag.

Reutilizar y verificar el solver 3D existente: el gauge fija el origen, el eje X, el plano XY y el signo mediante `up`; el gateway se localiza respecto a las cuatro anclas. Rechazar configuraciones coplanares, casi coplanares, colineales o ambiguas. Evaluar específicamente el tetraedro de las cuatro anclas del gauge: la planarity global de los cinco nodos puede parecer buena por la separación del gateway y ocultar un gauge degenerado. Definir antes de aceptar los resultados un umbral dimensionalmente normalizado de condicionamiento y comprobarlo con ruido representativo; no usar solo un volumen distinto de cero.

Las alturas resueltas son coordenadas Z relativas al plano del gauge, no una medición automática de gravedad o altura sobre el piso. Para que Z sea vertical, montar `origin`, `xaxis` y `plane` en un plano horizontal y `up` fuera de ese plano; documentar qué lado queda como +Z. Si ese plano está inclinado, el solver obtiene un marco inclinado y no debe presentarlo como altura sobre el suelo. Conservar `apos zoff` como traslación y `apos tagz` en el mismo marco; una traslación no corrige una inclinación.

Resolver el survey en 3D NO cambia el solver del tag a 3D: el posicionamiento TDoA actual conserva su modelo de altura de tag configurada. Validar esa altura en el marco resultante durante la puesta en marcha. Cinco nodos con las diez aristas disponibles dejan una sola ecuación redundante en el ajuste 3D (10 distancias frente a 9 grados de libertad); un residual pequeño no basta para validar la instalación.

### 4.3 Datos mínimos

Registro de nodo: EUI-64, dirección actual, rol, coordenadas y participación como receptor de BLINK. Separar identidad de hardware, índice geométrico y dirección de radio.

Registro de enlace estático: EUI de raíz, padre e hijo, `survey_generation`, `tau_dtu`, fuente `survey_static`, estado válido/inválido y métricas de aceptación. La dirección de radio no debe ser la clave persistente.

Persistir también gauge de cuatro anclas, dimensión 3D y coordenadas XYZ resueltas, junto con las referencias de marco existentes. El cero es una coordenada válida y un `tau` cero no debe confundirse con dato ausente: usar validez explícita.

En esta entrega todos los padres son el gateway y el nivel es uno. Reservar las identidades para futuros padres; no activar niveles adicionales.

### 4.4 Aplicación y operación

- Al aplicar, cada ancla obtiene su coordenada y compensación del enlace con la raíz; confirma persistencia y generación.
- Un cambio de `tau` usa el feed-forward de B14, sin doble aplicación y sin modificar calibración TX/RX.
- No arrancar ranging sin una petición de survey o una orden manual. No existe activación del TWR periódico en el firmware nuevo.
- Mantener CCP v2 por compatibilidad: sin auto, publicar el indicador de ausencia de medición nueva. No reducir el payload ni cambiar el PHY en esta entrega.
- Una calibración estática no caduca por tiempo; se invalida al cambiar equipo, padre, geometría o configuración de instalación relevante.
- La validez del reloj SÍ depende de la edad del último CCP válido. Un `tau` correcto no autoriza convertir BLINKs usando sincronización caducada.
- Si falta configuración estática válida, reportar `needs_survey`; no tratar `LEGACY/tau=0` como operación calibrada.

## 5. Secuencia de implementación

### T0. Preparar base y línea de comparación

Crear una rama `codex/b14-static-survey` desde B14, no desde el checkout antiguo. Antes, revisar el stash preservado de B14 y documentar qué cambios independientes contiene; no aplicarlo ni descartarlo automáticamente. Registrar commit base, firmware/configuración de cada equipo, survey vigente y métricas de posiciones antes del cambio.

Aplicar las decisiones D1–D3 ya confirmadas en la sección 2. Registrar la disposición física prevista para el gauge 3D y su convención de altura antes de las pruebas de instalación.

### T1. Modelo de nodos y persistencia versionada

Archivos principales: `apos_table.{c,h}`, `apos_store.{c,h}`, `uwb_config.{c,h}`, `uwb_store.c`.

Añadir rol, identidad de raíz/padre, generación y validez explícita. Crear un formato nuevo con versión y validación de longitudes/campos, sin depender de padding de structs. Cargar datos históricos sin borrarlos y marcar ausencia del enlace estático como pendiente de migración. Conservar el registro previo para recuperación. Rechazar versiones desconocidas y fallos de escritura de forma visible.

Pruebas: carga histórica, round-trip nuevo, truncamiento, versión desconocida, sustitución de placa, cambio de dirección conservando EUI, escritura fallida y reinicio.

### T2. Integrar gateway y gauge en el survey

Archivos: `apos_gw`, `apos_table`, `apos_shell`, `apos_geom`, `apos_store`.

Insertar nodo local exactamente una vez y resolver gauge exclusivamente entre anclas por identidades verificadas. Separar presencia de nodo y selección de gauge; no ampliar la interfaz para permitir gateway como referencia. Validar capacidad antes de admitir la flota; no truncar nodos. Integrar el solve 3D y el gate de condicionamiento del tetraedro de anclas descrito en 4.2. Mantener la asociación explícita de `apos ref` con el origen: cambiar origen/orientación invalida o exige reconfirmar la referencia, no conserva silenciosamente una ubicación geográfica incompatible.

Pruebas: cuatro anclas más gateway; recuperación XYZ sintética con gauge válido; gauge ausente, repetido o no encontrado; rechazo del gateway en cualquiera de las cuatro referencias; `up` ausente; anclas coplanares con gateway fuera del plano; tetraedro casi plano bajo ruido; gateway a ambos lados del plano de origen; cambio de orden de enumeración sin alterar identidades. Conservar pruebas de regresión del modo 2D histórico, sin usarlo como fallback del nuevo flujo.

### T3. Medir gateway↔ancla dentro de la sesión

Archivos: `apos_gw`, `apos_node`, `uwb_gateway`, `anchor_respond`, `ss_initiator`, `gw_selftau`.

Integrar el iniciador existente con la tabla de mediciones y habilitar la respuesta local del gateway solo en una ventana de survey autorizada. Corregir la validación de destino que hoy rechaza `0x0000`, sin alias por truncamiento a 8 bits. Revisar los filtros de poll y el direccionamiento real antes de reutilizar la respuesta.

Resolver el arranque desde fábrica: durante survey deben poder medirse equipos todavía sin posición. Mantener el rechazo de ranging de producción sin posición; la excepción debe quedar limitada a sesión, origen y ventana del survey, con timeout.

Usar máquinas de estados acotadas que preserven deadlines de beacon/CCP. No ejecutar lotes largos dentro del loop del gateway. Survey puede interrumpir la disponibilidad de posiciones durante mantenimiento; no prometer continuidad sin medición. Verificar memoria estática/stack y no colocar matrices grandes en el stack.

Pruebas: ambas direcciones, destino reservado correcto, equipo sin posición, ventana cerrada, sesión antigua, timeout y pérdida de respuesta. Hardware: los 20 pares dirigidos y beacon/CCP sin pérdida de deadlines.

### T4. Generar y validar enlaces estáticos

Crear un módulo puro para derivar enlaces de la geometría aceptada y contrastarlos con los rangos. Registrar error de distancia, dispersión y evidencia de calidad. No reutilizar `twr_quality` provisional como detector NLOS validado.

Gate inicial heredado de B14: contraste externo de compensación dentro de 0.5 ns, aproximadamente 32 DTU o 15 cm de distancia, por enlace. El residual geométrico por sí solo no demuestra ese gate; comprobar con distancias físicas independientes. No prometer exactitud universal del tag a partir de ese umbral.

Pruebas: distancias conocidas, alturas distintas, unidades/redondeo, entrada inválida, generación, padre/raíz incorrectos y ausencia de una arista de comprobación. Añadir vectores que detecten doble compensación.

### T5. Distribuir, aplicar y confirmar configuración

Archivos: `apos_frame`, `apos_node`, `apos_gw`, `sync_shell`, `ccp_slave`, almacenamiento.

Definir una extensión APOS versionada para instalar y confirmar el enlace estático junto con la posición. Especificar bytes, tamaños, identidad/generación, códigos de error, timeout y reintento. Actualizar buffers y presupuestos de aire. No modificar silenciosamente longitudes de subtipos que los firmwares antiguos interpretan de otra manera; una ancla incompatible debe producir un error explícito.

El candidato permanece separado del survey activo. Durante aplicación suspender la publicación de fixes que puedan mezclar generaciones; escribir/confirmar cada nodo, verificar la flota y activar el survey del gateway solo al completar el conjunto requerido. No afirmar atomicidad distribuida: ante corte eléctrico o ACK perdido, recuperar consultando estado y reintentando idempotentemente. Persistir suficiente estado para continuar o restaurar la generación anterior tras reboot. Si el coordinador no puede verificar coherencia, mantener salida de posiciones suspendida y diagnóstico visible.

Pruebas: ACK perdido/duplicado, comando repetido, reinicio en cada etapa, fallo de NVS, un nodo antiguo, un nodo ausente y aplicación parcial. Ningún caso debe producir fixes etiquetados como válidos con geometría y tau de generaciones distintas.

### T6. Desactivar TWR automático y migrar modos B14

Archivos: `gw_selftau`, `uwb_gateway`, `sync_shell`, `ccp_master`, `ccp_slave`, config/store.

Retirar el arranque y la planificación del round-robin TWR, su latch por superframe y sus llamadas desde el loop de operación. Mantener `sync tau measure/result` como diagnóstico explícito: medir no debe aplicar ni persistir automáticamente. Al activar la configuración nueva, seleccionar la fuente estática y retirar el estado dinámico como fuente efectiva. Una migración de AUTO/TWR debe exigir compensación estática validada; no copiar una última muestra en RAM como calibración definitiva ni volver a un survey antiguo sin advertencia.

Conservar el diagnóstico manual y el rollback de datos/firmware B14. `sync tau auto on` devuelve un error explícito de función retirada; los modos de consola TWR/AUTO dejan de ser seleccionables como fuente activa en la instalación estática. Leer valores históricos solo para migrarlos, sin iniciar ranging ni adoptar mediciones dinámicas recibidas por CCP. Mantener el parser CCP v2 por compatibilidad y enviar ausencia de medición nueva; eliminar la dependencia operativa del filtro dinámico, sin reutilizar esa ausencia para invalidar la calibración estática. Comprobar que ninguna ruta de arranque, mantenimiento o reboot vuelve a encender el round-robin.

Pruebas: inicio frío, reboot, modo histórico AUTO/TWR/SURVEY, manual, CCP v2 sin nuevas mediciones y paso de más de 60 s sin cambio de tau estático. Hardware: contador de intercambios no aumenta durante operación ociosa, y sí durante una orden manual.

### T7. Separar survey de receptores y proteger validez de sync

Archivos: `tdoa_gw`, `tdoa_collect`, `pos_json`, `ccp_slave`, `sync_model`, `blink_rx`.

Contar únicamente receptores configurados que pertenecen al survey activo. Cinco nodos geométricos deben seguir dando cuatro observaciones esperadas; no publicar el gateway como ancla receptora en el payload de cliente. Mantener el ancla de origen, sus coordenadas XYZ y las transformaciones del mapa al filtrar la lista publicada.

Añadir o completar el control de edad del último CCP válido en la conversión del BLINK: el silencio debe invalidar el reloj antes de que vuelva a llegar un CCP, no depender únicamente del conteo de huecos al reanudarse la recepción. Derivar el límite de la política de holdover existente y probar sus fronteras; no equiparar los 60 s del viejo TWR con validez de reloj. Mantener las invariantes B14 de skew, feed-forward y wrap.

Pruebas: cuatro observaciones liberan un grupo de una geometría de cinco nodos; silencio CCP con BLINKs entrantes; recuperación/rebaseline; wrap; raíz distinta; configuración no válida. No ampliar esta tarea a ponderación NLOS del solver ni a cambiar sus límites de rendimiento.

### T8. Documentar contrato multi-hop y cerrar aceptación

Crear `docs/sync-multihop-design.md` con el contrato de esta sección y actualizar guías de survey, B14 y AGENTS sin borrar historia. Documentar consola realmente implementada, procedimiento de instalación, recuperación y límites.

Compilar imágenes de producción y calibración con la herramienta del proyecto, ejecutar suites tocadas y registrar RAM/stack. La compilación y las pruebas host no sustituyen los gates de hardware de la sección 7.

Dependencias: T0 → T1 → T2/T3 → T4 → T5 → T6 → T7 → T8. T7 puede desarrollarse antes, pero la aceptación final requiere la cadena completa.

## 6. Contrato para el futuro multi-hop

- Un árbol de sincronización tiene una raíz identificada, padres explícitos y ausencia de ciclos; la topología de sync es distinta de la red de transporte de observaciones y del grafo de survey.
- Cada hijo compensa exclusivamente el vuelo de su enlace padre→hijo. El padre anuncia su instante real de TX expresado en tiempo de raíz. No sumar otra vez el vuelo de sus antecesores.
- Conservar el esquema que anuncia timestamps de TX confirmados; no sustituirlos por tiempos planificados sin demostrar equivalencia.
- Un relevo necesita validez del modelo ascendente y de su propia referencia de transmisión. La cancelación RX demostrada para una ancla receptora no prueba por sí sola la exactitud TX de un retransmisor; validar calibración y conversión en una fase futura.
- Padre nuevo exige enlace calibrado y reconstrucción del modelo antes de aceptar observaciones. Configuración geométrica estática no equivale a permiso para seguir usando un reloj caducado.
- No mezclar observaciones de raíces o épocas temporales incompatibles. El futuro protocolo deberá transportar los identificadores necesarios; reservar datos ahora no significa que el wire actual ya lo garantice.
- Survey global exige rigidez y referencias compartidas suficientes. Una cadena de distancias padre–hijo no determina coordenadas; futuras celdas no pueden elegir gauges independientes y publicarlos como un único marco.
- El ruido/covarianza ascendente y las ventanas de emisión/recepción CCP requieren presupuesto propio. Desactivar TWR no resuelve por sí solo colisiones entre relés.

## 7. Aceptación en hardware

1. Instalación nueva: un gateway y cuatro anclas sin posiciones previas completan survey 3D con gauge explícito de cuatro anclas no coplanares. Contrastar aristas y coordenadas/alturas en el marco del gauge con mediciones independientes. Probar además que cuatro anclas coplanares producen un rechazo claro incluso con gateway fuera de su plano; no aplicar ese candidato.
2. Identidades: cambiar orden de arranque y dirección de una placa no intercambia sus coordenadas ni compensaciones. Reemplazar placa exige validación.
3. Persistencia: reiniciar los cinco equipos; misma generación, fuente estática y tau. Esperar al menos diez minutos: sin TWR autónomo ni degradación por los 60 s históricos.
4. Temporización: registrar beacon/CCP durante survey y durante operación. Cero deadlines perdidos atribuibles al nuevo camino en la captura de aceptación; reportar duración y carga.
5. Posicionamiento: con geometría fija y dos tags FAST, comparar antes/después posiciones, tasa de fixes, intervalos p50/p95/máximo, rechazos y sync por ancla. Usar misma red/broker y condiciones para no atribuir a B14 una mejora de transporte. No dar por cumplidos los antiguos 0.35 m en una celda de 25 m sin medir allí.
6. Pérdida CCP: impedir sync mientras siguen llegando BLINKs; verificar rechazo oportuno, contador y recuperación sin timestamps corruptos.
7. Fallo durante aplicación: provocar un nodo ausente y reboot en actualización; comprobar recuperación, ausencia de mezcla de generaciones y conservación de configuración anterior.
8. Cambio físico deliberado: recalibración manual restaura geometría/tau; no se espera seguimiento automático. Mantener las anclas fijas para aislar la prueba de mover únicamente el gateway.
9. Diagnóstico manual: una medición solicitada no cambia el enlace activo sin aplicación explícita.

Registrar resultados como PASS/FAIL/NO EJECUTADO con logs y configuración. Ninguna prueba física se considera hecha por la existencia de este plan.

## 8. Despliegue, rollback y límites

No flashear por redactar este plan. La implementación y campaña física son trabajos posteriores. Antes de desplegar, exportar configuración B14 y conservar binarios/registro anterior; coordinar mantenimiento porque APOS extendido requiere compatibilidad en gateway y anclas.

Rollback: restaurar firmware y configuración de una misma generación compatible. No reutilizar geometría posterior a un movimiento con tau anterior. El firmware nuevo no reactiva el TWR automático ni siquiera por restaurar datos antiguos. Un retorno explícito al binario B14 histórico recupera su comportamiento original, incluido su auto por defecto; debe estar documentado y controlado en el procedimiento de mantenimiento.

Fuera de alcance: servidor local, broker, optimización de colas/solver, aumento de tags, cambio de PHY o superframe, TWR→DS-TWR, calibración de split, NLOS de BLINK, gateway como receptor TDoA y ejecución multi-hop. Esta entrega elimina TWR periódico; no promete convertir todo el tiempo liberado en capacidad de tags sin un cambio separado del schedule y medición.

## 9. Criterio de cierre

D1–D3 confirmadas y reflejadas en código; tareas T0–T8 terminadas; pruebas host y builds registradas; gates físicos ejecutados o entrega marcada explícitamente como pendiente de hardware. El resultado debe funcionar con cinco nodos de survey 3D, gauge exclusivamente de cuatro anclas, cuatro receptores de BLINK, tau estático válido tras reboot y TWR limitado a survey o medición manual, sin opción de activación periódica.
