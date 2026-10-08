# Nueva estrategia de lectura: por TIEMPO, no por ticks

Especificada por el usuario el 2026-10-08. **Sustituye al planificador por ticks.**

## ALCANCE: SOLO NODO C3

Decidido el 2026-10-08. Se trabaja **únicamente sobre `module-c-node3`** (dirección
LoRa 4, que Módulo B traduce a `NodoC-3`). Es el único nodo que se puede
reprogramar y verificar ahora mismo; C1 y C2 llevan semanas sin actualizar.

**No hay que mantener compatibilidad hacia atrás.** El usuario acepta que C1 y C2
se desconecten y dejen de mandar datos mientras dure el desarrollo; se flashearán
cuando el firmware nuevo funcione. Así que el formato de trama cambia de golpe,
sin prefijo discriminador ni doble parser en Módulo B.

## Por qué se cambia

El planificador actual cuenta en ticks (`SLEEP_CYCLE_SECONDS 10`), y un tick no
mide tiempo: el sueño son 10 s fijos **además** del trabajo, así que el tick real
son ~18,8 s y además varía con lo que tarde cada lectura. Consecuencias:

- Toda cadencia sale ~1,9x más lenta que la configurada (burst de 15 min -> 28,2 min).
- `NODE_TICK_SECONDS 10` en Módulo B fecha las lecturas multiplicando por 10 s
  cuando el tick real son 18,8, así que el histórico está comprimido casi a la
  mitad. Ver la tarea 3 de [[tareas-pendientes-desarrollo]].
- Ninguna constante fija puede acertar, porque el tick no es constante.

Pasando a periodos en tiempo real los tres problemas desaparecen a la vez.

## Los periodos, en MINUTOS

| Sensor | Mide | NORMAL | PREALARMA | ALARMA |
|---|---|---|---|---|
| BME690 — T/H/P | temperatura, humedad, presión | 5 | 3 | 1 |
| BME690 — VOC | resistencia de gas | 20 | 10 | 5 |
| CM1106SL-NS | CO2 | 30 | 30 | 30 |
| SEN0466 | CO | 30 | 10 | 5 |
| BMV080 | PM1 / PM2.5 / PM10 | 30 | 10 | 5 |
| Calypso ULP | viento: ángulo + velocidad | 5 | 5 | 5 |

Dos constantes en los tres estados: el **CO2 siempre a 30 min** (deliberado: el
CM1106 es el sensor menos fiable, se congela clavado en 5000 y solo se recupera
con ciclo de alimentación) y el **viento siempre a 5 min**.

## Lo que esto implica en el diseño

**El BME690 son DOS trabajos, no uno.** `readFast()` da T/H/P con el calefactor
apagado (~200 ms); `read()` enciende el calefactor a 320 °C (~10,8 s) y da además
el gas. Son los dos periodos de la tabla. **Decisión pendiente:** si el T/H/P de
los minutos que coinciden con VOC se reaprovecha de la lectura completa (más
barato) o se toma siempre de una rápida aparte (homogéneo). Recomendado lo
segundo: el calefactor sesga la temperatura del propio chip, y 1 de cada 4
muestras sesgada es peor que pagar 200 ms.

**El SEN0466 está permanentemente encendido.** Confirmado por el usuario el
2026-10-08: va cableado directo a corriente, saltándose el transistor, así que
`SLEEP_SKIP_SEN0466` está a 0. Sus 210 s de estabilización se pagan **una sola
vez** al energizar la placa; a partir de ahí sigue caliente aunque el ESP duerma.

Consecuencias, las dos importantes:

1. **El CO se puede leer durmiendo**, en normal y en prealarma. Desaparece la
   razón por la que el nodo tenía que quedarse despierto para medirlo, y el CO
   pasa de ser sólo confirmación a poder ser detección.
2. **~15 mA permanentes, 24/7.** El sensor no tiene modo de reposo. Son 360 mAh
   al día, y sobre el pack de 10 Ah eso son **~28 días sólo del sensor de CO**,
   independientemente de lo que haga el firmware. Es el suelo del presupuesto
   energético: el rediseño baja los despertares de 191/hora a 12/hora, pero por
   debajo de ese suelo no se baja por software. Ver [[power-source-now-lithium]].

**OBSOLETO Y HAY QUE REESCRIBIRLO:** el bloque de comentario de
`chip_forest_lora_tx.cpp:633`, titulado "WHY THE CO SENSOR CAN RUN IN PRE-ALARM
AND ALARM BUT NOT IN NORMAL OPERATION", describe el cableado anterior y ahora
engaña.

**Despertares por hora**, que es donde está el ahorro:

| Estado | Periodo más corto | Despertares/hora |
|---|---|---|
| NORMAL | 5 min | 12 |
| PREALARMA | 3 min | 28 (unión de múltiplos de 3, 5, 10 y 30) |
| ALARMA | 1 min | 60 |

Hoy, con el tick de 18,8 s, son **191 despertares/hora en estado normal**. El
nuevo normal son 12: dieciséis veces menos.

**EN ALARMA EL NODO NO DUERME.** Decidido por el usuario el 2026-10-08, y coincide
con lo que ya hace la rama de alarma actual. A 1 min de periodo el ahorro de
dormir sería marginal, y dormir rompería el SEN0466: sus 210 s de calentamiento
no caben en un ciclo de 1 min, así que el CO no se mediría justo cuando importa.
Despierto, los raíles se quedan arriba y el sensor está siempre caliente.

## El viento: DECIDIDO - lectura instantánea, ángulo Y velocidad

Resuelto el 2026-10-08. **Nada de medias.** En cada despertar de los 5 min se lee
una sola trama NMEA y se guardan **las dos magnitudes**:

- `values[0]` = ángulo del viento (grados, relativo)
- `values[1]` = velocidad (m/s)
- `values[2]` = validez (1 = trama `A`, válida)

Esto descarta la media vectorial que se había propuesto, y con ella desaparece el
problema de promediar ángulos (la media aritmética de 350° y 10° da 180°, la
dirección contraria). No aplica: no se promedia nada.

Coste: ~1,5 s despierto (`CALYPSO_READ_WINDOW_MS`), 12 despertares/hora en normal.
Lo que se acepta a cambio: una ráfaga puntual en el instante del muestreo queda
registrada como si fuera el viento de esos 5 minutos.

## EL CONSUMO YA NO ES UNA RESTRICCION

Decidido por el usuario el 2026-10-08: "olvida lo de conservar consumos". Lo que
manda es que cada medida caiga cuando toca, no el ahorro. En concreto:

- **Despertar con antelación generosa.** El nodo despierta en
  `instante_objetivo - preparación` de cada sensor, redondeando hacia arriba, para
  que el dato exista en el instante pedido en vez de empezar a calentar entonces.
- Los dos tiempos sin confirmar (`CM1106_WARMUP_MS 3000`, `BMV080_STARTUP_DELAY_MS
  5000`, ambos marcados "UNCONFIRMED vs datasheet" en `sensor/Config.h`) se pueden
  **sobreestimar con margen** en vez de medirlos primero. Sobrar tiempo despierto
  ya no cuesta nada; faltar hace que la medida llegue tarde.
- Los ~15 mA permanentes del SEN0466 dejan de ser un problema.
- Prealarma puede quedarse despierta igual que la alarma.

Lo que NO cambia: el nodo sigue durmiendo entre medidas en estado normal - es lo
que pidió el usuario explícitamente - sólo que ya no hay que exprimirlo.

## Tiempos de preparación de cada sensor

| Sensor | Preparación | Fuente |
|---|---|---|
| BME690 — T/H/P | ~0,2 s + ~1 s de levantar el raíl 3V3 | calefactor apagado |
| BME690 — VOC | **10,8 s** de calefactor a 320 °C | `notes/power_budget.md` |
| BMV080 | 5 s arranque + 2 s láser (SIN CONFIRMAR) | `BMV080_STARTUP_DELAY_MS`, `BMV080_SETTLE_MS` |
| CM1106 | 3 s + ciclo de alimentación (SIN CONFIRMAR) | `CM1106_WARMUP_MS` |
| SEN0466 | **~0 s** - siempre alimentado y caliente | ver arriba |
| Calypso | 1,5 s de escucha NMEA | `CALYPSO_READ_WINDOW_MS` |

**Los sensores NO pueden leerse a la vez**: en paralelo reinician la placa, por eso
van en slots secuenciales - ver [[module-c-sequential-sensors]]. Así que cuando
varios coinciden en el mismo minuto es imposible que todos caigan en el instante
exacto; el barrido los reparte sobre ~19 s. La solución acordada es doble: anclar
el **BME690 T/H/P** al minuto exacto (es el de periodo más corto y el que dispara
la alarma) y **fechar cada lectura con su instante real** en vez de forzarlas
todas al mismo - lo cual arregla de paso el `ts` que venimos arrastrando.

## Envío a Módulo B

**NORMAL: periodo de 30 min desplazado 1 min.** Envíos en el minuto 31, 61, 91...
siempre un minuto después del barrido de los 30.

El desfase es deliberado y es lo que el usuario pidió: en el minuto 30 vencen
CINCO lecturas a la vez (T/H/P y viento por ser múltiplos de 5; CO2, CO y
partículas por ser múltiplos de 30), y como los sensores no pueden leerse en
paralelo van en fila y tardan **~22 s** calculados, **19,3 s medidos**. El usuario
propuso primero 10 s de desfase, que habría enviado el paquete con las partículas
y el CO2 a medias. Un minuto da margen de sobra.

**OJO - no es un periodo de 31 minutos.** Un periodo de 31 min derivaría: los
envíos caerían en 31, 62, 93, 124... mientras las lecturas caen en 30, 60, 90,
120, y el desfase crecería un minuto cada vuelta hasta perder toda relación con el
barrido. Es periodo 30 + desfase 1, no periodo 31.

**PREALARMA: cada 15 min. ALARMA: cada 3 min.** (Hoy la rama de alarma manda cada
5 s, así que esto es mucho más espaciado que lo actual.)

### CHOQUE CON EL LIMITE DEL PAQUETE - SIN RESOLVER

El RYLR998 admite 240 bytes, y hoy eso son **3 lecturas por paquete**
(`READINGS_PER_PACKET_MAX`, con `BATCH_READING_WORST_BYTES 70`). Los periodos
acordados producen más:

| Estado | Lectura más frecuente | Envío | Lecturas acumuladas | ¿Cabe? |
|---|---|---|---|---|
| NORMAL | T/H/P y viento cada 5 min | 30 min | **6** | NO |
| PREALARMA | T/H/P cada 3 min | 15 min | **5** | NO |
| ALARMA | T/H/P cada 1 min | 3 min | **3** | justo |

Hay salida, y pasa por el formato: los 70 bytes son el **peor caso con todos los
campos llenos**, pero con la nueva estrategia la mayoría de las filas van casi
vacías. En el minuto 5 sólo hay T/H/P y viento; CO2, CO y partículas no han
vencido. Una fila dispersa que sólo lleve los campos frescos ocupa la mitad o
menos, y entonces las 6 caben.

Opciones, de mejor a peor:
1. **Formato disperso**: cada fila lleva sólo los campos que han medido de verdad.
   Además resuelve un problema viejo - hoy la telemetría repite el último valor
   conocido, lo que hace indistinguible "midió lo mismo" de "no midió".
2. **Varios paquetes por envío**: hoy el código se niega a partir un lote; podría
   mandar 2 seguidos.
3. Descartar lecturas y mandar sólo las 3 últimas. Se pierde dato.

## Decisiones del planificador (2026-10-08)

**1. Los periodos se alinean al RELOJ DE PARED**, no al arranque. Una lectura de
periodo P cae en los múltiplos de P de la hora real: T/H/P a las :00, :05, :10...
Eso hace que **los tres nodos midan en el mismo instante**, así que sus datos son
comparables entre sí y con cualquier otra fuente - que es lo que de verdad
significa "medida en el minuto exacto". Mientras el nodo no haya recibido su
primer `CFG,TIME` planifica con su tiempo propio y se re-fasea al sincronizar.

**2. En PREALARMA el nodo NO duerme**, igual que en alarma. Sólo el estado normal
duerme. Así prealarma y alarma comparten camino de código, y durante los 10
minutos de confirmación - cuando se decide si hay fuego - el nodo está reactivo.

**3. Al cambiar de estado se mide INMEDIATAMENTE y se re-fasea.** Entrar en
prealarma significa que algo cruzó un umbral: ese es el momento de mirar, no tres
minutos después. El calendario arranca desde ese instante con los periodos nuevos.

**Ojo con la tensión entre 1 y 3:** re-fasear rompe la alineación al reloj. No
importa mientras dure el episodio, porque ahí el nodo está despierto y manda la
latencia; pero **al volver a normal hay que re-alinearse al reloj de pared**.

## Los comandos de Módulo A sobrevivien como MULTIPLICADORES

`INTERVAL`, `NXT_LECTURA` y `NXT_ENVIO` son las tres claves que Módulo B genera
desde las acciones de Módulo A, y hoy están atadas a ticks. Pasan a multiplicar
los minutos de la tabla: `NXT_LECTURA=2` duplica todos los periodos de sensor,
`NXT_ENVIO=0.5` divide a la mitad el de envío. La cadena A->B->C sigue funcionando
sin tocar Módulo A ni Módulo B. Ver [[downlink-a-to-b-unresolved]].

Las claves atadas a ticks que desaparecen: `SENSOR_READ`, `BURST`, `LORA_TRANS`.

## Abierto
- **El T/H/P en los minutos que coinciden con VOC**: reaprovechar la lectura
  completa o tomar una rápida aparte (recomendado lo segundo, ver arriba).
