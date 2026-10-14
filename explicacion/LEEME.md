# CHIP FOREST: código simplificado para explicar

Dos proyectos PlatformIO independientes, uno por placa. Contienen la misma lógica que el
firmware real (calendario por sensor, tiempos medidos, sincronización de reloj, alarma),
pero en un solo `main.cpp` comentado en español, pensado para leerlo de arriba abajo.

| Carpeta | Placa | Qué hace | `main.cpp` |
|---|---|---|---|
| `modulo_c/` | Nodo sensor | Mide cada sensor en su minuto, evalúa el riesgo de incendio, manda lotes por LoRa y duerme | ~730 líneas |
| `modulo_b/` | Pasarela | Recibe los lotes, da la hora a los nodos, fecha cada lectura y la sube a ThingsBoard | ~510 líneas |

```
 Módulo C (x3)                  Módulo B                       ThingsBoard
 sensores ──► ESP32-S3 ──lotes LoRa──► ESP32-S3 ──NB-IoT/MQTT──► panel web
                       ◄──CFG,TIME───      ▲
                                           └── hora de la red móvil (AT+CCLK?)
```

Los dos compilan (comprobado el 2026-10-14). **No se han probado en las placas.** El
firmware de producción sigue siendo el de `src/`.

---

## Cómo abrirlos

1. En VS Code: **PlatformIO → Open Project** y elige `explicacion/modulo_b` o
   `explicacion/modulo_c`. Cada carpeta es un proyecto completo.
2. **Solo Módulo B:** en `src/main.cpp`, cambia `MQTT_TOKEN` por el token real del
   dispositivo pasarela en ThingsBoard. **No compartas el archivo con el token puesto.**
3. **Solo Módulo C:** compila una vez (fallará con `sfTk/bmv080.h: No such file or
   directory`) y después copia los ficheros del SDK de Bosch, que no vienen en la
   librería pública:

   ```powershell
   $lib = ".pio\libdeps\modulo_c\SparkFun BMV080 Arduino Library\src"
   $sdk = "..\..\bosch_bmv080_sdk"
   Copy-Item "$sdk\bmv080.h", "$sdk\bmv080_defs.h" "$lib\sfTk\"
   Copy-Item "$sdk\lib_bmv080.a", "$sdk\lib_postProcessor.a" "$lib\esp32s3\"
   ```

   Vuelve a compilar y ya pasa. Si copias el proyecto fuera de este repositorio, lleva
   también la carpeta `bosch_bmv080_sdk/`.

Para pegarlo en un proyecto nuevo propio: copia el `platformio.ini` y todo `src/`. Los
`.h`/`.cpp` que acompañan a `main.cpp` son los drivers reales de los sensores y de la
radio, copiados sin tocar desde `src/module_c/sensor/` y `src/shared/radio/`.

---

## Módulo C: el nodo sensor (`modulo_c/src/main.cpp`)

El nodo duerme casi siempre. **No se despierta cada X segundos fijos**, sino cuando a
algún sensor le toca medir o cuando toca enviar. Cada despertar es un arranque nuevo del
ESP32, así que lo que tiene que sobrevivir al sueño (reloj, calendario, lecturas, estado
de alarma) se guarda en memoria RTC (`RTC_DATA_ATTR`).

El archivo tiene tres piezas: el reloj (A), el calendario (B) y la alarma (C).

### A. El reloj

El ESP32 no tiene reloj con pila. Mientras duerme cuenta el tiempo con un oscilador RC
interno, que **se desvía un 1-2 %**, y `millis()` vuelve a cero en cada despertar. Por
eso el nodo lleva dos tiempos:

| | Qué es | Función |
|---|---|---|
| **Tiempo del nodo** | Lo que el nodo cree que ha pasado desde que se encendió. Siempre existe. Se acumula en `dormir()`: lo que estuvo despierto + lo que va a dormir. | `tiempoNodoMs()` |
| **Hora real** | La hora de la red móvil. Módulo B se la manda con `CFG,TIME=<segundos desde 1970>` justo después de recibir sus datos. | `horaRealMs()` |

`sincronizarReloj()` hace tres cosas cada vez que llega la hora:

1. **Apunta la pareja** (hora real, tiempo del nodo) de ese instante. A partir de ahí,
   hora real = hora recibida + tiempo transcurrido × deriva.
2. **Aprende la deriva** comparando dos sincronizaciones: si el nodo contó 1000 s y la
   red dice que pasaron 1013, la deriva es 1,013 (su oscilador va un 1,3 % lento).
   - Solo la acepta entre 0,97 y 1,03. Un valor fuera de ahí no es deriva, es un error.
   - Solo la mide si han pasado al menos 5 minutos. En menos, un 1 % es ruido.
3. **La primera vez, rehace el calendario.** Desde ese momento todos los nodos miden en
   los mismos minutos de reloj, y sus datos son comparables.

**La deriva se usa al dormir** (`segundosHastaDespertar()`). Para dormir 300 s reales se
le piden al oscilador 300 / deriva de los suyos. Esta división es la que hace que "cada 5
minutos" signifique 5 minutos de verdad.

### B. El calendario

Una tabla de trabajos (`TRABAJOS[]`): uno por sensor y otro para el envío. Cada uno tiene
un periodo **por estado de alarma** y un **tiempo de preparación**:

| Trabajo | NORMAL | PRE-ALARMA | ALARMA | Preparación | Por qué esa preparación |
|---|---|---|---|---|---|
| BME690 temp/humedad/presión | 5 min | 3 min | 1 min | 1,2 s | lectura rápida, sin calefactor |
| BME690 gas (VOC) | 20 min | 10 min | 5 min | 11 s | calefactor a 320 °C |
| CM1106 CO₂ | 30 min | 30 min | 30 min | 4 s | calentamiento tras el ciclo de alimentación |
| SEN0466 CO | 30 min | 10 min | 5 min | 0,5 s | lectura inmediata |
| BMV080 partículas | 30 min | 10 min | 5 min | 8 s | 5 s de arranque + 2 s de láser |
| Calypso viento | 5 min | 5 min | 5 min | 21,5 s | **20 s de arranque** + 1,5 s de escucha |
| Envío a Módulo B | 30 min (+1) | 15 min | 3 min | 0,5 s | |

- **Alineado al reloj.** Una medida de periodo 5 min cae en :00, :05, :10… y no "5 minutos
  después de la anterior" (`siguienteMultiplo()`). Así el retraso de una lectura no se va
  acumulando.
- **Se despierta antes de tiempo, por la preparación.** El viento de las 12:05 despierta
  al nodo a las 12:04:38,5, para que el dato exista a las 12:05 y no empiece a prepararse
  entonces.
- **Agrupa lo que vence a la vez.** La temperatura y el viento van los dos cada 5 min. Sin
  agrupar, irían en dos despertares seguidos, porque sus preparaciones son distintas.
- **El envío va en el minuto 31, no en el 30.** En el minuto 30 vencen cinco lecturas que
  van en fila (~19 s), y así se transmite con todo ya leído.

**Una hora en estado NORMAL**, simulada con este mismo código:

```
 5 min  T/H/P viento          35 min  T/H/P viento
10 min  T/H/P viento          40 min  T/H/P gas viento
15 min  T/H/P viento          45 min  T/H/P viento
20 min  T/H/P gas viento      50 min  T/H/P viento
25 min  T/H/P viento          55 min  T/H/P viento
30 min  T/H/P CO2 CO PM viento 60 min  T/H/P gas CO2 CO PM viento
31 min  ENVIO (6 lecturas)    61 min  ENVIO (6 lecturas)
```

**Tiempos medidos en el banco** (nodo C3, 2026-10-06 y 2026-10-09):

| Sensor | Tiempo leyendo | Nota |
|---|---|---|
| BME690 | 0,04 s | el calefactor trabaja dentro del sensor y no bloquea el ESP32 |
| CM1106 | 3,5 s | solo mide al encenderse: cada valor nuevo exige un ciclo de alimentación |
| BMV080 | 2,1 s | láser de ~68 mA: se enciende solo durante la medida |
| Calypso | 1,5 s | pero necesita **20 s encendido** antes: a los 12,4 s no contesta, a los 22,7 s sí |
| SEN0466 | 0,02 s | |
| Reposo entre sensores | 1,5 s | **nunca dos a la vez**: juntos tumban la placa (31 reinicios frente a 2) |
| Barrido completo | 19,3 s | de los que solo 7,2 s son medida |

**Las lecturas se guardan y se envían en lotes.** Cada despertar guarda una fila con solo
lo que se midió (el resto, `-999` = "no tocaba") y el segundo en que se tomó. Al enviar,
cada fila lleva su **antigüedad en segundos**:

```
B,6,21.5,40.2,1013.2,-999,-999,-999,-999,-999,-999,-999,180,1.2,1,1555;21.6,...;...
    └────────────────────────── 13 valores ───────────────────────────┘ └edad┘
```

El nodo no necesita saber la hora para esto: Módulo B, que sí la sabe, fecha cada fila
restando la antigüedad a la hora en que la recibe.

**Orden de una transmisión** (`transmitir()`):

1. Apagar los raíles de los sensores: la radio pide ~120 mA y la placa no da tanto con
   todo encendido.
2. Mandar el lote, en uno o varios paquetes de 240 bytes como máximo.
3. Escuchar 2 s. Es el **único** momento en que el nodo oye a Módulo B, y ahí llega la hora.
4. Mandar la trama de estado `STAT`, cuyo campo 8 es el estado de alarma.

### C. La alarma

Cuatro umbrales por magnitud: dos para **entrar** (`preOn`, `almOn`) y dos, algo más
bajos, para **salir** (`preOff`, `almOff`). Es una histéresis: un valor que oscila justo
en el umbral no hace parpadear el estado. Basta una magnitud para subir de nivel, y la
humedad va al revés que el resto, porque en un incendio baja.

| Magnitud | Pre-alarma | Alarma |
|---|---|---|
| Temperatura | ≥ 50 °C | ≥ 60 °C |
| Humedad | ≤ 20 % | ≤ 10 % |
| PM1 / PM2.5 / PM10 | ≥ 100 µg/m³ | ≥ 250 µg/m³ |
| CO | ≥ 8 ppm | ≥ 20 ppm |
| Viento | ≥ 5 | ≥ 10 |

- **Confirmación:** una pre-alarma que dura 60 minutos pasa a alarma.
- **Al cambiar de estado se rehace el calendario,** porque cambian todos los periodos.
- **En pre-alarma y alarma el nodo no duerme:** mira el calendario cada 2 s y envía cada
  15 o 3 minutos.
- **Se evalúa sobre el último valor conocido de cada magnitud.** Un fuego detectado por
  las partículas no se olvida porque el siguiente despertar solo mida la temperatura.

**Arranque en frío** (encendido o reset, no despertar del sueño): se borra la memoria RTC
y se rehace el calendario. El nodo se queda despierto 30 s antes de dormir por primera
vez, porque dormido el USB desaparece y no se podría reflashear. **El primer dato sale a
los 5 minutos y el primer envío a los 31**, y no antes, porque todo va en la rejilla del
reloj.

---

## Módulo B: la pasarela (`modulo_b/src/main.cpp`)

| Paso | Función | Qué hace |
|---|---|---|
| 1 | `setup()` → `radio.begin(...)` | Configura la radio LoRa con **exactamente** los mismos parámetros que los nodos. |
| 2 | `conectarModem()` | Enciende el modem, comprueba la SIM, se registra en la red, **lee la hora** y abre MQTT. |
| 3 | `loop()` → `radio.poll()` | Recoge las tramas que hayan llegado. No espera: si no hay nada, sigue. |
| 4 | `procesarTramaLoRa()` | Separa lotes (`B,`), estado (`STAT,`) y confirmaciones (`ACK,`). |
| 5 | `mandarHoraSiToca()` | **Lo primero al recibir un lote:** le manda la hora al nodo, como mucho una vez cada 5 min. |
| 6 | `interpretarLote()` | Trocea cada fila (14 campos) y la fecha: hora de ahora − antigüedad. |
| 7 | `publicarCola()` | Cada 10 min, o **al momento si un nodo está en alarma**, sube lo acumulado. |

**La hora.** El modem la obtiene de la red móvil (`AT+CCLK?`). Entre lectura y lectura,
Módulo B cuenta con `millis()`, que en esta placa es fiable porque nunca duerme. Un
detalle medido: **este modem da la hora ya en UTC** aunque anuncie la zona (`+08` = +2 h).
Restarla, como diría la norma, atrasaba todas las lecturas dos horas.

**El modem se maneja con comandos AT**, texto por el puerto serie:

| Comando | Para qué |
|---|---|
| `AT` | ¿Estás vivo? |
| `AT+CTZU=1` | Que el modem actualice su hora con la de la red. |
| `AT+CPIN?` | ¿Está la SIM lista? |
| `AT+QCGDEFCONT="IP","sensor.net"` | Configura el APN de la SIM. |
| `AT+CEREG?` | ¿Registrado en la red? (`0,1` red propia, `0,5` itinerancia) |
| `AT+CCLK?` | La hora de la red. |
| `AT+QMTOPEN=0,"host",puerto` | Abre la conexión con el servidor MQTT. |
| `AT+QMTCONN=0,"id","token"` | Se autentica en ThingsBoard con el token. |
| `AT+QMTPUB=0,0,0,0,"topic"` | Publica: el modem contesta `>`, se manda el JSON y se cierra con Ctrl+Z. |

**Por qué `v1/gateway/telemetry`:** en ThingsBoard, Módulo B es una *pasarela*, un solo
dispositivo que publica en nombre de varios. El JSON va agrupado por nodo, y cada lectura
lleva su hora (`ts`, en ms) para que ThingsBoard la pinte cuando se tomó y no cuando se
subió:

```json
{"NodoC-3":[{"ts":1791983100000,"values":{"temp":21.50,"rh":40.20,"windValid":true,"rssi":-60,"alarmState":0}}]}
```

Los campos con `-999` no se publican. Si se publicaran, la gráfica pintaría una medida
real de -999. Mientras Módulo B no tenga hora de red, las lecturas van sin `ts` y
ThingsBoard pone la hora de llegada.

---

## Qué se ha quitado respecto al firmware real

Conviene decirlo al explicar, para que nadie piense que el sistema real es así de simple:

**Módulo C** (`src/module_c/communications/chip_forest_lora_tx.cpp`, 4.500 líneas):
- **Bajar la alarma.** El real exige cinco lecturas limpias seguidas; aquí basta con bajar
  del umbral de salida.
- **Datos frescos.** El real comprueba que cada regla de alarma tenga un dato *fresco* de
  su sensor; aquí se usa el último conocido.
- **Calefactor del BME690.** El real reinicia los raíles después de la medida de gas, y
  calienta el sensor de CO en alarma.
- **Comandos `CFG` además de `TIME`:** cambiar umbrales y periodos desde ThingsBoard.
- **Modo seguro.** Tras 12 arranques fallidos deja los sensores apagados y avisa por radio.
- **Memoria RTC protegida** (`RTC_NOINIT` con número mágico), para sobrevivir también a
  un reinicio por caída de tensión.
- **LED de estado y medida de batería.**

**Módulo B** (`src/module_b/communications/`, ~3.800 líneas):
- **El modem no bloquea.** En el real es una máquina de estados, para no perder tramas
  LoRa mientras espera al modem. Aquí las funciones esperan, que es más fácil de leer.
- **Reintentos** con espera creciente y reconexión automática de MQTT.
- **Comandos de ThingsBoard → nodo:** los recibe por MQTT y los reenvía al nodo en su
  ventana de escucha. Tienen prioridad sobre la hora.
- **Pantalla OLED, panel web por WiFi y CSV del histórico.**
