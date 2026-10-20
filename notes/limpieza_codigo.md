# Limpieza de codigo - candidatos a eliminar (2026-10-20)

Cada punto esta marcado en el codigo con un comentario `[ELIMINAR] <motivo>` justo
encima. No se ha borrado nada: las marcas son solo comentarios, y se comprobo que los
13 objetos compilados de `module-c-node3` y `module-b-main` siguen identicos byte a
byte (`objcopy --strip-debug` + `cmp`).

Para encontrarlas todas: `grep -rn "\[ELIMINAR\]" src platformio.ini`

## 1. Gasta energia en cada despertar (lo mas importante)

En despliegue no hay cable USB, asi que todo esto es tiempo despierto (~50 mA) sin
beneficio. Quitarlo acorta cada despertar en unos 3 s.

| Que | Donde | Coste |
|---|---|---|
| `delay(2000)` "dejar que el USB enumere" | `enableRails()` | 2 s por despertar |
| Diagnostico del bus I2C al arrancar (`delay(300)` + comprobaciones de pull-up) | principio de `setup()` | ~0,4 s por despertar |
| `i2cReportPullups()` antes y despues de los rails | `setup()` | por despertar |
| `scanI2C()` en cada arranque | `setup()` | por despertar |
| `probeBmeChipId()` dos veces; la de 0x77 siempre falla y llena el log de `Error -1` | `setup()` | por despertar |

Eran diagnosticos de la averia de SCL a masa, ya resuelta. Se pueden dejar detras de
una bandera de depuracion en vez de borrarlos.

## 2. No hace nada

- **`CFG,INTERVAL` y la accion "frecuencia" de Modulo B.** El nodo la acepta, la
  recorta y la guarda en `g_txPeriodMs`, pero el calendario no la lee. Quitar juntos:
  `g_txPeriodMs`, `g_txPeriodMsPersist`, `LORA_TX_PERIOD_MS/_MIN_MS/_MAX_MS`, la clave
  `INTERVAL` en `applyConfigCommand()` y la rama "frecuencia" en
  `handleConfigurarModuloC()`. La sustituye `NxT_enviodatos`.
- **`sen0466MaintainWarmup()`**: vacia, porque su cuerpo depende de
  `SLEEP_SKIP_SEN0466`, que vale 0. Quitar con `g_coWarmStartMs`, `g_coWarmRunning`,
  `SEN0466_RUN_WHEN_AWAKE` y sus dos llamadas en `loop()`.
- **`PowerManager`** (`.h` y `.cpp`) y la llamada `power.enable3V3Sensors()`: stub que
  solo imprime.

## 3. Ramas que nunca se compilan

- `#if SLEEP_ENABLED && SLEEP_SKIP_SEN0466` en `setup()` (`SLEEP_SKIP_SEN0466` = 0).
- `#if !TX_KEEP_5V_RAIL` en el corte de rails del envio y dentro de
  `sen0466MaintainWarmup()` (`TX_KEEP_5V_RAIL` = 1, y debe seguir asi).
- `SEN0466_GATE_ALWAYS_ON`: ajuste de banco superado por el precalentamiento; con 0
  fijo sobran sus ramas.

## 4. Constantes y variables sin uso

- Modulo C: `FIRST_TX_DELAY_MS`, `LORA_TX_MIN_GAP_MS`, `ALARM_TX_MIN_GAP_MS`,
  `LED_FLASH_MS`, `SENSOR_READ_EVERY_DEFAULT`, `LORA_TRANS_EVERY_DEFAULT`,
  `BURST_EVERY_DEFAULT`, `COUNTER_EVERY_MIN/MAX`, `READINGS_PER_PACKET_MAX`,
  `BATCH_READING_WORST_BYTES`, `SLEEP_MAX_AWAKE_MS`, `CM1106_FREEZE_LIMIT`,
  `READ_LOG_GAP_MS`, `BMV080_MEASURE_TIMEOUT_MS`; funcion `readingsPerPacket()`.
- Variables que se escriben y nunca se leen: `g_gasBaseline` (ocupa memoria RTC),
  `g_bmvPhaseStartedMs`, `g_calSt`.
- `pins.h`: `PIN_PCB_EN_ACTIVE` (ademas enganoso: las dos puertas tienen polaridad
  opuesta), `PIN_CM1106_RDY`. `sensor/Config.h`: `SAMPLE_INTERVAL_MS`.
- Etiquetas "SWAPPED" en el log de `cm1106.begin` / `bmv080.begin`.
- Modulo B: `NBIOT_BATCH_DEFAULT_READINGS/_SECONDS`, `NBIOT_PAYLOAD_MAX_BYTES`,
  `NBIOT_CFG_READINGS_MAX`; `NBIOT_RST_*` solo lo usa `modem_probe.cpp`.
- Modulo B, via UDP retirada: `parseQiopen`, `parseQiurcRecv`, `parseQirdHeader`,
  `parseDownlink`, `encodeBatch*`. Solo los usa la prueba unitaria; quitar con ella.

## 5. Entornos de compilacion

- `medir-normal-t`: depende de `BURST_EVERY_DEFAULT`, que ya no hace nada.
- `medir-co`: duplica `medir-alarma`.
- `medir-tiempos`: medida ya hecha.

## Lo que NO se ha marcado, a proposito

- Los entornos `medir-normal`, `medir-prealarma`, `medir-alarma`: se siguen usando
  para las medidas de consumo. Quitar antes del despliegue.
- Las herramientas de diagnostico (`module-c-power-bench`, `module-c-calypso-probe`,
  `module-c-cm1106-monitor`, `module-c-rail-gate`, `module-c-i2c-solo`,
  `module-b-modem-probe`): no entran en el firmware de produccion y cada una responde
  a una averia que puede volver.
- Comentarios historicos largos (ej. el de la puerta del SEN0466): explican por que
  el codigo es como es y sirven para la auditoria. Algunos se pueden acortar.
