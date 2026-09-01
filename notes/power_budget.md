# CHIP FOREST — Module C power budget & sensor timing

Transcribed from the user's `alarma/prealarma/estado_normal/parametros/…` spreadsheet
(2026-09-01). This is the **deployment target** — the current bench firmware
(`chip_forest_lora_tx.cpp`, 15 s / no sleep) does NOT meet it and isn't meant to.

## Energy source — supercap bank (`parametros`)

| Parameter | Value |
|---|---|
| Bank capacity | 600 F |
| V max (charge) | 3.8 V |
| V min (usable) | 2.5 V |
| DC/DC efficiency | 0.85 |
| Leakage / ageing factor | 0.80 |
| Theoretical energy `0.5·C·(Vmax²−Vmin²)/3600` | 0.6825 Wh |
| After efficiency | 0.5801 Wh |
| **Practical energy available** | **464.1 mWh** |

Autonomy (h, no sun) = 464.1 mWh ÷ total power (mWh/h). `mW ≡ mWh consumed per hour`.

## Comms — ESP32 + LoRa (`parametros`)

| Element | V | I active | t active / send | I sleep |
|---|---|---|---|---|
| ESP32 | 3.3 V | 40 mA | 2 s | 15 µA (deep sleep, optimised PCB) |
| LoRa TX | 3.3 V | 25 mA | 0.1 s (time on air) | 1 µA |

`DUTY_CYCLE`: `I_avg µA = ((I_act mA · 1000 · t_act) + (I_sleep µA · (T − t_act))) / T`,
where `T = send_period_min · 60`. The ESP "2 s active" only holds if it deep-sleeps
through every sensor settling window (see SEN0466 below).

## Per-sensor profile

| Sensor | Rail | Method | I active | **t active / reading** | I idle/base | Base period | Notes |
|---|---|---|---|---|---|---|---|
| BME690 T/H/P | 3.3 V | BASE_ESCALADA | 2.2 µA | 1 s | 4.2 µA @ 5 min | 5 min | fast read |
| BME690 VOC (gas) | 3.3 V | DUTY_CYCLE | 3.1 mA | **10.8 s** | 0.11 µA | 20 min | gas algorithm cycle — do not truncate without validating |
| CM1106SL-NS CO₂ | 3.3 V ⚠️ | BASE_ESCALADA | 3 µA | 0.7 s | 2.5 µA | 30 min | single-shot; keep 30 min unless changed |
| SEN0466 CO | 5 V | DUTY_CYCLE | 5 mA | **210 s** | 0 | 30 min | conservative 210 s settle before reading |
| BMV080 PM | 3.3 V | DUTY_CYCLE | **68 mA** | 20 s | 30 µA | 30 min | validate enclosure vs smoke/dust/condensation |
| Calypso ULP wind | 5 V | DUTY_CYCLE | 0.15 mA | 1 s | 0 | 5 min | OFF in Normal; every 5 min in Prealarma/Alarma |

⚠️ Spreadsheet lists CM1106 at 3.3 V; on the bench it hangs at 3.3 V and needs 5 V
(now moved to the 5 V rail, 2026-09-01). Power impact is negligible (~0.008 mWh/h)
but the model's rail assignment should be corrected.

## Three operating states

| State | Send period | Sensor power | ESP+LoRa power | **Total** | **Autonomy** |
|---|---|---|---|---|---|
| **Normal** | 30 min | 5.622 mWh/h | 0.204 mWh/h | 5.826 mWh/h | **79.7 h (3.32 d)** |
| **Prealarma** | 15 min | 16.544 mWh/h | 0.355 mWh/h | 16.899 mWh/h | **27.5 h (1.14 d)** |
| **Alarma** | 3 min | 33.001 mWh/h | 1.565 mWh/h | 34.566 mWh/h | **13.4 h (0.56 d)** |

### Sampling period per sensor, per state (min)

| Sensor | Normal | Prealarma | Alarma |
|---|---|---|---|
| BME690 T/H/P | 5 | 3 | 1 |
| BME690 VOC | 20 | 10 | 5 |
| CM1106 CO₂ | 30 | 30 | 30 |
| SEN0466 CO | 30 | 10 | 5 |
| BMV080 PM | 30 | 10 | 5 |
| Calypso wind | off | 5 | 5 |

## Firmware implications

1. **Sampling is decoupled from sending.** Each sensor has its own period; the radio
   sends on a separate period carrying the latest value of every field (fresh or held).
   The current firmware reads everything every cycle — wrong model.
2. **The ESP must deep-sleep through settling windows** (SEN0466 210 s, BMV080 20 s,
   VOC 10.8 s), not spin — otherwise the 40 mA ESP current during those windows blows
   the budget. Rails switched on before, off after.
3. **Switched 5 V and 3.3 V rails** so the 68 mA BMV080 and 5 mA/210 s SEN0466 only
   draw during their windows.
4. **State machine** Normal→Prealarma→Alarma driven by sensor thresholds, changing both
   sampling and send periods.
5. `TX_INTERVAL_MIN_MS` / duty-cycle: send period also bounded by EU868 airtime, but at
   ≥3 min sends that's never the binding constraint (SF7 ~137 ms/packet).
