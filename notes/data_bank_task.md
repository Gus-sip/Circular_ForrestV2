# Task — synthetic data bank: ~200 Module C nodes × 1 year

Opened 2026-09-01. Not started - captured here so it isn't lost.

## The ask

Build a "data bank" of roughly a year's worth of Module C telemetry across
~200 nodes. Since 200 real nodes can't be run for a year, this is a
**synthetic / simulated** dataset. The ~16k real rows in
`logs/module_b_lora_log.csv` (one node, ~4 days, 5-min cadence as of
2026-09-01) are the reference for realistic value ranges, noise and
cross-sensor correlation.

## Volume math

| Cadence | Rows/node/year | 200 nodes |
|---|---|---|
| 5 min (current bench) | ~105,000 | ~21 M |
| 30 min (deployment "Normal", notes/power_budget.md) | ~17,500 | ~3.5 M |

Row = 13 telemetry fields (temp,hum,pres,gas,pm1,pm25,pm10,co2,co,coTemp,
windAngle,windSpeed,windValid) + node id + timestamp, optionally rssi/snr and
lat/lon. As CSV: single-digit GB at 5 min, ~hundreds of MB at 30 min; ~10x
smaller as Parquet.

## Open design decisions (blocked on the user)

1. **Purpose** — scale-testing ThingsBoard / analytics & dashboard dev / ML
   training (needs labelled events) / storage sizing. Drives fidelity.
2. **Destination** — flat files (CSV/Parquet, partitioned), streamed into
   ThingsBoard via the gateway MQTT API as 200 devices, or a DB
   (Timescale/Postgres/SQLite).
3. **Fidelity** — full (diurnal + seasonal + shared weather for nearby nodes +
   sensor noise + gaps + injected faults like the CM1106 freeze) vs
   trends+simple-noise vs plausible-random.
4. **Cadence** — 5 min, 30 min, or state-driven (mostly 30 min, faster during
   simulated pre-alarm/alarm episodes).

## Notes / building blocks already on hand

- Real per-sensor ranges & behaviour: see `notes/module_c_STATUS.md`,
  `logs/module_b_lora_log.csv`.
- `CM1106` freeze / peg-at-5000 is a real, characterisable fault to inject.
- `SEN0466` CO reads ~0 in clean air (correct) - episodes would be rare spikes.
- Wind: `$IIMWV` angle 0-359, speed m/s; `windValid` 0/1.
- `notes/power_budget.md` has the Normal/Prealarma/Alarma sampling model if the
  cadence should be state-driven.
- `logs/csv_for_excel.py` shows the Spanish-locale CSV convention the user uses.
