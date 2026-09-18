# Plug-and-play flashing

Two files. **No compiler, no libraries, no build.** Each contains everything —
bootloader, partition table, OTA selector and application — merged into a single
image that goes to offset `0x0`.

| File | Board | What it does |
|---|---|---|
| `CHIP-FOREST_MODULE-C.bin` | Sensor node | Reads the sensors, transmits over LoRa |
| `CHIP-FOREST_MODULE-B.bin` | Relay | Receives LoRa, uplinks to ThingsBoard over NB-IoT |

Both are for the **ESP32-S3**.

---

## Flashing

Install `esptool` once:

```
pip install esptool
```

Then, with the board plugged in:

```
esptool.py --chip esp32s3 --port COM14 --baud 460800 write_flash 0x0 CHIP-FOREST_MODULE-C.bin
```

```
esptool.py --chip esp32s3 --port COM10 --baud 460800 write_flash 0x0 CHIP-FOREST_MODULE-B.bin
```

Replace the port with the board's own (Windows: Device Manager; Linux/macOS:
`/dev/ttyACM0`, `/dev/cu.usbmodem*`). The command is otherwise identical on every
platform.

---

## IMPORTANT: every sensor node gets LoRa address 1

`CHIP-FOREST_MODULE-C.bin` has **address 1 compiled into it**. Flash it to two
boards and both identify as address 1.

Module B then **cannot tell them apart**. Their readings merge into a single
ThingsBoard device — silently, with no error at either end — and what you get is
two sensors in two places averaged into one. That is worse than losing one of them,
because the result still looks like valid data.

**One node from this file is fine. More than one needs different addresses**, which
means building from source — see the parent `README.md`, where `platformio.ini`
provides `node1`, `node2` and `node3` environments (addresses 1, 3 and 4).

Address 2 belongs to Module B and is never given to a node.

---

## Before Module B will reach ThingsBoard

The binary carries a broker address, a gateway name and an access token compiled
in. Unless yours match exactly, it will connect to nothing:

| Setting | Value in this build |
|---|---|
| Broker | `test-moduloa.home.kg` |
| Port | `18831` — **not** 1883 |
| Gateway device | `CON-1`, with **"Is gateway" enabled** in ThingsBoard |

The port matters more than it looks: 1883 returns a well-formed `CONNACK 5` with
the same token, which reads as a credentials problem and is not.

Changing any of these means building from source.

It also needs a **working NB-IoT SIM**, and its own antenna.

---

## If the upload fails

**"the port is busy or doesn't exist"** on a node that already has firmware is
normal, not a fault. Deep sleep drops the USB port, so it exists for only a second
or two per wake. Either retry until a wake catches it, or power-cycle the board and
flash within 30 seconds — a cold boot deliberately stays awake that long to be
reprogrammable.

A brand-new board flashes first time; it has nothing to sleep.

---

## Checking it worked

Open the serial port at **115200**.

**Module C**, once per tick:

```
Node address: 1 (Module B sees this as +RCV addr=1)
[sentinel] bme=OK(T26.1 H34.0) co=NotInit(0.00)  (40ms)
TX [0..0/1] 72 bytes: 26.04,34.05,940.90,...  (sent)
[sleep] tick work done - sleeping 10s
```

**Module B**, on receiving and uplinking:

```
+RCV addr=1 len=72 rssi=-25 snr=13 data="26.04,34.05,..."
[nbiot-mqtt] -> AT+QMTPUB=0,0,0,0,"v1/gateway/telemetry"
[nbiot-mqtt] <- +QMTPUB: 0,0,0
```

Without a cable, Module C's LED says the same thing: **green wave** every wake,
**blue** while reading, **white** while transmitting, **red** if a sensor that was
expected did not answer, **off** asleep.

Module B has an OLED showing `NBIoT:` (the modem state machine — `IDLE` or
`PUBLISHING` mean the link is up) and `MQTT: activo` / `caido`.
