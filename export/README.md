# CHIP FOREST — Module C firmware (standalone export)

Everything needed to build and flash a Module C sensor node, without the rest of
the development tree.

---

---

## Just want to flash a board? Don't build anything.

`prebuilt/` contains **two ready-to-flash binaries** — one per module. Each is
bootloader, partition table, OTA selector and application merged into a single
image. No compiler, no libraries, no Bosch SDK step:

```
esptool.py --chip esp32s3 --port COM14 --baud 460800 write_flash 0x0 prebuilt/CHIP-FOREST_MODULE-C.bin
esptool.py --chip esp32s3 --port COM10 --baud 460800 write_flash 0x0 prebuilt/CHIP-FOREST_MODULE-B.bin
```

See `prebuilt/FLASH.md`. Build from source only when you need to CHANGE something
— in particular, **`CHIP-FOREST_MODULE-C.bin` has LoRa address 1 compiled in**, so
a second node needs the `node2` environment below rather than a second copy of that
file.

---

## The one thing that will trip you up

**The BMV080 particulate sensor needs two precompiled Bosch binaries that are not
in any public library.** They are in `bosch_bmv080_sdk/` here, and they must be
copied into the SparkFun library *after* PlatformIO has downloaded it.

Until you do, the build fails with:

```
fatal error: sfTk/bmv080.h: No such file or directory
```

That reads like a missing library. It is not — the library is there, it just ships
without Bosch's proprietary blobs.

**This is also why the firmware cannot be delivered as a single copy-paste file.**
`lib_bmv080.a` and `lib_postProcessor.a` are binaries, not text.

---

## Build steps

**1. Open this folder** in PlatformIO (VS Code: *File → Open Folder*). Not the
parent — this folder.

**2. Build once and let it fail.** This is expected. It downloads the libraries:

```
pio run -e node1
```

**3. Copy the Bosch SDK into the downloaded library.** From `bosch_bmv080_sdk/`:

```powershell
./restore.ps1 -EnvName node1
```

Or by hand, into
`.pio/libdeps/<env>/SparkFun BMV080 Arduino Library/src/`:

| From `bosch_bmv080_sdk/` | To |
|---|---|
| `bmv080.h` | `src/sfTk/bmv080.h` |
| `bmv080_defs.h` | `src/sfTk/bmv080_defs.h` |
| `lib_bmv080.a` | `src/esp32s3/lib_bmv080.a` |
| `lib_postProcessor.a` | `src/esp32s3/lib_postProcessor.a` |

**`.pio/libdeps` is per-environment.** Building `node2` downloads a *second* copy
of the library, which needs the same treatment. Re-run `restore.ps1` for each
environment you build.

**4. Build and upload:**

```
pio run -e node1 -t upload
```

---

## Which environment to flash

**Every node needs its own address.** Two nodes sharing one means Module B cannot
tell them apart, and their readings interleave into a single device — silently,
with no error anywhere.

| Environment | LoRa address | Appears as |
|---|---|---|
| `node1` | 1 | `NodoC-1` |
| `node2` | 3 | `NodoC-2` |
| `node3` | 4 | `NodoC-3` |

**Address 2 is Module B** and must never be given to a node. Add more environments
by copying the pattern in `platformio.ini`, and add matching entries to
`NBIOT_NODE_NAMES` in Module B's `Config.h` or they arrive as
`NodoDesconocido-<n>`.

**Label each board physically with its address.** The boards are identical and the
COM port number follows the USB socket, not the board.

---

## Hardware this firmware expects

Pin assignments are in `src/pins.h`. The ones that are not obvious:

| Pin | Purpose | Note |
|---|---|---|
| GPIO10 | 3V3 sensor rail gate | **active LOW** |
| GPIO11 | 5V sensor rail gate | **active HIGH** — opposite polarity to GPIO10 |
| GPIO13 | LoRa rail gate | active LOW |
| GPIO15 | BMV080 enable | active LOW |
| GPIO14 | Supercap voltage sense | **analog input — never drive it** |
| GPIO12 | Cubic CM1106 line | **do not drive** |
| GPIO21 | Status LED | WS2812, **NEO_RGB** order, not the usual GRB |

The two rail gates having **opposite active levels** has caused real confusion —
driving both low leaves the 5V rail off, and the sensors on it simply appear dead.

---

## Reflashing a running node

With deep sleep enabled the USB port only exists for a second or two per wake, so
uploads fail with *"the port is busy or doesn't exist"*. That is not a fault.

Either **retry the upload in a loop** until a wake catches it, or **power-cycle the
board and flash within 30 seconds** — a cold boot deliberately stays awake that
long so it can be reprogrammed.

---

## Serial output

115200 baud over the board's native USB. A healthy node prints, per tick:

```
Tick: sensor_read 3/3, lora_trans 4/4
[sentinel] bme=OK(T26.1 H34.0) co=NotInit(0.00)  (40ms)
TX [0..0/1] 72 bytes: 26.04,34.05,940.90,...  (sent)
[stat] STAT,94,8,0,11100,0,2236,-1  (sent)
[sleep] tick work done - sleeping 10s
```

The LED says the same thing without a cable: **green wave** on every wake, **blue**
while reading, **white** while transmitting, **red** if a sensor that was expected
did not answer, **off** asleep.
