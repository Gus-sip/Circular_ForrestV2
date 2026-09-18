# Plug-and-play flashing

One file per node. **No compiler, no libraries, no build** — everything (bootloader,
partition table, OTA selector and application) is merged into a single image that
goes to offset `0x0`.

| File | LoRa address | Appears in ThingsBoard as |
|---|---|---|
| `CHIP-FOREST_node1.bin` | 1 | `NodoC-1` |
| `CHIP-FOREST_node2.bin` | 3 | `NodoC-2` |

**Address 2 is Module B** and is never given to a node.

---

## Flashing

The only prerequisite is `esptool`:

```
pip install esptool
```

Then, with the board plugged in:

```
esptool.py --chip esp32s3 --port COM14 --baud 460800 write_flash 0x0 CHIP-FOREST_node1.bin
```

Replace `COM14` with the board's port (Windows: Device Manager; Linux/macOS:
`/dev/ttyACM0`, `/dev/cu.usbmodem*`). On Linux/macOS the command is the same.

That is the whole procedure.

---

## The one rule

**Each physical board must get a different file.** Two nodes on the same address
means Module B cannot tell them apart, and their readings merge into one device —
silently, with no error at either end. The data simply becomes two sensors averaged
into one, which is worse than losing it.

**Write the address on the board** as you flash it. The boards are identical and
the COM port follows the USB socket, not the board.

Need more than three nodes? They need new addresses, which means a rebuild from
`../` — see the parent `README.md`. These binaries have their address compiled in.

---

## If the upload fails

**"the port is busy or doesn't exist"** on a board that already has firmware: this
is normal. Deep sleep drops the USB port, so it only exists for a second or two per
wake. Either retry until a wake catches it, or power-cycle the board and flash
within 30 seconds — a cold boot deliberately stays awake that long to be
reprogrammable.

**A brand-new board** should flash first time; it has nothing to sleep.

---

## Checking it worked

Open the serial port at **115200**. A healthy node prints, once per tick:

```
Node address: 1 (Module B sees this as +RCV addr=1)
[sentinel] bme=OK(T26.1 H34.0) co=NotInit(0.00)  (40ms)
TX [0..0/1] 72 bytes: 26.04,34.05,940.90,...  (sent)
[sleep] tick work done - sleeping 10s
```

Check the **`Node address:`** line matches the file you flashed.

Without a cable, the LED says the same thing: **green wave** every wake, **blue**
while reading sensors, **white** while transmitting, **red** if a sensor that was
expected did not answer, **off** while asleep.
