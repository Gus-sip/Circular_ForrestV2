# How to Flash Module C and Module B

Step-by-step instructions for flashing the two boards in this project. If
you just want the short version: pick your board below, follow either the
VS Code steps or the terminal command, done.

## Which firmware am I flashing?

Each board has **one real shipping firmware** - don't confuse it with the
bring-up/test envs also in `platformio.ini` (those are for isolating one
sensor/radio at a time, not for the finished board):

| Board | Real firmware file | PlatformIO env |
|---|---|---|
| Module C (sensor node) | `src/module_c/communications/chip_forest_lora_tx.cpp` | `module-c-lora-tx` |
| Module B (relay) | `src/module_b/communications/main.cpp` | `module-b-main` |

If you're not sure which board is which: Module C has the five environmental
sensors wired to it and transmits over LoRa. Module B has no sensors - it
just receives LoRa and forwards over NB-IoT/MQTT to ThingsBoard, and (once
wired up) drives the OLED status display.

## Before you start

1. Plug the board into your PC over USB.
2. Find out which COM port it showed up as. Open a terminal in this project
   folder and run:
   ```
   pio device list
   ```
   Look for a device with a **USB VID:PID** line (that's the ESP32's own USB
   port) - note its COM number (e.g. `COM3`, `COM10`). If two boards are
   plugged in at once, you'll see two entries - unplug one if you're not
   sure which is which, or check the hardware ID against what you plugged in
   last.

   If `pio` isn't recognized as a command, either:
   - Use the full path instead: `C:\Users\<you>\.platformio\penv\Scripts\pio.exe`
   - Or open a terminal from inside the PlatformIO extension in VS Code
     (its integrated terminal already has `pio` on PATH) - see below.

## Option 1: VS Code PlatformIO extension (easiest)

1. Open this project folder in VS Code (the PlatformIO extension must be
   installed).
2. Look at the **bottom status bar** - there's an environment selector
   showing the currently active env (e.g. `module-c-main` or similar).
   Click it and choose the env you want from the list:
   - `module-c-lora-tx` for Module C
   - `module-b-main` for Module B
3. Make sure the right board is plugged in over USB.
4. Click the **→ (right-pointing arrow / upload)** icon in the same status
   bar area. This builds the firmware and flashes it in one step - watch the
   bottom panel for `SUCCESS`.
5. Click the **plug icon** right next to it to open the serial monitor and
   watch the board boot (115200 baud). Ctrl+C or click the trash icon to
   close the monitor when you're done.

If PlatformIO uploads to the wrong board (you have more than one plugged in),
set the port explicitly: click the alien-head PlatformIO icon in the left
sidebar → **PROJECT TASKS** → pick the env → **Advanced → Upload Port** (or
just unplug the board you don't want flashed).

## Option 2: Terminal, typed yourself

From this project's root folder:

```
pio run -e module-c-lora-tx -t upload
```
or
```
pio run -e module-b-main -t upload
```

If more than one board is connected and it picks the wrong one, pin the
port explicitly (get the port name from `pio device list` first):

```
pio run -e module-c-lora-tx -t upload --upload-port COM3
```

To watch the serial output afterward (115200 baud):

```
pio device monitor -p COM3 -b 115200
```

Or do build+upload+monitor in one shot:

```
pio run -e module-b-main -t upload -t monitor --upload-port COM3
```

## Troubleshooting

- **Upload fails / times out**: wrong COM port, or the board's USB-CDC isn't
  enumerating. Unplug/replug, re-check `pio device list`, and make sure
  nothing else (like the Arduino IDE's Serial Monitor) has the port open.
- **Uploaded fine but nothing happens on serial**: you may be watching the
  wrong port, or the board needs a manual reset button press right after
  upload starts on some ESP32-S3 boards.
- **Board not detected on ANY port**: check the USB cable is data-capable
  (not charge-only), and try a different USB port.
- **Wrong firmware behavior after flashing**: double check you picked
  `module-c-lora-tx` / `module-b-main` and not one of the bring-up/test envs
  (e.g. `module-c-bme690-test`, `module-b-radio-bridge`) - see the comment
  above each `[env:...]` block in `platformio.ini` for what each one is for.
