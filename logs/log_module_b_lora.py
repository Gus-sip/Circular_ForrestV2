import csv
import os
import re
import sys
import time
from datetime import datetime, timezone

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM10"
BAUD = 115200
OUT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "module_b_lora_log.csv")

FIELDS = [
    "timestamp", "senderAddr", "rssi", "snr",
    "temp", "hum", "pres", "gas", "pm1", "pm25", "pm10",
    "co2", "co", "coTemp", "windAngle", "windSpeed", "windValid",
]

RCV_RE = re.compile(r'\+RCV addr=(\d+) len=(\d+) rssi=(-?\d+) snr=(-?\d+) data="(.*)"')

is_new_file = not os.path.exists(OUT_PATH)


def open_port():
    while True:
        try:
            return serial.Serial(PORT, BAUD, timeout=1)
        except serial.SerialException as e:
            print(f"open failed ({e}), retrying in 3s...", flush=True)
            time.sleep(3)


with open(OUT_PATH, "a", newline="") as f:
    writer = csv.writer(f)
    if is_new_file:
        writer.writerow(FIELDS)
        f.flush()

    s = open_port()
    print(f"Logging Module B ({PORT}) LoRa receptions to {OUT_PATH}", flush=True)

    while True:
        # Windows' USB-CDC serial driver on this machine occasionally drops
        # into a "device doesn't recognize the command" state mid-read
        # (same class of glitch seen on other boards' ports this session) -
        # not a data-corruption issue, just a transient driver hiccup. Rather
        # than let that kill the logger, close and reopen the port and keep
        # going; nothing is lost except whatever reading arrived during the
        # brief gap.
        try:
            raw = s.readline()
        except serial.SerialException as e:
            print(f"serial error ({e}) - reopening port...", flush=True)
            try:
                s.close()
            except Exception:
                pass
            time.sleep(2)
            s = open_port()
            print("port reopened, resuming.", flush=True)
            continue

        if not raw:
            continue
        try:
            line = raw.decode(errors="replace").rstrip()
        except Exception:
            continue

        m = RCV_RE.search(line)
        if not m:
            continue

        addr, length, rssi, snr, data = m.groups()
        parts = data.split(",")
        if len(parts) != 13:
            print(f"skipped (expected 13 fields, got {len(parts)}): {line}", flush=True)
            continue

        ts = datetime.now(timezone.utc).isoformat()
        row = [ts, addr, rssi, snr] + parts
        writer.writerow(row)
        f.flush()
        print(f"logged: addr={addr} rssi={rssi} snr={snr} data={data}", flush=True)
