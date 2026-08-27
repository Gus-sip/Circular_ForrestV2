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

s = serial.Serial(PORT, BAUD, timeout=1)
print(f"Logging Module B ({PORT}) LoRa receptions to {OUT_PATH}", flush=True)

with open(OUT_PATH, "a", newline="") as f:
    writer = csv.writer(f)
    if is_new_file:
        writer.writerow(FIELDS)
        f.flush()

    while True:
        raw = s.readline()
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
