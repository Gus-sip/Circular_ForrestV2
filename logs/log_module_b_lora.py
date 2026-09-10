import csv
import os
import re
import sys
import time
from datetime import datetime, timedelta, timezone

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

# One Module C sleep tick. Batched readings carry their age in ticks, so this is
# what converts that age back into seconds for the timestamp.
TICK_SECONDS = 10

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

        # Two payload shapes now arrive, because Module C batches when more than
        # one reading has accumulated since the last transmit:
        #
        #   single  "<13 comma-separated fields>"
        #   batch   "B,<count>,<14 fields>;<14 fields>;..."
        #
        # The batch form was previously skipped outright by the 13-field check,
        # which quietly dropped roughly one transmit in three - the read and
        # transmit schedules (every 3 ticks and every 4) realign every 12 ticks and
        # that tick sends two readings.
        #
        # A batch record carries one extra trailing field: its age in 10s ticks at
        # the moment of transmit. That is used to BACKDATE the timestamp rather
        # than being stored, so every row keeps the same 17 columns and a batched
        # reading lands at the time it was actually taken instead of the time it
        # happened to be sent.
        records = []  # (fields13, age_ticks)
        if data.startswith("B,"):
            body = data.split(",", 2)
            if len(body) < 3:
                print(f"skipped (malformed batch header): {line}", flush=True)
                continue
            for chunk in body[2].split(";"):
                chunk = chunk.strip()
                if not chunk:
                    continue
                fields = chunk.split(",")
                if len(fields) != 14:
                    print(f"skipped batch record (expected 14 fields, got "
                          f"{len(fields)}): {chunk}", flush=True)
                    continue
                try:
                    age = int(fields[13])
                except ValueError:
                    age = 0
                records.append((fields[:13], age))
        else:
            parts = data.split(",")
            if len(parts) != 13:
                print(f"skipped (expected 13 fields, got {len(parts)}): {line}", flush=True)
                continue
            records.append((parts, 0))

        now = datetime.now(timezone.utc)
        for fields, age in records:
            ts = (now - timedelta(seconds=age * TICK_SECONDS)).isoformat()
            writer.writerow([ts, addr, rssi, snr] + fields)
        f.flush()
        print(f"logged {len(records)} reading(s): addr={addr} rssi={rssi} snr={snr}",
              flush=True)
