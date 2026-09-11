"""Record ALL LoRa communications seen by Module B into logs/porthole_data.csv.

Deliberately a superset of log_module_b_lora.py, which only ever wrote rows it
could parse into exactly 13 telemetry fields and silently dropped everything else
- batches, ACKs, downlinks and malformed frames all vanished without trace. A log
that quietly discards what it does not understand is the same failure mode as a
sensor cache that keeps its last good value: it cannot tell you that something
went wrong, only that nothing changed.

So every LoRa line is written here, parsed or not:

  kind=single    one reading, 13 comma-separated fields
  kind=batch     "B,<n>,<14 fields>;<14 fields>;..." - ONE ROW PER READING
  kind=ack       a node's "ACK,<seq>" reply to a config push
  kind=downlink  a CFG command Module B sent DOWN to a node
  kind=other     anything else that arrived - kept verbatim rather than dropped

The raw payload is always kept, so even an unparseable frame is recoverable.

Timestamps: a batch record carries its age in 10s ticks at the moment of
transmit, so `timestamp` is backdated by that age to when the reading was
actually TAKEN, while `received_at` records when Module B heard it. For single
readings the two are equal.
"""
import csv
import os
import re
import sys
import time
from datetime import datetime, timedelta, timezone

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM10"
BAUD = 115200
OUT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "porthole_data.csv")

# One Module C sleep tick, for converting a batch record's age back to seconds.
TICK_SECONDS = 10

FIELDS = [
    "timestamp", "received_at", "direction", "kind",
    "addr", "rssi", "snr", "len",
    "batch_index", "batch_count", "age_ticks",
    "temp", "hum", "pres", "gas", "pm1", "pm25", "pm10",
    "co2", "co", "coTemp", "windAngle", "windSpeed", "windValid",
    "raw",
]

RCV_RE = re.compile(r'\+RCV addr=(\d+) len=(\d+) rssi=(-?\d+) snr=(-?\d+) data="(.*)"')
DOWNLINK_RE = re.compile(r'-> downlink to (\S+): (.*?) \((sent|SEND FAILED)\)')

is_new_file = not os.path.exists(OUT_PATH)


def open_port():
    while True:
        try:
            return serial.Serial(PORT, BAUD, timeout=1)
        except serial.SerialException as e:
            print(f"open failed ({e}), retrying in 3s...", flush=True)
            time.sleep(3)


def blank_row(**kw):
    row = {k: "" for k in FIELDS}
    row.update(kw)
    return row


def rows_for_payload(addr, length, rssi, snr, data, now):
    """Every row this payload yields. Never returns empty - an unparseable frame
    still produces one row, so nothing is lost."""
    common = dict(received_at=now.isoformat(), direction="RX", addr=addr,
                  rssi=rssi, snr=snr, len=length, raw=data)

    if data.startswith("ACK,"):
        return [blank_row(timestamp=now.isoformat(), kind="ack", **common)]

    if data.startswith("B,"):
        head = data.split(",", 2)
        if len(head) < 3:
            return [blank_row(timestamp=now.isoformat(), kind="other", **common)]
        chunks = [c for c in head[2].split(";") if c.strip()]
        out = []
        for i, chunk in enumerate(chunks):
            f = chunk.split(",")
            if len(f) != 14:
                out.append(blank_row(timestamp=now.isoformat(), kind="other", **common))
                continue
            try:
                age = int(f[13])
            except ValueError:
                age = 0
            ts = now - timedelta(seconds=age * TICK_SECONDS)
            r = blank_row(timestamp=ts.isoformat(), kind="batch", batch_index=i,
                          batch_count=len(chunks), age_ticks=age, **common)
            for name, val in zip(FIELDS[11:24], f[:13]):
                r[name] = val
            out.append(r)
        return out or [blank_row(timestamp=now.isoformat(), kind="other", **common)]

    parts = data.split(",")
    if len(parts) == 13:
        r = blank_row(timestamp=now.isoformat(), kind="single", **common)
        for name, val in zip(FIELDS[11:24], parts):
            r[name] = val
        return [r]

    return [blank_row(timestamp=now.isoformat(), kind="other", **common)]


with open(OUT_PATH, "a", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=FIELDS)
    if is_new_file:
        writer.writeheader()
        f.flush()

    s = open_port()
    print(f"Recording ALL Module B ({PORT}) LoRa traffic to {OUT_PATH}", flush=True)

    while True:
        # The USB-CDC driver on this machine intermittently drops into a
        # "device doesn't recognize the command" state mid-read. Reopen and carry
        # on rather than letting a transient driver hiccup kill the recording.
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

        now = datetime.now(timezone.utc)

        m = RCV_RE.search(line)
        if m:
            addr, length, rssi, snr, data = m.groups()
            rows = rows_for_payload(addr, length, rssi, snr, data, now)
            for r in rows:
                writer.writerow(r)
            f.flush()
            kinds = ",".join(sorted({r["kind"] for r in rows}))
            print(f"RX {kinds}: addr={addr} rssi={rssi} snr={snr} -> {len(rows)} row(s)",
                  flush=True)
            continue

        d = DOWNLINK_RE.search(line)
        if d:
            node, cfg, result = d.groups()
            writer.writerow(blank_row(timestamp=now.isoformat(), received_at=now.isoformat(),
                                      direction="TX", kind="downlink", addr=node,
                                      raw=f"{cfg} ({result})"))
            f.flush()
            print(f"TX downlink to {node}: {cfg} ({result})", flush=True)
