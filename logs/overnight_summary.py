"""Compact digest of an overnight Module B capture.

Reads module_b_lora_log.csv and prints ~15 lines: how many packets arrived, how
regularly, whether any went missing, and the range each sensor covered. The point
is to answer "did it run all night" without anyone reading thousands of rows.

    python logs/overnight_summary.py            # whole file
    python logs/overnight_summary.py 12         # last 12 hours only

Gaps are the interesting part. Node C2 transmits on a fixed tick, so a run of
evenly spaced packets means the sleep cycle held; a long gap means it stopped and
restarted, and the gap's position says when.
"""

import csv
import os
import sys
from datetime import datetime, timedelta

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(HERE, "module_b_lora_log.csv")

# Anything longer than this between packets is reported individually - a missed
# packet or two is normal on a radio link; a minutes-long hole is not.
GAP_ALERT_S = 120

NUMERIC = ["temp", "hum", "pres", "gas", "pm1", "pm25", "pm10", "co2", "co", "rssi", "snr"]


def parse_ts(s):
    for fmt in ("%Y-%m-%dT%H:%M:%S.%f%z", "%Y-%m-%dT%H:%M:%S%z", "%Y-%m-%d %H:%M:%S"):
        try:
            return datetime.strptime(s, fmt)
        except ValueError:
            continue
    return None


def main():
    if not os.path.exists(CSV_PATH):
        print("No log file at", CSV_PATH)
        return

    hours = float(sys.argv[1]) if len(sys.argv) > 1 else None

    with open(CSV_PATH, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        print("Log file is empty - the logger ran but no packets were parsed.")
        return

    stamped = [(parse_ts(r.get("timestamp", "")), r) for r in rows]
    stamped = [(t, r) for t, r in stamped if t is not None]
    if not stamped:
        print("%d rows, but no parseable timestamps." % len(rows))
        return

    if hours:
        cutoff = stamped[-1][0] - timedelta(hours=hours)
        stamped = [(t, r) for t, r in stamped if t >= cutoff]

    first, last = stamped[0][0], stamped[-1][0]
    span = (last - first).total_seconds()
    n = len(stamped)

    print("=" * 58)
    print("Module B overnight capture")
    print("=" * 58)
    print("packets   : %d" % n)
    print("first     : %s" % first.strftime("%Y-%m-%d %H:%M:%S"))
    print("last      : %s" % last.strftime("%Y-%m-%d %H:%M:%S"))
    print("span      : %.1f h" % (span / 3600.0))
    if n > 1 and span > 0:
        print("mean gap  : %.0f s  (expected ~30s at sensor_read=3/lora_trans=3)"
              % (span / (n - 1)))

    # Gaps: where did it stop?
    gaps = []
    for i in range(1, len(stamped)):
        dt = (stamped[i][0] - stamped[i - 1][0]).total_seconds()
        if dt > GAP_ALERT_S:
            gaps.append((stamped[i - 1][0], stamped[i][0], dt))
    print()
    if gaps:
        print("GAPS over %ds: %d" % (GAP_ALERT_S, len(gaps)))
        for a, b, dt in gaps[:10]:
            print("  %s -> %s   %.0f min" % (a.strftime("%H:%M:%S"), b.strftime("%H:%M:%S"),
                                             dt / 60.0))
        if len(gaps) > 10:
            print("  ... and %d more" % (len(gaps) - 10))
    else:
        print("GAPS over %ds: none - the link held for the whole span." % GAP_ALERT_S)

    # Value ranges: did anything flatline or go out of range?
    print()
    print("%-8s %10s %10s %10s" % ("field", "min", "max", "last"))
    print("-" * 42)
    for key in NUMERIC:
        vals = []
        for _, r in stamped:
            try:
                vals.append(float(r.get(key, "")))
            except (TypeError, ValueError):
                pass
        if not vals:
            continue
        flat = "  FLAT" if min(vals) == max(vals) else ""
        print("%-8s %10.2f %10.2f %10.2f%s" % (key, min(vals), max(vals), vals[-1], flat))

    print()
    print("FLAT means the value never changed across the whole span - for a live")
    print("sensor that is a fault, though co/wind are expected flat (SEN0466 is")
    print("skipped and no Calypso is fitted).")


if __name__ == "__main__":
    main()
