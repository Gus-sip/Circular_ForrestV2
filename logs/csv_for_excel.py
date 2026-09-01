"""
Convert module_b_lora_log.csv into a spreadsheet-friendly copy for a
Spanish-locale Excel: ';' column separator, ',' decimal separator, and the
ISO timestamp flattened to 'YYYY-MM-DD HH:MM:SS'.

  python logs/csv_for_excel.py            # UTC timestamps (as logged)
  python logs/csv_for_excel.py --local    # shift to Spain summer time (UTC+2, CEST)

Output: logs/module_b_lora_log_excel.csv  (double-click to open)

Note: --local applies a flat +2h (CEST). The whole log so far is inside summer
time; if it ever spans the late-October DST change, split it or adjust here.
"""

import csv
import os
import sys
from datetime import datetime, timedelta, timezone

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "module_b_lora_log.csv")
DST = os.path.join(HERE, "module_b_lora_log_excel.csv")

to_local = "--local" in sys.argv
tz = timezone(timedelta(hours=2))  # CEST (Spain summer time)

with open(SRC, newline="") as fin, open(DST, "w", newline="") as fout:
    reader = csv.reader(fin)
    writer = csv.writer(fout, delimiter=";")
    header = next(reader)
    writer.writerow(header)  # first column stays "timestamp"
    n = 0
    for row in reader:
        if not row:
            continue
        try:
            dt = datetime.fromisoformat(row[0])
            if to_local:
                dt = dt.astimezone(tz)
            row[0] = dt.strftime("%Y-%m-%d %H:%M:%S")
        except ValueError:
            pass  # leave a malformed timestamp untouched
        # dot -> comma on every remaining numeric cell
        row = [row[0]] + [c.replace(".", ",") for c in row[1:]]
        writer.writerow(row)
        n += 1

kind = "Europe/Madrid local" if to_local else "UTC"
print(f"wrote {n} rows to {DST}  ({kind} timestamps, ';' sep, ',' decimals)")
