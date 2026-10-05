"""Show where ThingsBoard actually stored the commandLog records.

WHY THIS EXISTS: Module B publishes the outcome of every A->C command as a
`commandLog` telemetry record, and its serial log confirms the broker accepted
the publish (`+QMTPUB: 0,0,0`). Despite that, the Module A dashboard can sit at
"en tr\u00e1nsito al MODULO-C" forever. Three different things produce that same
symptom and they need opposite fixes:

  1. The record IS on the gateway device (CON-1) and the dashboard widget is
     bound to the NODE instead. Fix on the dashboard - no firmware change.
  2. The record is nowhere. ThingsBoard rejected the publish despite acking it
     at the MQTT layer, e.g. the key is being written by a token that may not.
     Fix in Module B.
  3. The record is there and CURRENT, and the dashboard is simply stale. Fix by
     refreshing.

Guessing between these is what we have been doing. This asks the server.

`commandLog` is published to v1/devices/me/telemetry, which with the gateway's
access token means it lands on the GATEWAY device, CON-1 - NOT on NodoC-3. That
asymmetry is the leading suspect, and this script proves or kills it in one run.

The password is a command-line argument and is never stored or sent anywhere but
your own ThingsBoard. Run it yourself; nothing here phones home.

Usage:
    python read_commandlog.py --user you@example.com --password SECRET

    # also dump every telemetry key on each device, not just the command ones
    python read_commandlog.py --user ... --password ... --all-keys
"""

import argparse
import json
import sys
import time
import urllib.error
import urllib.parse
import urllib.request


def post(url, payload, token=None):
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=data, method="POST")
    req.add_header("Content-Type", "application/json")
    if token:
        req.add_header("X-Authorization", "Bearer " + token)
    with urllib.request.urlopen(req, timeout=30) as r:
        body = r.read().decode("utf-8")
        return json.loads(body) if body.strip() else {}


def get(url, token):
    req = urllib.request.Request(url, method="GET")
    req.add_header("X-Authorization", "Bearer " + token)
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read().decode("utf-8"))


def find_device(base, token, name):
    """Resolve a device name to its id, or None."""
    url = base + "/api/tenant/devices?pageSize=200&page=0&textSearch=" + \
        urllib.parse.quote(name)
    try:
        page = get(url, token)
    except urllib.error.HTTPError as e:
        print("  device lookup failed: %s" % e.code)
        return None
    for d in page.get("data", []):
        if d.get("name") == name:
            return d["id"]["id"]
    return None


def report(base, token, name, want_all):
    print("")
    print("=== %s ===" % name)
    dev_id = find_device(base, token, name)
    if not dev_id:
        print("  NOT FOUND as a device on this tenant")
        return

    keys_url = base + "/api/plugins/telemetry/DEVICE/%s/keys/timeseries" % dev_id
    try:
        keys = get(keys_url, token)
    except urllib.error.HTTPError as e:
        print("  key listing failed: %s" % e.code)
        return

    interesting = [k for k in keys if "command" in k.lower() or "cmd" in k.lower()]
    if not interesting:
        print("  no commandLog-like key here.  keys present: %s" %
              (", ".join(sorted(keys)) if keys else "(none)"))
        if not want_all:
            return
    else:
        print("  command keys: %s" % ", ".join(sorted(interesting)))

    show = sorted(keys) if want_all else sorted(interesting)
    if not show:
        return

    val_url = base + "/api/plugins/telemetry/DEVICE/%s/values/timeseries?keys=%s" % (
        dev_id, urllib.parse.quote(",".join(show)))
    try:
        vals = get(val_url, token)
    except urllib.error.HTTPError as e:
        print("  value read failed: %s" % e.code)
        return

    now_ms = int(time.time() * 1000)
    for k in show:
        for entry in vals.get(k, []):
            ts = entry.get("ts", 0)
            age = (now_ms - ts) / 1000.0
            stamp = time.strftime("%d/%m %H:%M:%S", time.localtime(ts / 1000.0))
            print("    %-14s %s  (%.0fs ago)" % (k, stamp, age))
            print("      %s" % entry.get("value"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="test-moduloa.home.kg")
    ap.add_argument("--scheme", default="https", choices=["http", "https"])
    ap.add_argument("--user", required=True)
    ap.add_argument("--password", required=True)
    ap.add_argument("--gateway", default="CON-1",
                    help="gateway device name - where commandLog SHOULD be")
    ap.add_argument("--nodes", default="NodoC-1,NodoC-2,NodoC-3",
                    help="child devices to also check, comma separated")
    ap.add_argument("--all-keys", action="store_true",
                    help="list every telemetry key, not just the command ones")
    args = ap.parse_args()

    base = "%s://%s" % (args.scheme, args.host)
    print("logging in to %s ..." % base, flush=True)
    try:
        auth = post(base + "/api/auth/login",
                    {"username": args.user, "password": args.password})
    except urllib.error.HTTPError as e:
        sys.exit("login failed: %s %s" % (e.code, e.read().decode("utf-8", "replace")))
    token = auth.get("token")
    if not token:
        sys.exit("login returned no token: %r" % auth)

    report(base, token, args.gateway, args.all_keys)
    for n in [x.strip() for x in args.nodes.split(",") if x.strip()]:
        report(base, token, n, args.all_keys)

    print("")
    print("HOW TO READ THIS")
    print("  commandLog on %s, timestamp recent  -> the dashboard widget is bound" % args.gateway)
    print("     to the wrong device, or is stale. Fix is on Module A, not in firmware.")
    print("  commandLog NOWHERE                  -> ThingsBoard is dropping the")
    print("     publish even though MQTT acked it. Fix is in Module B.")
    print("  commandLog on a NodoC-* device      -> it is already going to the node;")
    print("     point the widget there.")


if __name__ == "__main__":
    main()
