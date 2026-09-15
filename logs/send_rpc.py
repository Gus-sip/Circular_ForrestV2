"""Send a cfg RPC to a Module C node, via ThingsBoard, from the command line.

ThingsBoard CE has no "send RPC" button on the device page - it is a REST call or
a dashboard widget. This is the REST call, wrapped up.

The chain this exercises:

    this script -> ThingsBoard -> MQTT -> Module B -> LoRa -> Module C
                                                        <- ACK

Module B relays the RPC's `params` object as the "CFG,K=V,..." string Module C's
applyConfigCommand() parses, and the node ACKs only what it actually APPLIED - so
an ACK is proof the setting took effect, not merely that the packet arrived.

Two things to expect:

  * It is NOT instant. The node is deaf except for a ~2 second window right after
    each transmit, so delivery waits for its next check-in - up to a full transmit
    cycle. A ThingsBoard RPC timeout is therefore not proof of failure; watch
    Module B's serial for the ACK instead.

  * The method MUST be "cfg". Module B rejects anything else with
    "unknown method, expected 'cfg'".

Usage:
    python send_rpc.py --host test-moduloa.home.kg --user you@example.com \
        --password SECRET --device NodoC-1 --set SENSOR_READ=6

    python send_rpc.py ... --set SENSOR_READ=6 --set LORA_TRANS=8
"""
import argparse
import json
import sys
import urllib.error
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="test-moduloa.home.kg",
                    help="ThingsBoard host (no scheme)")
    ap.add_argument("--scheme", default="https", choices=["http", "https"])
    ap.add_argument("--user", required=True, help="ThingsBoard login")
    ap.add_argument("--password", required=True)
    ap.add_argument("--device", default="NodoC-1",
                    help="CHILD device name, e.g. NodoC-1 - not the gateway CON-1")
    ap.add_argument("--set", action="append", required=True, metavar="KEY=VALUE",
                    help="config to apply, repeatable")
    ap.add_argument("--oneway", action="store_true",
                    help="fire and forget instead of waiting for the node's reply")
    args = ap.parse_args()

    base = "%s://%s" % (args.scheme, args.host)

    params = {}
    for pair in args.set:
        if "=" not in pair:
            sys.exit("--set expects KEY=VALUE, got %r" % pair)
        k, v = pair.split("=", 1)
        # Values go across as strings; Module C parses them itself.
        params[k.strip()] = v.strip()

    print("logging in to %s ..." % base, flush=True)
    try:
        auth = post(base + "/api/auth/login",
                    {"username": args.user, "password": args.password})
    except urllib.error.HTTPError as e:
        sys.exit("login failed: %s %s" % (e.code, e.read().decode("utf-8", "replace")))
    token = auth.get("token")
    if not token:
        sys.exit("login returned no token: %r" % auth)

    print("looking up device %r ..." % args.device, flush=True)
    try:
        dev = get(base + "/api/tenant/devices?deviceName=" + args.device, token)
    except urllib.error.HTTPError as e:
        sys.exit("device lookup failed: %s - is %r the exact device name? "
                 "It is auto-created by the first telemetry batch."
                 % (e.code, args.device))
    device_id = dev["id"]["id"]
    print("  id = %s" % device_id, flush=True)

    body = {"method": "cfg", "params": params}
    kind = "oneway" if args.oneway else "twoway"
    url = "%s/api/rpc/%s/%s" % (base, kind, device_id)

    print("sending %s RPC: %s" % (kind, json.dumps(body)), flush=True)
    try:
        resp = post(url, body, token)
        print("ThingsBoard replied: %s" % json.dumps(resp), flush=True)
    except urllib.error.HTTPError as e:
        detail = e.read().decode("utf-8", "replace")
        # 408 here means the node did not answer inside ThingsBoard's RPC window.
        # That is expected on a sleeping node and does NOT mean the command was
        # lost - Module B delivers it at the node's next check-in.
        print("HTTP %s: %s" % (e.code, detail), flush=True)
        if e.code in (408, 504):
            print("\nThat is a TIMEOUT, not necessarily a failure: the node only "
                  "listens for ~2s after each transmit, so delivery can take a "
                  "full cycle. Watch Module B's serial for:\n"
                  "    -> downlink to %s: CFG,... (sent)\n"
                  "    +RCV ... data=\"ACK,...\"     <- applied" % args.device,
                  flush=True)
        return

    print("\nNow confirm on Module B's serial:\n"
          "    -> downlink to %s: CFG,... (sent)   <- relayed over LoRa\n"
          "    +RCV ... data=\"ACK,...\"            <- node applied it"
          % args.device, flush=True)


if __name__ == "__main__":
    main()
