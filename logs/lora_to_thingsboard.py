"""Module B -> ThingsBoard bridge, over this PC's internet instead of NB-IoT.

Module B's BC660K-GL answers nothing - no bytes on any pin order, any baud, with
or without the channel gate, and identically on pre-change firmware. Without AT
there is no MQTT, so the cellular path cannot be revived in software.

But the LoRa half works perfectly: readings are arriving at Module B over USB
right now. This reads them from that same serial port and publishes them to the
SAME ThingsBoard broker, with the SAME gateway token and the SAME telemetry keys
the firmware would have used - so from ThingsBoard's side the data is
indistinguishable from a working NB-IoT uplink.

Matches src/module_b/communications/modem_nbiot_mqtt.cpp exactly:
  topic    v1/gateway/telemetry
  auth     access token as MQTT username, no password
  device   the gateway ("CON-1"), which must have "Is gateway" enabled
  keys     temp, rh, pres, gas, pm1, pm25, pm10, co2, co, coTemp,
           windAngle, windSpeed, windValid, rssi, snr

PAYLOAD SHAPE - the rule that cost a day on 2026-08-28: a "values" wrapper is
valid ONLY together with a "ts". The firmware has no RTC so it sends the flat
form; THIS BRIDGE HAS A REAL CLOCK, so it sends {"ts":..,"values":{..}}, which is
the correct paired form and preserves when a reading was actually taken - including
backdating batched readings by their age in ticks. Sending "values" WITHOUT "ts"
is silently dropped server-side while the publish still reports success.
"""
import json
import re
import sys
import time
from datetime import datetime, timedelta, timezone

import serial

try:
    import paho.mqtt.client as mqtt
except ImportError:
    sys.exit("paho-mqtt is not installed. Run:  pip install paho-mqtt")

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM10"
BAUD = 115200

# Straight from src/module_b/communications/Config.h - do not diverge.
BROKER_HOST = "test-moduloa.home.kg"
BROKER_PORT = 18831  # NOT 1883: that port returns a well-formed CONNACK 5
                     # ("not authorized") with the identical token, which looks
                     # exactly like a credentials problem. Confirmed 2026-08-19.
CLIENT_ID = "CON-1"
ACCESS_TOKEN = "QRPJgyk5COJPCavycmpp"
TOPIC = "v1/gateway/telemetry"

TICK_SECONDS = 10  # one Module C sleep tick, for backdating batched readings

NODE_NAMES = {"1": "NodoC-1", "3": "NodoC-2", "4": "NodoC-3"}

# Order matters: matches the CSV field order Module C transmits.
KEYS = ["temp", "rh", "pres", "gas", "pm1", "pm25", "pm10",
        "co2", "co", "coTemp", "windAngle", "windSpeed", "windValid"]

RCV_RE = re.compile(r'\+RCV addr=(\d+) len=(\d+) rssi=(-?\d+) snr=(-?\d+) data="(.*)"')


def node_name(addr):
    return NODE_NAMES.get(str(addr), "NodoDesconocido-%s" % addr)


def to_values(fields, rssi, snr):
    """13 CSV fields -> the telemetry dict, with windValid as a real bool."""
    out = {}
    for key, raw in zip(KEYS, fields):
        if key == "windValid":
            out[key] = raw.strip() not in ("0", "0.0", "0.00", "")
        else:
            try:
                out[key] = float(raw)
            except ValueError:
                out[key] = 0.0
    out["rssi"] = int(rssi)
    out["snr"] = int(snr)
    return out


def readings_from(data, rssi, snr, now):
    """(timestamp, values) per reading. Handles both single and batch payloads;
    a batch record's trailing age-in-ticks backdates it to when it was taken."""
    if data.startswith("ACK,"):
        return []

    if data.startswith("B,"):
        head = data.split(",", 2)
        if len(head) < 3:
            return []
        out = []
        for chunk in head[2].split(";"):
            chunk = chunk.strip()
            if not chunk:
                continue
            f = chunk.split(",")
            if len(f) != 14:
                continue
            try:
                age = int(f[13])
            except ValueError:
                age = 0
            ts = now - timedelta(seconds=age * TICK_SECONDS)
            out.append((ts, to_values(f[:13], rssi, snr)))
        return out

    f = data.split(",")
    if len(f) == 13:
        return [(now, to_values(f, rssi, snr))]
    return []


def open_port():
    while True:
        try:
            return serial.Serial(PORT, BAUD, timeout=1)
        except serial.SerialException as e:
            print("serial open failed (%s), retrying in 3s..." % e, flush=True)
            time.sleep(3)


def main():
    client = mqtt.Client(client_id=CLIENT_ID)
    # ThingsBoard access-token auth: token as username, no password.
    client.username_pw_set(ACCESS_TOKEN)

    def on_connect(c, userdata, flags, rc):
        if rc == 0:
            print("MQTT connected to %s:%d as %s" % (BROKER_HOST, BROKER_PORT, CLIENT_ID),
                  flush=True)
        else:
            # rc 5 on port 1883 is the classic wrong-port symptom, not a bad token.
            print("MQTT CONNACK %d (5=not authorized: check token AND that the "
                  "device has 'Is gateway' enabled)" % rc, flush=True)

    client.on_connect = on_connect
    client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
    client.loop_start()

    ser = open_port()
    print("Bridging %s -> %s" % (PORT, TOPIC), flush=True)

    sent = 0
    while True:
        try:
            raw = ser.readline()
        except serial.SerialException as e:
            print("serial error (%s) - reopening..." % e, flush=True)
            try:
                ser.close()
            except Exception:
                pass
            time.sleep(2)
            ser = open_port()
            continue

        if not raw:
            continue
        line = raw.decode(errors="replace").rstrip()
        m = RCV_RE.search(line)
        if not m:
            continue

        addr, _length, rssi, snr, data = m.groups()
        now = datetime.now(timezone.utc)
        items = readings_from(data, rssi, snr, now)
        if not items:
            continue

        name = node_name(addr)
        # "ts" and "values" together - see the module docstring.
        payload = {name: [{"ts": int(ts.timestamp() * 1000), "values": v} for ts, v in items]}
        info = client.publish(TOPIC, json.dumps(payload), qos=1)
        info.wait_for_publish(timeout=10)
        sent += len(items)
        print("-> %s: %d reading(s) published (total %d)" % (name, len(items), sent), flush=True)


if __name__ == "__main__":
    main()
