"""
Publishes one test telemetry reading to Module A's ThingsBoard exactly the way
Module B's firmware does - same broker, port, gateway token, topic, and (as of
2026-08-28) the same FLAT payload shape (no "values" wrapper, no "ts").

Purpose: confirm the gateway path end-to-end from the dev PC before reflashing
Module B. If this makes a `NodoC-1` device appear in ThingsBoard with the values
below, the firmware fix is correct and the only remaining variable is the modem.

Usage:
    pip install paho-mqtt
    python logs/tb_gateway_test.py

Then look in ThingsBoard: Entities -> Devices -> NodoC-1 -> Latest telemetry.

If CONNACK fails / times out: the broker/relay is unreachable or the token is
wrong. If it connects + publishes OK but no NodoC-1 appears: the gateway device
(CON-1) does not have "Is gateway" enabled in ThingsBoard.
"""

import json
import logging
import time

import paho.mqtt.client as mqtt
import paho.mqtt.enums as mqtt_enums

# --- must match src/module_b/communications/Config.h ---
BROKER_HOST = "test-moduloa.home.kg"
BROKER_PORT = 18831
ACCESS_TOKEN = "QRPJgyk5COJPCavycmpp"  # CON-1 gateway device token
CLIENT_ID = "CON-1-pytest"
TOPIC = "v1/gateway/telemetry"
NODE_NAME = "NodoC-1"
TIMEOUT_S = 20

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("tb-test")

# One reading, flat keys - identical shape to buildGatewayPayload()'s output.
payload = {
    NODE_NAME: [
        {
            "temp": 21.5, "rh": 55.0, "pres": 1013.2, "gas": 12345.0,
            "pm1": 3.0, "pm25": 5.0, "pm10": 7.0, "co2": 588.0,
            "co": 0.0, "coTemp": 24.1, "windAngle": 0.0, "windSpeed": 0.0,
            "windValid": False, "rssi": -30, "snr": 11,
        }
    ]
}

state = {"connack": None, "published": False}


def on_connect(client, userdata, flags, reason_code, properties=None):
    state["connack"] = reason_code
    log.info("CONNACK: %s", reason_code)
    if getattr(reason_code, "is_failure", False) or reason_code != 0:
        return
    info = client.publish(TOPIC, json.dumps(payload), qos=1)
    log.info("PUBLISH sent (mid=%s): %s", info.mid, json.dumps(payload))


def on_publish(client, userdata, mid, reason_code=None, properties=None):
    state["published"] = True
    log.info("PUBACK for mid=%s", mid)


def on_disconnect(client, userdata, *args):
    log.info("disconnected %s", args)


client = mqtt.Client(mqtt_enums.CallbackAPIVersion.VERSION2, client_id=CLIENT_ID)
client.username_pw_set(ACCESS_TOKEN)  # ThingsBoard: token as username, no password
client.on_connect = on_connect
client.on_publish = on_publish
client.on_disconnect = on_disconnect
client.enable_logger(log)

log.info("connecting to %s:%s as gateway ...", BROKER_HOST, BROKER_PORT)
client.connect(BROKER_HOST, BROKER_PORT, keepalive=30)
client.loop_start()

deadline = time.time() + TIMEOUT_S
while time.time() < deadline and not state["published"]:
    time.sleep(0.2)

client.loop_stop()
client.disconnect()

if state["connack"] == 0 and state["published"]:
    print("\nOK - now check ThingsBoard: Entities -> Devices -> NodoC-1 -> Latest telemetry.")
    print("If NodoC-1 is missing, enable 'Is gateway' on CON-1 and re-run.")
elif state["connack"] is None:
    print("\nFAILED - no CONNACK within %ds. Broker/relay unreachable at this port." % TIMEOUT_S)
else:
    print("\nCONNACK=%s, published=%s - see log above." % (state["connack"], state["published"]))
