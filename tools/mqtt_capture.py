"""Capture position + blink MQTT traffic from the ANCLA gateway.

Logs every message on uwb/# to a CSV with the local receive timestamp
(milliseconds) so duplicates and inter-arrival gaps can be measured.
"""

import csv
import json
import sys
import time

import paho.mqtt.client as mqtt

HOST = "duck.lmq.cloudamqp.com"
PORT = 1883
USER = "isgekeah:isgekeah"
PASS = "nm2h7-Oi4PXYU54sbcr3yPWmfij7DSFu"

DURATION_S = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
OUT = sys.argv[2] if len(sys.argv) > 2 else "mqtt_capture.csv"


def main() -> None:
    rows = []

    def on_connect(client, userdata, flags, rc, properties=None):
        print("connected rc=", rc)
        client.subscribe("uwb/#")

    def on_message(client, userdata, msg):
        rows.append((time.time() * 1000.0, msg.topic,
                     msg.payload.decode("utf-8", "replace")))

    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    c.username_pw_set(USER, PASS)
    c.on_connect = on_connect
    c.on_message = on_message
    c.connect(HOST, PORT, 60)
    c.loop_start()
    time.sleep(DURATION_S)
    c.loop_stop()

    with open(OUT, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["local_ms", "topic", "payload"])
        w.writerows(rows)
    print(f"captured {len(rows)} messages -> {OUT}")

    # Quick duplicate report on the position topic.
    pos = [(t, p) for (t, top, p) in rows if "position" in top]
    prev = {}
    dup = 0
    for t, p in pos:
        try:
            d = json.loads(p)
        except ValueError:
            continue
        tid = d.get("Tid")
        key = (tid, d.get("x"), d.get("y"))
        if key in prev and t - prev[key][0] < 250:
            dup += 1
            print(f"DUP tag {tid}: same (x,y) after "
                  f"{t - prev[key][0]:.0f} ms at {t:.0f}")
        prev[key] = (t, p)
    print(f"position msgs: {len(pos)}, near-instant duplicates: {dup}")


if __name__ == "__main__":
    main()
