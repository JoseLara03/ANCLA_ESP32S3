"""Send a signed command to a node over MQTT and print its answer.

    python -I edge/tools/nodecmd.py genkey                       # make a per-site secret
    python -I edge/tools/nodecmd.py 7CDFA1B2C3D40000 status      # an anchor, by EUI
    python -I edge/tools/nodecmd.py 7CDFA1B2C3D40000 tau_set 312
    python -I edge/tools/nodecmd.py 7CDFA1B2C3D40000 tau_mode 1  # 0 legacy, 1 survey
    python -I edge/tools/nodecmd.py 7CDFA1B2C3D40000 ant_set 16453 16385   # next boot
    python -I edge/tools/nodecmd.py 7CDFA1B2C3D40000 sync_reset
    python -I edge/tools/nodecmd.py gw reboot

Since 2026-10-21 an anchor is addressed by its EUI (16 hex digits: the MAC on its label followed by
0000, as `apos addr` or `anchor show` prints it), not by its anchor id -- two boards can share an id,
never an EUI. Its topics and the signature both use that name.

The key comes from NODE_CMD_KEY (hex) or --key-file. The broker is the one in edge/.env
(LOCAL_HOST/PORT/USER/PASS, run from the edge directory) or --host/--port/--user/--password.

Each node must have been given the same key ON THE BENCH, once:   net cmdkey <hex>
Until then it answers "disabled" to everything (it fails closed).

Flow: read the node's retained status report -> take the challenge in it -> sign
"<zone>|<node>|<challenge>|<cmd>|<a>|<b>" -> publish to uwb/anchor/cmd/<zone>/<node> -> wait for
the ack on uwb/anchor/ack/<zone>/<node>. The challenge is single-use and rotates every 10 s, so
the whole exchange has to finish within about a status period; this tool does it in well under a
second on a LAN.
"""
import argparse
import os
import re
import secrets
import ssl
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from tdoa_server import nodecmd  # noqa: E402


def load_env(path=".env"):
    out = {}
    if os.path.exists(path):
        for line in open(path, encoding="utf-8"):
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                out[k] = v
    return out


def mk_client(cid):
    import paho.mqtt.client as mqtt
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=cid)
    except AttributeError:
        return mqtt.Client(client_id=cid)


def main():
    env = load_env()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("node", help="anchor EUI (16 hex digits), 'gw', or 'genkey'")
    ap.add_argument("cmd", nargs="?")
    ap.add_argument("args", nargs="*", type=int)
    ap.add_argument("--zone", default=env.get("ZONE", "852541"))
    ap.add_argument("--host", default=env.get("LOCAL_HOST", "127.0.0.1"))
    ap.add_argument("--port", type=int, default=int(env.get("LOCAL_PORT", "1883")))
    ap.add_argument("--user", default=env.get("LOCAL_USER"))
    ap.add_argument("--password", default=env.get("LOCAL_PASS"))
    ap.add_argument("--tls", action="store_true", default=env.get("LOCAL_TLS", "0") == "1")
    ap.add_argument("--key-file")
    ap.add_argument("--timeout", type=float, default=6.0)
    a = ap.parse_args()

    if a.node == "genkey":
        k = secrets.token_hex(32)
        print(k)
        print("fingerprint " + nodecmd.fingerprint(bytes.fromhex(k)) +
              "   <- every node prints this when it stores the key; it must match")
        print("\nOn the bench, once per node (anchor and gateway), over its console:\n  net cmdkey " + k)
        print("On the machine that sends commands:\n  set NODE_CMD_KEY=" + k)
        return 0
    if a.cmd is None:
        ap.error("give a command")

    key_hex = os.environ.get("NODE_CMD_KEY") or (open(a.key_file).read().strip() if a.key_file else None)
    if not key_hex:
        sys.exit("no key: set NODE_CMD_KEY (hex) or pass --key-file")
    key = bytes.fromhex(key_hex)
    print("using key with fingerprint %s" % nodecmd.fingerprint(key), file=sys.stderr)

    node = a.node if a.node == "gw" else a.node.upper()
    if node != "gw" and not re.match(r"^[0-9A-F]{16}$", node):
        ap.error("node must be an anchor EUI (16 hex digits, as `apos addr` prints it) or 'gw' -- "
                 "anchors are no longer addressed by id")
    status_topic = "uwb/anchor/status/%s/%s" % (a.zone, node)
    cmd_topic = "uwb/anchor/cmd/%s/%s" % (a.zone, node)
    ack_topic = "uwb/anchor/ack/%s/%s" % (a.zone, node)
    got_status, got_ack = threading.Event(), threading.Event()
    box = {}

    def on_connect(c, u, f, rc, *x):
        c.subscribe([(status_topic, 0), (ack_topic, 0)])

    def on_message(c, u, m):
        if m.topic == status_topic:
            ch = nodecmd.challenge_from_status(m.payload)
            box["status"] = m.payload.decode()
            box["ch"] = ch
            got_status.set()
        elif m.topic == ack_topic and "sent" in box:
            box["ack"] = m.payload.decode()
            got_ack.set()

    c = mk_client("nodecmd-%d" % os.getpid())
    if a.user:
        c.username_pw_set(a.user, a.password or "")
    if a.tls:
        c.tls_set(cert_reqs=ssl.CERT_NONE)
        c.tls_insecure_set(True)
    c.on_connect, c.on_message = on_connect, on_message
    c.connect(a.host, a.port, 30)
    c.loop_start()
    try:
        if not got_status.wait(a.timeout):
            sys.exit("no status report from node %s within %.0f s -- is it online and on the new firmware?"
                     % (node, a.timeout))
        if box.get("ch") is None:
            sys.exit("node %s publishes no challenge: remote commands are not enabled on it "
                     "(no key stored -- run `net cmdkey` on its console once). Status: %s"
                     % (node, box.get("status")))
        payload = nodecmd.build(key, a.zone, node, box["ch"], a.cmd,
                                a.args[0] if len(a.args) > 0 else None,
                                a.args[1] if len(a.args) > 1 else None)
        box["sent"] = True
        c.publish(cmd_topic, payload, qos=0)
        if not got_ack.wait(a.timeout):
            sys.exit("no ack from node %s (command sent: it may have been dropped, or the node is "
                     "not subscribed)" % node)
        print(box["ack"])
        return 0 if '"ok":1' in box["ack"] else 2
    finally:
        c.loop_stop()


if __name__ == "__main__":
    sys.exit(main())
