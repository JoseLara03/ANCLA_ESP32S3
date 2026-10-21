"""MQTT glue around tdoa.Pipeline.

  local broker  --uwb/anchor/blink/<zone>-->  [collect, rebase, solve, filter]
                --uwb/anchor/setup/<zone>-->   (anchor geometry, forwarded retained)
                                                   |
  cloud broker  <--uwb/response/position/<zone>----+   {"Tid":N,"x":..,"y":..,"z":0}
  local broker  <--uwb/edge/position/<zone>--------+   same payload, for local viewers

The position payload is the frozen contract the platform already consumes
(src/pos_json.c). Nothing here publishes on the topics the GATEWAY publishes on
the local broker (uwb/response/position/<zone>), so the two never collide.

Configuration is environment-only (see .env.example); no secret lives in code.
"""
from __future__ import annotations

import json
import logging
import os
import re
import ssl
import threading
import time
from typing import Dict, Optional, Tuple

import paho.mqtt.client as mqtt

from .tdoa import Fix, Obs, Pipeline

log = logging.getLogger("tdoa_server")


def _env(name: str, default: Optional[str] = None) -> Optional[str]:
    v = os.environ.get(name)
    return default if v is None or v == "" else v


def _mk_client(client_id: str) -> mqtt.Client:
    try:                                   # paho-mqtt >= 2.0
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    except AttributeError:                 # paho-mqtt 1.x
        return mqtt.Client(client_id=client_id)


def tid_for(tag_addr: int, tid_map: Dict[int, int]) -> int:
    """Stable platform identity. The firmware derives Tid from the tag's EUI via
    the gateway's seat table; the edge only sees the short address, so a site
    maps address -> Tid explicitly (TID_MAP) and otherwise falls back to the
    short address, exactly like the gateway does for a tag with no live seat."""
    return tid_map.get(tag_addr, tag_addr)


def parse_tid_map(spec: Optional[str]) -> Dict[int, int]:
    out: Dict[int, int] = {}
    for item in (spec or "").split(","):
        item = item.strip()
        if not item:
            continue
        a, t = item.split(":")
        out[int(a, 0)] = int(t, 0) & 0x7FFFFFFF
    return out


def parse_setup(payload: bytes) -> Dict[int, Tuple[float, float, float]]:
    """uwb/anchor/setup/<zone> -> {anchor_id: (x, y, z)}.

    The gateway names anchor k "ANC-LOBBY-%03u" with the anchor's SHORT ADDRESS
    (0x0001 + anchor_id), so anchor_id = N - 1 (src/pos_json.c)."""
    doc = json.loads(payload)
    out: Dict[int, Tuple[float, float, float]] = {}
    for a in doc.get("anchors", []):
        m = re.search(r"(\d+)\s*$", str(a.get("name", "")))
        if not m:
            continue
        out[int(m.group(1)) - 1] = (float(a["x"]), float(a["y"]), float(a.get("z", 0.0)))
    return out


_EUI_RE = re.compile(r"^[0-9A-Fa-f]{16}$")


class UnknownEui(Exception):
    """An observation names its anchor by an EUI no status report has mapped to an id yet."""


def norm_eui(v) -> Optional[str]:
    """The canonical text form of an EUI (16 upper-case hex digits), or None if `v` is not one."""
    return v.upper() if isinstance(v, str) and _EUI_RE.match(v) else None


def eui_id_from_status(doc: dict) -> Optional[Tuple[str, int]]:
    """(EUI, anchor id) from one node status report, or None when it carries no EUI or no id
    (a gateway, an unassigned anchor, or firmware older than 2026-10-21)."""
    eui = norm_eui(doc.get("eui"))
    if eui is None or doc.get("role") != "an" or "id" not in doc:
        return None
    return eui, int(doc["id"])


def parse_obs(payload: bytes, eui_to_id: Optional[Dict[str, int]] = None) -> Obs:
    """One observation. Since 2026-10-21 an anchor names itself by EUI ("u"); the id ("a") is
    only a radio handle `apos assign` may move. With `eui_to_id` (built from the status topic)
    an observation carrying "u" is resolved through it and raises UnknownEui if the EUI is not
    mapped yet -- never falls back to "a", which could name another board. Without "u" (older
    firmware), or with no map given, "a" is used as before."""
    j = json.loads(payload)
    anchor = int(j["a"])
    if "u" in j:
        eui = norm_eui(j["u"])
        if eui is None:
            raise ValueError("malformed EUI %r" % (j["u"],))
        if eui_to_id is not None:
            if eui not in eui_to_id:
                raise UnknownEui(eui)
            anchor = eui_to_id[eui]
    return Obs(tag=int(j["t"]), seq=int(j["s"]), anchor=anchor,
               t_dtu=int(j["ts"]), flags=int(j.get("f", 0)), sigma=int(j.get("e", 0)))


def resolve_keys(d: Dict[str, object], eui_to_id: Dict[str, int]) -> Dict[int, object]:
    """ANCHORS_JSON / OFFSETS_DTU keyed by anchor id ("2") or by EUI ("7CDFA1B2C3D40000").
    EUI keys whose id is not known yet are left out until a status report maps them."""
    out: Dict[int, object] = {}
    for k, v in d.items():
        eui = norm_eui(k)
        if eui is not None:
            if eui in eui_to_id:
                out[eui_to_id[eui]] = v
        else:
            out[int(k)] = v      # base 10, as before: "08" must keep working
    return out


def update_eui_map(eui_to_id: Dict[str, int], doc: dict) -> bool:
    """Fold one status report into the EUI -> id map. Returns True if the map changed.

    An anchor that reports an id claims it (and an id another EUI held is released: the gateway
    re-assigned it). An anchor that reports its EUI but NO id (unassigned again, e.g. `anchor id
    auto`) drops its entry, so its observations stop resolving to a position it no longer has. A
    last will or a gateway report changes nothing."""
    if doc.get("role") != "an" or doc.get("online") == 0:
        return False
    eui = norm_eui(doc.get("eui"))
    if eui is None:
        return False
    if "id" not in doc:
        return eui_to_id.pop(eui, None) is not None
    aid = int(doc["id"])
    if eui_to_id.get(eui) == aid:
        return False
    for other in [k for k, v in eui_to_id.items() if v == aid and k != eui]:
        del eui_to_id[other]
    eui_to_id[eui] = aid
    return True


def status_events(prev: Optional[dict], cur: dict) -> list:
    """Human-readable events from two consecutive status reports of ONE node.

    Anchors cannot be reached once mounted, so this is the only place a reset, a brownout
    or a corrupted sync model is noticed. Pure so it is unit-tested."""
    if cur.get("role") == "gw":
        name = "gateway"
    elif "id" in cur:
        name = "anchor %s" % cur.get("id") + (" (%s)" % cur["eui"] if cur.get("eui") else "")
    else:
        name = "anchor %s (unassigned)" % cur.get("eui", "?")
    out = []
    if cur.get("online") == 0:
        return ["%s went OFFLINE (last will)" % name]
    if prev is None or prev.get("online") == 0:
        out.append("%s is ONLINE (reset=%s, up %ss, fw %s)"
                   % (name, cur.get("reset"), cur.get("up_s"), cur.get("fw")))
        if cur.get("reset") in ("brownout", "watchdog", "lockup"):
            out.append("%s last reset was a %s" % (name, str(cur.get("reset")).upper()))
    elif cur.get("up_s", 0) < prev.get("up_s", 0):
        out.append("%s REBOOTED between reports (reset=%s)" % (name, cur.get("reset")))
    sc, sp = cur.get("sync"), (prev or {}).get("sync")
    if sc:
        if sc.get("sigma", 0) > 1000:
            out.append("%s sync sigma %s DTU (jitter_est %s): its time conversion is not trustworthy"
                       % (name, sc.get("sigma"), sc.get("jitter")))
        if sp:
            for key, what in (("rebase", "re-baselined"), ("outliers", "rejected outlier observations")):
                d = sc.get(key, 0) - sp.get(key, 0)
                if d > 0:
                    out.append("%s %s %d time(s) since the last report" % (name, what, d))
    return out


class Service:
    def __init__(self) -> None:
        self.zone = _env("ZONE", "852541")
        self.tag_z = float(_env("TAG_Z", "-0.9"))
        self.tick_s = float(_env("TICK_MS", "20")) / 1000.0
        self.tid_map = parse_tid_map(_env("TID_MAP"))
        self.forward_setup = _env("FORWARD_SETUP", "1") == "1"
        self.publish_cloud = _env("PUBLISH_CLOUD", "1") == "1"
        self.stats_period = float(_env("STATS_PERIOD_S", "10"))
        self.t_blink = f"uwb/anchor/blink/{self.zone}"
        self.t_setup = f"uwb/anchor/setup/{self.zone}"
        self.t_pos_cloud = f"uwb/response/position/{self.zone}"
        self.t_pos_local = f"uwb/edge/position/{self.zone}"
        self.t_status = f"uwb/anchor/status/{self.zone}/#"
        self.node_status: Dict[str, dict] = {}

        # EUI -> anchor id, learned from the retained status reports. Observations that name
        # their anchor by EUI ("u") are resolved through it; see parse_obs().
        self.eui_to_id: Dict[str, int] = {}
        self.n_unknown_eui = 0
        override = _env("ANCHORS_JSON")
        self.override_raw = {k: tuple(v) for k, v in json.loads(override).items()} if override else {}
        self.have_override = bool(override)
        offsets = _env("OFFSETS_DTU")
        self.offsets_raw = {k: int(v) for k, v in json.loads(offsets).items()} if offsets else {}
        self.pipe = Pipeline(
            resolve_keys(self.override_raw, self.eui_to_id), self.tag_z,
            sigma_max=int(_env("SIGMA_MAX_DTU", "1000")),
            offsets_dtu=resolve_keys(self.offsets_raw, self.eui_to_id) or None,
            hull_margin_m=float(_env("HULL_MARGIN_M", "0.5")))
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.n_pub = 0
        self.n_pub_fail = 0
        self.last_setup: Optional[bytes] = None
        self.lat_ms = []                   # arrival-of-last-obs -> publish, per fix

        self.local = _mk_client(_env("LOCAL_CLIENT_ID", "tdoa-edge-local"))
        self.local.on_connect = self._on_local_connect
        self.local.on_message = self._on_local_message
        self.local.on_disconnect = lambda *a, **k: log.warning("local broker: disconnected")
        if _env("LOCAL_USER"):
            self.local.username_pw_set(_env("LOCAL_USER"), _env("LOCAL_PASS", ""))
        if _env("LOCAL_TLS", "0") == "1":
            self.local.tls_set(cert_reqs=ssl.CERT_NONE)
            self.local.tls_insecure_set(True)

        self.cloud: Optional[mqtt.Client] = None
        if self.publish_cloud:
            self.cloud = _mk_client(_env("CLOUD_CLIENT_ID", "tdoa-edge-cloud"))
            self.cloud.on_connect = self._on_cloud_connect
            self.cloud.on_disconnect = lambda *a, **k: log.warning("cloud broker: disconnected")
            if _env("CLOUD_USER"):
                self.cloud.username_pw_set(_env("CLOUD_USER"), _env("CLOUD_PASS", ""))
            if _env("CLOUD_TLS", "1") == "1":
                if _env("CLOUD_TLS_INSECURE", "0") == "1":
                    self.cloud.tls_set(cert_reqs=ssl.CERT_NONE)
                    self.cloud.tls_insecure_set(True)
                else:
                    self.cloud.tls_set()

    # -- local broker -------------------------------------------------------
    def _on_local_connect(self, client, userdata, flags, rc, *a):
        log.info("local broker connected rc=%s; subscribing %s, %s, %s", rc, self.t_blink,
                 self.t_setup, self.t_status)
        client.subscribe([(self.t_blink, 0), (self.t_setup, 0), (self.t_status, 0)])

    def _on_local_message(self, client, userdata, msg):
        now_ms = time.monotonic() * 1000.0
        try:
            if msg.topic == self.t_blink:
                try:
                    o = parse_obs(msg.payload, self.eui_to_id)
                except UnknownEui as e:
                    self.n_unknown_eui += 1
                    if self.n_unknown_eui == 1:
                        log.warning("observation from EUI %s, which no status report maps to an "
                                    "anchor id yet -- dropped (counted as unknown_eui)", e)
                    return
                with self.lock:
                    self.pipe.ingest(o, now_ms)
            elif msg.topic == self.t_setup:
                self._on_setup(msg.payload)
            elif msg.topic.startswith(self.t_status[:-1]):
                self._on_status(msg.topic.rsplit("/", 1)[-1], msg.payload)
        except (ValueError, KeyError, TypeError) as e:
            log.warning("bad message on %s: %s", msg.topic, e)

    def _on_status(self, node: str, payload: bytes) -> None:
        cur = json.loads(payload)
        for ev in status_events(self.node_status.get(node), cur):
            log.warning("node %s: %s", node, ev)
        self.node_status[node] = cur
        if update_eui_map(self.eui_to_id, cur):
            log.info("anchor ids by EUI: %s", self.eui_to_id)
            with self.lock:
                if self.override_raw:
                    self.pipe.set_anchors(resolve_keys(self.override_raw, self.eui_to_id))
                if self.offsets_raw:
                    self.pipe.offsets_dtu = resolve_keys(self.offsets_raw, self.eui_to_id)

    def _on_setup(self, payload: bytes) -> None:
        self.last_setup = payload
        if not self.have_override:
            anchors = parse_setup(payload)
            if anchors:
                with self.lock:
                    if anchors != self.pipe.anchors:
                        log.info("anchor geometry: %s", {k: tuple(round(c, 3) for c in v)
                                                         for k, v in sorted(anchors.items())})
                    self.pipe.set_anchors(anchors)
        self._forward_setup()

    # -- cloud broker -------------------------------------------------------
    def _on_cloud_connect(self, client, userdata, flags, rc, *a):
        log.info("cloud broker connected rc=%s", rc)
        self._forward_setup()

    def _forward_setup(self) -> None:
        if self.forward_setup and self.cloud is not None and self.last_setup \
                and self.cloud.is_connected():
            self.cloud.publish(self.t_setup, self.last_setup, qos=1, retain=True)
            log.info("forwarded retained anchor map to cloud")

    # -- main loop ----------------------------------------------------------
    @staticmethod
    def payload_for(fix: Fix, tid: int) -> str:
        return '{"Tid":%d,"x":%.2f,"y":%.2f,"z":0}' % (tid, fix.x, fix.y)

    def _publish(self, fix: Fix) -> None:
        body = self.payload_for(fix, tid_for(fix.tag, self.tid_map))
        self.local.publish(self.t_pos_local, body, qos=0)
        if self.cloud is not None:
            info = self.cloud.publish(self.t_pos_cloud, body, qos=0)
            if info.rc != mqtt.MQTT_ERR_SUCCESS:
                self.n_pub_fail += 1
        self.n_pub += 1

    def _worker(self) -> None:
        next_stats = time.monotonic() + self.stats_period
        while not self.stop.is_set():
            time.sleep(self.tick_s)
            now = time.monotonic()
            with self.lock:
                fixes = self.pipe.step(now * 1000.0)
            for fx in fixes:
                self._publish(fx)
            if now >= next_stats:
                next_stats = now + self.stats_period
                with self.lock:
                    st = dict(self.pipe.stats)
                    anchors = len(self.pipe.anchors)
                st.update(published=self.n_pub, pub_fail=self.n_pub_fail, anchors=anchors,
                          unknown_eui=self.n_unknown_eui,
                          cloud=bool(self.cloud and self.cloud.is_connected()))
                log.info("stats %s", json.dumps(st))

    def run(self) -> None:
        host = _env("LOCAL_HOST", "127.0.0.1")
        port = int(_env("LOCAL_PORT", "1883"))
        self.local.connect_async(host, port, 60)
        self.local.loop_start()
        if self.cloud is not None:
            self.cloud.connect_async(_env("CLOUD_HOST", "duck.lmq.cloudamqp.com"),
                                     int(_env("CLOUD_PORT", "8883")), 60)
            self.cloud.loop_start()
        t = threading.Thread(target=self._worker, name="solver", daemon=True)
        t.start()
        try:
            while t.is_alive():
                t.join(1.0)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop.set()
            self.local.loop_stop()
            if self.cloud is not None:
                self.cloud.loop_stop()


def main() -> None:
    logging.basicConfig(level=_env("LOG_LEVEL", "INFO"),
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    Service().run()
