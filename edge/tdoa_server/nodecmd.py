"""Signing side of the remote node commands (src/cmd_proto.h is the other half).

    canonical = "<zone>|<node>|<ch>|<c>|<a>|<b>"          (a, b decimal, or empty when absent)
    sig       = HMAC-SHA256(key, canonical)
    command   = {"c":..,["a":..,]["b":..,]"ch":..,"sig":..}

The challenge `ch` is read from the node's retained status report
(uwb/anchor/status/<zone>/<node> -> "cmd":{"ch":"..."}): a node only accepts the challenge it last
published and burns it on use, so a captured command cannot be replayed.

tests/test_tdoa.py pins this against the same vector the firmware's host test uses
(tests/cmd_proto/test_cmd_proto.c), so the two implementations cannot drift apart silently.
"""
from __future__ import annotations

import hashlib
import hmac
import json
import re
from typing import Optional

COMMANDS = {"status", "sync_reset", "tau_set", "tau_mode", "ant_set", "reboot"}
CH_RE = re.compile(r"^[0-9a-f]{16}$")


def fingerprint(key: bytes) -> str:
    """First 4 bytes of SHA-256(key), 8 hex characters. The firmware prints the same value when
    `net cmdkey` stores a key, so a key that lost characters on the way (a pasted line overflowing
    the shell's input buffer did exactly that) shows up as a different fingerprint at once."""
    return hashlib.sha256(key).hexdigest()[:8]


def canonical(zone: str, node: str, ch: str, cmd: str, a: Optional[int] = None,
              b: Optional[int] = None) -> str:
    return "%s|%s|%s|%s|%s|%s" % (zone, node, ch, cmd, "" if a is None else a, "" if b is None else b)


def sign(key: bytes, zone: str, node: str, ch: str, cmd: str, a: Optional[int] = None,
         b: Optional[int] = None) -> str:
    return hmac.new(key, canonical(zone, node, ch, cmd, a, b).encode(), hashlib.sha256).hexdigest()


def build(key: bytes, zone: str, node: str, ch: str, cmd: str, a: Optional[int] = None,
          b: Optional[int] = None) -> str:
    """The exact payload to publish. Compact separators: the firmware's parser accepts both, but
    smaller is better on a 256-byte MQTT buffer."""
    if cmd not in COMMANDS:
        raise ValueError("unknown command %r (want one of %s)" % (cmd, sorted(COMMANDS)))
    if not CH_RE.match(ch):
        raise ValueError("challenge must be 16 lowercase hex characters, got %r" % ch)
    if len(key) < 16:
        raise ValueError("key must be at least 16 bytes")
    obj = {"c": cmd}
    if a is not None:
        obj["a"] = int(a)
    if b is not None:
        obj["b"] = int(b)
    obj["ch"] = ch
    obj["sig"] = sign(key, zone, node, ch, cmd, a, b)
    return json.dumps(obj, separators=(",", ":"))


def challenge_from_status(payload: bytes) -> Optional[str]:
    """The challenge in a node's status report, or None if commands are not enabled on it."""
    doc = json.loads(payload)
    if doc.get("online") != 1:
        return None
    ch = (doc.get("cmd") or {}).get("ch")
    return ch if isinstance(ch, str) and CH_RE.match(ch) else None
