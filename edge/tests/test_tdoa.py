import hashlib
import json
import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from tdoa_server import nodecmd, tdoa                          # noqa: E402
from tdoa_server.service import parse_obs, parse_setup, parse_tid_map, status_events  # noqa: E402

ANCHORS = {0: (0.0, 0.0, 0.0), 1: (3.2, 0.0, 0.0), 2: (3.34, 1.83, 0.0), 3: (-0.01, 2.42, 0.0)}
TAG_Z = -0.9                       # tag 0.9 m below the anchor plane
M_PER_DTU = tdoa.TDOA_M_PER_DTU


def dtu_for(tag_xy, anchor, t0, tag_z=TAG_Z):
    """Anchor RX timestamp (DTU) of a blink emitted at t0 DTU, from the model."""
    ax, ay, az = anchor
    r = math.sqrt((tag_xy[0] - ax) ** 2 + (tag_xy[1] - ay) ** 2 + (az - tag_z) ** 2)
    return (t0 + round(r / M_PER_DTU)) % tdoa.DTU_MODULO


class Dtu(unittest.TestCase):
    def test_rebase_across_the_40_bit_wrap(self):
        top = tdoa.DTU_MODULO - 100
        self.assertEqual(tdoa.rebase([top, 50, top - 30]), [0, 150, -30])

    def test_plausible_bound(self):
        self.assertTrue(tdoa.plausible([0, 32768, -32768]))
        self.assertFalse(tdoa.plausible([0, 32769]))

    def test_sdelta40(self):
        self.assertEqual(tdoa.sdelta40(5, tdoa.DTU_MODULO - 5), 10)
        self.assertEqual(tdoa.sdelta40(tdoa.DTU_MODULO - 5, 5), -10)


class Solve(unittest.TestCase):
    def test_recovers_position_exactly(self):
        for xy in [(2.0, 2.4), (0.5, 0.5), (3.0, 1.0), (1.6, 1.2)]:
            t0 = tdoa.DTU_MODULO - 1234          # forces the wrap
            ts = [dtu_for(xy, ANCHORS[a], t0) for a in sorted(ANCHORS)]
            rb = tdoa.rebase(ts)
            meas = [tdoa.Meas(*ANCHORS[a][:2], ANCHORS[a][2] - TAG_Z, t)
                    for a, t in zip(sorted(ANCHORS), rb)]
            x, y, resid = tdoa.solve(meas)
            self.assertAlmostEqual(x, xy[0], delta=0.02)
            self.assertAlmostEqual(y, xy[1], delta=0.02)
            self.assertLess(resid, 0.02)

    def test_three_anchors_is_exact(self):
        xy = (2.0, 2.4)
        ids = [0, 1, 3]
        ts = tdoa.rebase([dtu_for(xy, ANCHORS[a], 1000) for a in ids])
        meas = [tdoa.Meas(*ANCHORS[a][:2], ANCHORS[a][2] - TAG_Z, t) for a, t in zip(ids, ts)]
        x, y, _ = tdoa.solve(meas)
        self.assertAlmostEqual(x, xy[0], delta=0.03)
        self.assertAlmostEqual(y, xy[1], delta=0.03)

    def test_refuses_fewer_than_three(self):
        self.assertIsNone(tdoa.solve([tdoa.Meas(0, 0, 1, 0), tdoa.Meas(1, 0, 1, 5)]))

    def test_collinear_anchors_refused(self):
        meas = [tdoa.Meas(0, 0, 1, 0), tdoa.Meas(1, 0, 1, 10), tdoa.Meas(2, 0, 1, 20)]
        self.assertIsNone(tdoa.solve(meas, seed=(1.0, 0.0)))


class Abg(unittest.TestCase):
    def test_gain_relation(self):
        c = tdoa.AbgCfg.from_index(0.02)
        self.assertAlmostEqual(c.gamma, c.beta ** 2 / (2 * c.alpha), places=12)
        self.assertGreater(c.alpha, 0)
        self.assertLess(c.alpha, 1)

    def test_converges_and_gates(self):
        c = tdoa.AbgCfg.from_index(0.02)
        f = tdoa.Abg()
        f.seed(0.0, 0.0)
        for _ in range(200):
            self.assertEqual(f.step(c, 0.2, 1.0, 1.0, True), tdoa.ABG_ACCEPTED)
        self.assertAlmostEqual(f.x, 1.0, delta=1e-3)
        # a 5 m jump is gated, and reseeds after reset_after consecutive rejects
        r = [f.step(c, 0.2, 6.0, 6.0, True) for _ in range(5)]
        self.assertEqual(r[:4], [tdoa.ABG_REJECTED] * 4)
        self.assertEqual(r[4], tdoa.ABG_RESEEDED)
        self.assertAlmostEqual(f.x, 6.0)

    def test_bad_input(self):
        c = tdoa.AbgCfg.from_index(0.02)
        f = tdoa.Abg()
        self.assertEqual(f.step(c, 0.2, 1, 1, True), tdoa.ABG_BAD_INPUT)   # not seeded
        f.seed(0, 0)
        self.assertEqual(f.step(c, 0.0, 1, 1, True), tdoa.ABG_BAD_INPUT)   # dt <= 0
        self.assertEqual(f.step(c, 0.2, math.nan, 1, True), tdoa.ABG_BAD_INPUT)


class TagId(unittest.TestCase):
    def test_fnv1a_vector_and_bit31(self):
        self.assertEqual(tdoa.tag_id_from_eui(b"a"), 0xE40C292C & 0x7FFFFFFF)
        for i in range(50):
            self.assertLess(tdoa.tag_id_from_eui(bytes([i] * 8)), 1 << 31)


class Collector(unittest.TestCase):
    def obs(self, a, seq=1, tag=256):
        return tdoa.Obs(tag, seq, a, 1000 * a)

    def test_early_release_when_complete(self):
        c = tdoa.Collector(4)
        for a in range(4):
            c.add(self.obs(a), 0.0)
        out = c.take_ready(1.0)
        self.assertEqual(len(out), 1)
        self.assertEqual([o.anchor for o in out[0][1]], [0, 1, 2, 3])

    def test_window_release_with_three_and_discard_with_two(self):
        c = tdoa.Collector(4)
        for a in (0, 1, 3):
            c.add(self.obs(a, seq=1), 0.0)
        for a in (0, 1):
            c.add(self.obs(a, seq=2), 0.0)
        self.assertEqual(c.take_ready(149.0), [])
        out = c.take_ready(151.0)
        self.assertEqual([len(g) for _t, g in out], [3])       # seq 2 discarded
        self.assertEqual(c.take_ready(10_000.0), [])

    def test_duplicate_anchor_rejected_and_oldest_first(self):
        c = tdoa.Collector(3)
        self.assertTrue(c.add(self.obs(0, seq=1), 0.0))
        self.assertFalse(c.add(self.obs(0, seq=1), 1.0))
        for a in (1, 2):
            c.add(self.obs(a, seq=1), 2.0)
        for a in (0, 1, 2):
            c.add(self.obs(a, seq=2), 50.0)
        out = c.take_ready(60.0)
        self.assertEqual(len(out), 2)


class Pipeline(unittest.TestCase):
    def run_static(self, n_blinks=200, noise_dtu=15.0, anchors=(0, 1, 2, 3), seed=1):
        rnd = random.Random(seed)
        xy = (2.0, 2.4)
        p = tdoa.Pipeline({a: ANCHORS[a] for a in anchors}, TAG_Z)
        fixes = []
        t0 = 123_456_789
        for k in range(n_blinks):
            now = k * 200.0
            emit = t0 + int(k * 200e-3 / tdoa.S_PER_DTU)
            for a in anchors:
                ts = dtu_for(xy, ANCHORS[a], emit) + round(rnd.gauss(0, noise_dtu))
                p.ingest(tdoa.Obs(256, k % 256, a, ts % tdoa.DTU_MODULO, 0), now + rnd.uniform(0, 30))
            fixes += p.step(now + 60.0)
        return p, fixes, xy

    def test_static_tag_four_anchors(self):
        p, fixes, xy = self.run_static()
        self.assertGreater(len(fixes), 190)
        last = fixes[-50:]
        mx = sum(f.x for f in last) / len(last)
        my = sum(f.y for f in last) / len(last)
        self.assertAlmostEqual(mx, xy[0], delta=0.15)
        self.assertAlmostEqual(my, xy[1], delta=0.15)
        self.assertTrue(all(f.n_anchors == 4 for f in fixes))
        self.assertEqual(p.stats["reorder"], 0)

    def test_three_anchor_residual_is_zero_by_construction(self):
        _p, fixes, _ = self.run_static(anchors=(0, 1, 3))
        self.assertTrue(fixes)
        self.assertTrue(all(f.residual == 0.0 and f.n_anchors == 3 for f in fixes))

    def test_out_of_order_group_is_discarded(self):
        p = tdoa.Pipeline(ANCHORS, TAG_Z)
        xy = (2.0, 2.4)
        t_a, t_b = 5_000_000_000, 5_000_000_000 + int(0.2 / tdoa.S_PER_DTU)

        def feed(seq, t_emit, now):
            for a in ANCHORS:
                p.ingest(tdoa.Obs(256, seq, a, dtu_for(xy, ANCHORS[a], t_emit)), now)
            return p.step(now + 1)

        self.assertEqual(len(feed(2, t_b, 0.0)), 1)
        self.assertEqual(len(feed(1, t_a, 300.0)), 0)       # older blink arrives later
        self.assertEqual(p.stats["reorder"], 1)

    def test_unknown_anchor_counted(self):
        p = tdoa.Pipeline({0: ANCHORS[0]}, TAG_Z)
        p.ingest(tdoa.Obs(1, 1, 3, 0), 0.0)
        self.assertEqual(p.stats["no_anchor"], 1)


class Protections(unittest.TestCase):
    """Sigma exclusion and static offsets: what protects the solve when anchors cannot be reached."""

    XY = (2.0, 2.4)

    def feed(self, p, k, bias_dtu=None, sigma=None, anchors=(0, 1, 2, 3)):
        bias_dtu = bias_dtu or {}
        sigma = sigma or {}
        emit = 9_000_000_000 + k * 12_780_000_000
        for a in anchors:
            ts = dtu_for(self.XY, ANCHORS[a], emit) + bias_dtu.get(a, 0)
            p.ingest(tdoa.Obs(256, k % 256, a, ts % tdoa.DTU_MODULO, 0, sigma.get(a, 20)), k * 200.0)
        return p.step(k * 200.0 + 1.0)

    def drive(self, p, n=40, **kw):
        out = []
        for k in range(n):
            out += self.feed(p, k, **kw)
        return out

    def test_corrupt_reference_hurts_without_sigma_exclusion(self):
        p = tdoa.Pipeline(ANCHORS, TAG_Z)                     # no protection
        fixes = self.drive(p, bias_dtu={0: 400}, sigma={0: 29926})
        # a corrupt reference either breaks every solve or yields a wrong position
        err = math.hypot(fixes[-1].x - self.XY[0], fixes[-1].y - self.XY[1]) if fixes else math.inf
        self.assertGreater(err, 0.5)

    def test_sigma_exclusion_ignores_the_unstable_anchor(self):
        p = tdoa.Pipeline(ANCHORS, TAG_Z, sigma_max=1000)
        fixes = self.drive(p, bias_dtu={0: 400}, sigma={0: 29926})
        self.assertTrue(fixes)
        self.assertAlmostEqual(fixes[-1].x, self.XY[0], delta=0.05)
        self.assertAlmostEqual(fixes[-1].y, self.XY[1], delta=0.05)
        self.assertEqual(fixes[-1].n_anchors, 3)
        self.assertGreater(p.stats["sigma_excluded"], 0)

    def test_reference_is_the_anchor_with_smallest_sigma(self):
        p = tdoa.Pipeline(ANCHORS, TAG_Z, sigma_max=1000)
        grp = [tdoa.Obs(256, 1, a, 1000 * a, 0, s) for a, s in ((0, 40), (1, 30), (2, 20), (3, 50))]
        self.assertEqual([o.anchor for o in p._prepare(grp)], [2, 1, 0, 3])

    def test_too_few_stable_anchors_falls_back_to_all(self):
        p = tdoa.Pipeline(ANCHORS, TAG_Z, sigma_max=1000)
        grp = [tdoa.Obs(256, 1, a, 1000 * a, 0, s) for a, s in ((0, 29926), (1, 20), (2, 29926), (3, 29926))]
        self.assertEqual(len(p._prepare(grp)), 4)
        self.assertEqual(p.stats["sigma_fallback"], 1)

    def test_unknown_sigma_is_never_excluded(self):
        p = tdoa.Pipeline(ANCHORS, TAG_Z, sigma_max=1000)
        grp = [tdoa.Obs(256, 1, a, 1000 * a, 0, 0) for a in (0, 1, 2, 3)]
        self.assertEqual(len(p._prepare(grp)), 4)

    def test_static_offset_is_removed(self):
        biased = tdoa.Pipeline(ANCHORS, TAG_Z)
        fixed = tdoa.Pipeline(ANCHORS, TAG_Z, offsets_dtu={2: 250, 3: -120})
        bias = {2: 250, 3: -120}
        bad = self.drive(biased, bias_dtu=bias)[-1]
        good = self.drive(fixed, bias_dtu=bias)[-1]
        self.assertGreater(math.hypot(bad.x - self.XY[0], bad.y - self.XY[1]), 0.3)
        self.assertAlmostEqual(good.x, self.XY[0], delta=0.05)
        self.assertAlmostEqual(good.y, self.XY[1], delta=0.05)

    def test_offset_wraps_the_40_bit_counter(self):
        p = tdoa.Pipeline(ANCHORS, TAG_Z, offsets_dtu={1: 100})
        out = p._prepare([tdoa.Obs(1, 1, 1, 50, 0, 20)])
        self.assertEqual(out[0].t_dtu, tdoa.DTU_MODULO - 50)


class Parsing(unittest.TestCase):
    def test_parse_obs(self):
        o = parse_obs(b'{"a":2,"t":256,"s":46,"ts":"379107328198","q":223,"b":36,"f":1,"e":42}')
        self.assertEqual((o.anchor, o.tag, o.seq, o.t_dtu, o.flags), (2, 256, 46, 379107328198, 1))

    def test_parse_setup_maps_name_to_anchor_id(self):
        doc = {"name": "852541", "anchors": [
            {"name": "ANC-LOBBY-001", "x": 0.0, "y": 0.0, "z": 0.0},
            {"name": "ANC-LOBBY-003", "x": 3.34, "y": 1.83, "z": 0.0},
            {"name": "ANC-LOBBY-002", "x": 3.2, "y": 0.0, "z": 0.0}]}
        got = parse_setup(json.dumps(doc).encode())
        self.assertEqual(got[0], (0.0, 0.0, 0.0))
        self.assertEqual(got[1], (3.2, 0.0, 0.0))
        self.assertEqual(got[2], (3.34, 1.83, 0.0))

    def test_tid_map(self):
        self.assertEqual(parse_tid_map("256:693116308, 0x101:5"), {256: 693116308, 0x101: 5})
        self.assertEqual(parse_tid_map(None), {})


class StatusEvents(unittest.TestCase):
    BASE = {"online": 1, "role": "an", "id": 1, "up_s": 100, "reset": "power", "fw": "x",
            "sync": {"sigma": 20, "jitter": 18, "rebase": 5, "outliers": 0}}

    def test_first_report_announces_the_node(self):
        ev = status_events(None, self.BASE)
        self.assertEqual(len(ev), 1)
        self.assertIn("ONLINE", ev[0])

    def test_brownout_reset_is_called_out(self):
        cur = dict(self.BASE, reset="brownout")
        self.assertTrue(any("BROWNOUT" in e for e in status_events(None, cur)))

    def test_last_will_means_offline(self):
        self.assertEqual(status_events(self.BASE, {"online": 0, "role": "an", "id": 1}),
                         ["anchor 1 went OFFLINE (last will)"])

    def test_reboot_between_reports(self):
        cur = dict(self.BASE, up_s=3, reset="software")
        self.assertTrue(any("REBOOTED" in e for e in status_events(self.BASE, cur)))

    def test_saturated_sigma_and_new_rebaselines(self):
        cur = dict(self.BASE, up_s=110, sync={"sigma": 29941, "jitter": 84561, "rebase": 7, "outliers": 2})
        ev = " | ".join(status_events(self.BASE, cur))
        self.assertIn("not trustworthy", ev)
        self.assertIn("re-baselined 2 time(s)", ev)
        self.assertIn("rejected outlier observations 2 time(s)", ev)

    def test_quiet_when_nothing_changed(self):
        self.assertEqual(status_events(self.BASE, dict(self.BASE, up_s=110)), [])


class NodeCmd(unittest.TestCase):
    """Pinned to the vectors in tests/cmd_proto/test_cmd_proto.c, which verifies them in C."""

    KEY = bytes(range(32))

    def test_matches_the_firmware_vectors(self):
        vec = [("tau_set", 312, None, "3dbf20d755345f70100590e7e59556893055348e6f01c93004102403100d34a9"),
               ("ant_set", 16453, 16385, "a1462f949d1b0d9d4dda1231503367d98a036f7bd6481ef3d2a91e04fb33933f"),
               ("reboot", None, None, "a385df6ea492394f078ba6e9eebec7185976f2409536391b6e5bb5785e24ab09"),
               ("tau_set", -45, None, "2be60c67357e747a90f83b675a851f5d59db70f48bcc73321d54c675de229ffc")]
        for cmd, a, b, want in vec:
            self.assertEqual(nodecmd.sign(self.KEY, "852541", "1", "a1b2c3d4e5f60718", cmd, a, b), want)

    def test_fingerprint_matches_the_firmware_vector(self):
        # tests/hmac_sha256/test_hmac_sha256.c checks the same value in C for key 00..1f
        self.assertEqual(nodecmd.fingerprint(self.KEY), hashlib.sha256(bytes(range(32))).hexdigest()[:8])
        self.assertEqual(len(nodecmd.fingerprint(self.KEY)), 8)
        # one lost byte changes it, which is the whole point
        self.assertNotEqual(nodecmd.fingerprint(self.KEY), nodecmd.fingerprint(self.KEY[:-1]))

    def test_canonical_form(self):
        self.assertEqual(nodecmd.canonical("852541", "1", "a1b2c3d4e5f60718", "tau_set", 312),
                         "852541|1|a1b2c3d4e5f60718|tau_set|312|")
        self.assertEqual(nodecmd.canonical("852541", "gw", "a1b2c3d4e5f60718", "reboot"),
                         "852541|gw|a1b2c3d4e5f60718|reboot||")

    def test_payload_shape(self):
        p = json.loads(nodecmd.build(self.KEY, "852541", "1", "a1b2c3d4e5f60718", "ant_set", 16453, 16385))
        self.assertEqual((p["c"], p["a"], p["b"], p["ch"]), ("ant_set", 16453, 16385, "a1b2c3d4e5f60718"))
        self.assertEqual(len(p["sig"]), 64)
        self.assertLess(len(json.dumps(p, separators=(",", ":"))), 256)

    def test_rejects_bad_input(self):
        with self.assertRaises(ValueError):
            nodecmd.build(self.KEY, "852541", "1", "a1b2c3d4e5f60718", "format_disk")
        with self.assertRaises(ValueError):
            nodecmd.build(self.KEY, "852541", "1", "NOTHEX", "reboot")
        with self.assertRaises(ValueError):
            nodecmd.build(b"short", "852541", "1", "a1b2c3d4e5f60718", "reboot")

    def test_challenge_from_status(self):
        ok = b'{"online":1,"role":"an","id":1,"cmd":{"ch":"a1b2c3d4e5f60718"}}'
        self.assertEqual(nodecmd.challenge_from_status(ok), "a1b2c3d4e5f60718")
        self.assertIsNone(nodecmd.challenge_from_status(b'{"online":1,"role":"an","id":1}'))   # no key
        self.assertIsNone(nodecmd.challenge_from_status(b'{"online":0,"role":"an","id":1}'))   # last will
        self.assertIsNone(nodecmd.challenge_from_status(b'{"online":1,"cmd":{"ch":"zz"}}'))


if __name__ == "__main__":
    unittest.main()
