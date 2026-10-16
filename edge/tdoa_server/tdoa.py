"""TDoA pipeline for the edge server: a port of the gateway's host-tested C modules.

Ported 1:1 (same constants, same gates, same order of operations) from
  src/tdoa_dtu.c      40-bit rebase + plausibility bound
  src/tdoa_collect.c  grouping of the anchors' observations of ONE blink
  src/tdoa_solve.c    hyperbolic 2D Gauss-Newton over range differences
  src/pos_abg.c       alpha-beta-gamma filter on the solved position
  src/tdoa_gw.c       solve_one()/resolve_one(): dt classification, seed memo,
                      mirror-branch jump gate, publish rules
  src/tag_id.c        FNV-1a 32-bit Tid, bit 31 cleared

Differences that are deliberate: float64 instead of float32 (the C uses float
because the ESP32-S3 has a single-precision FPU; nothing here depends on it),
and the collector is a dict instead of 16 fixed slots (no eviction policy is
needed off the ESP32 -- a hard cap guards memory instead).

Pure Python, no third-party imports, so the same file runs in the container and
in the unit tests.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# ---- constants (each one cites the C header it comes from) -----------------
TDOA_M_PER_DTU = 4.69175e-3          # tdoa_solve.h
TDOA_MIN_ANCHORS = 3                  # tdoa_solve.h
DTU_MODULO = 1 << 40                  # tdoa_dtu.h
DTU_HALF = DTU_MODULO // 2
DTU_MAX_SPREAD = 32768                # tdoa_dtu.h: 153.7 m of path difference
COLLECT_WINDOW_MS = 150               # tdoa_collect.h
S_PER_DTU = 1.56498e-11               # tdoa_gw.c TDOA_GW_S_PER_DTU
DT_MAX_S = 1.0                        # tdoa_gw.h TDOA_DT_MAX_MS
DT_REORDER_MAX_S = 1.0                # tdoa_gw.h TDOA_DT_REORDER_MAX_MS
SEED_AGE_MS = 1000                    # tdoa_gw.h TDOA_GW_SEED_AGE_MS
MAX_JUMP_M = 10.0                     # tdoa_gw.h TDOA_GW_MAX_JUMP_M
BLINK_FLAG_MOVING = 0x02              # blink_frame.h

# tdoa_solve.c
_MAX_ITER = 20
_CONV_EPS_M = 1e-4
_DET_EPS = 1e-6
_MAX_HALVINGS = 8
_GRAD_EPS = 5e-2
_R_MIN_M = 1e-3

# pos_abg.h
ABG_T_NOM_S = 0.2
ABG_DT_MIN_S = 0.05
ABG_GATE_DT_CAP = 3.0


# ---- tdoa_dtu ---------------------------------------------------------------
def sdelta40(a: int, b: int) -> int:
    """Signed difference of two 40-bit counters (|result| < 2**39)."""
    d = (a - b) & (DTU_MODULO - 1)
    return d - DTU_MODULO if d & (1 << 39) else d


def rebase(t_dtu: List[int]) -> List[int]:
    """Express every timestamp relative to t_dtu[0], folded into [-2^39, 2^39)."""
    base = t_dtu[0]
    out = []
    for t in t_dtu:
        d = (t - base) % DTU_MODULO       # Python % is already non-negative
        if d >= DTU_HALF:
            d -= DTU_MODULO
        out.append(d)
    return out


def plausible(rebased: List[int]) -> bool:
    return bool(rebased) and all(-DTU_MAX_SPREAD <= t <= DTU_MAX_SPREAD for t in rebased)


# ---- tdoa_solve -------------------------------------------------------------
@dataclass
class Meas:
    x: float
    y: float
    dz: float
    t_dtu: int          # already rebased: meas[0].t_dtu == 0


def _slant(a: Meas, x: float, y: float) -> float:
    dx, dy = x - a.x, y - a.y
    return math.sqrt(dx * dx + dy * dy + a.dz * a.dz)


def _diff_residual(m: List[Meas], x: float, y: float) -> float:
    r0 = _slant(m[0], x, y)
    acc = 0.0
    for i in range(1, len(m)):
        d_meas = (m[i].t_dtu - m[0].t_dtu) * TDOA_M_PER_DTU
        f = (_slant(m[i], x, y) - r0) - d_meas
        acc += f * f
    return math.sqrt(acc / (len(m) - 1))


def _grad_norm2(m: List[Meas], x: float, y: float) -> float:
    r0 = _slant(m[0], x, y)
    if r0 < _R_MIN_M:
        return -1.0
    g0 = g1 = 0.0
    for i in range(1, len(m)):
        ri = _slant(m[i], x, y)
        if ri < _R_MIN_M:
            return -1.0
        d_meas = (m[i].t_dtu - m[0].t_dtu) * TDOA_M_PER_DTU
        f = (ri - r0) - d_meas
        gx = (x - m[i].x) / ri - (x - m[0].x) / r0
        gy = (y - m[i].y) / ri - (y - m[0].y) / r0
        g0 += gx * f
        g1 += gy * f
    return g0 * g0 + g1 * g1


def solve(m: List[Meas], seed: Optional[Tuple[float, float]] = None
          ) -> Optional[Tuple[float, float, float]]:
    """Gauss-Newton over range DIFFERENCES. Returns (x, y, residual) or None.

    Same convergence test, backtracking line search and final stationarity gate
    as tdoa_solve(): a stalled search returns None rather than the seed.
    """
    n = len(m)
    if n < TDOA_MIN_ANCHORS:
        return None
    if seed is not None:
        x, y = seed
    else:
        x = sum(a.x for a in m) / n
        y = sum(a.y for a in m) / n

    for _ in range(_MAX_ITER):
        jtj00 = jtj01 = jtj11 = jtf0 = jtf1 = 0.0
        r0 = _slant(m[0], x, y)
        if r0 < _R_MIN_M:
            return None
        for i in range(1, n):
            ri = _slant(m[i], x, y)
            if ri < _R_MIN_M:
                return None
            d_meas = (m[i].t_dtu - m[0].t_dtu) * TDOA_M_PER_DTU
            f = (ri - r0) - d_meas
            gx = (x - m[i].x) / ri - (x - m[0].x) / r0
            gy = (y - m[i].y) / ri - (y - m[0].y) / r0
            jtj00 += gx * gx
            jtj01 += gx * gy
            jtj11 += gy * gy
            jtf0 += gx * f
            jtf1 += gy * f
        det = jtj00 * jtj11 - jtj01 * jtj01
        if abs(det) < _DET_EPS:
            return None
        dx = -(jtj11 * jtf0 - jtj01 * jtf1) / det
        dy = -(-jtj01 * jtf0 + jtj00 * jtf1) / det
        if abs(dx) < _CONV_EPS_M and abs(dy) < _CONV_EPS_M:
            break
        cost0 = _diff_residual(m, x, y)
        step = 1.0
        improved = False
        nx = ny = 0.0
        for _ls in range(_MAX_HALVINGS):
            nx, ny = x + step * dx, y + step * dy
            cost1 = _diff_residual(m, nx, ny)
            if math.isfinite(cost1) and cost1 < cost0:
                improved = True
                break
            step *= 0.5
        if not improved:
            break
        x, y = nx, ny

    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    g2 = _grad_norm2(m, x, y)
    if g2 < 0.0 or g2 > _GRAD_EPS * _GRAD_EPS:
        return None
    return x, y, _diff_residual(m, x, y)


# ---- pos_abg ----------------------------------------------------------------
def _cbrt(v: float) -> float:
    return math.copysign(abs(v) ** (1.0 / 3.0), v)


@dataclass
class AbgCfg:
    alpha: float
    beta: float
    gamma: float
    alpha_still: float = 0.15
    gate_m: float = 1.5
    reset_after: int = 5

    @classmethod
    def from_index(cls, lam: float = 0.02, **kw) -> "AbgCfg":
        """Gray-Murray gains from one tracking index (pos_abg_cfg_from_index)."""
        b = lam / 2.0 - 3.0
        c = lam / 2.0 + 3.0
        d = -1.0
        p = c - b * b / 3.0
        q = 2.0 * b ** 3 / 27.0 - b * c / 3.0 + d
        v = math.sqrt(q * q + 4.0 * p ** 3 / 27.0)
        s = -_cbrt((q + v) / 2.0) - _cbrt((q - v) / 2.0) - b / 3.0
        alpha = 1.0 - s * s
        beta = 2.0 * (1.0 - s) ** 2
        gamma = beta * beta / (2.0 * alpha)
        return cls(alpha=alpha, beta=beta, gamma=gamma, **kw)


ABG_ACCEPTED = "accepted"
ABG_REJECTED = "rejected"
ABG_RESEEDED = "reseeded"
ABG_BAD_INPUT = "bad_input"


@dataclass
class Abg:
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    reject_streak: int = 0
    init: bool = False

    def seed(self, x: float, y: float) -> None:
        self.x, self.y = x, y
        self.vx = self.vy = self.ax = self.ay = 0.0
        self.reject_streak = 0
        self.init = True

    def step(self, c: AbgCfg, dt: float, zx: float, zy: float, still: bool) -> str:
        if (not self.init or not (dt > 0.0) or not math.isfinite(dt)
                or not math.isfinite(zx) or not math.isfinite(zy)):
            return ABG_BAD_INPUT
        T, T2 = dt, dt * dt
        xp = self.x + self.vx * T + 0.5 * self.ax * T2
        yp = self.y + self.vy * T + 0.5 * self.ay * T2
        vxp = self.vx + self.ax * T
        vyp = self.vy + self.ay * T
        rx, ry = zx - xp, zy - yp
        scale = min(max(T / ABG_T_NOM_S, 1.0), ABG_GATE_DT_CAP)
        if math.hypot(rx, ry) > c.gate_m * scale:
            self.x, self.y, self.vx, self.vy = xp, yp, vxp, vyp
            self.reject_streak = min(self.reject_streak + 1, 255)
            if c.reset_after and self.reject_streak >= c.reset_after:
                self.seed(zx, zy)
                return ABG_RESEEDED
            return ABG_REJECTED
        a = c.alpha_still if still else c.alpha
        self.x, self.y = xp + a * rx, yp + a * ry
        if still:
            self.vx = self.vy = self.ax = self.ay = 0.0
        else:
            self.vx = vxp + (c.beta / T) * rx
            self.vy = vyp + (c.beta / T) * ry
            self.ax = self.ax + (c.gamma / T2) * rx
            self.ay = self.ay + (c.gamma / T2) * ry
        self.reject_streak = 0
        return ABG_ACCEPTED


# ---- Tid ----------------------------------------------------------------------
def tag_id_from_eui(eui: bytes) -> int:
    h = 2166136261
    for byte in eui:
        h ^= byte
        h = (h * 16777619) & 0xFFFFFFFF
    return h & 0x7FFFFFFF


# ---- collector ----------------------------------------------------------------
@dataclass
class Obs:
    tag: int
    seq: int
    anchor: int
    t_dtu: int
    flags: int = 0
    sigma: int = 0      # the anchor's own 1-sigma conversion error, DTU (wire field "e"); 0 = unknown


@dataclass
class _Group:
    first_ms: float
    obs: Dict[int, Obs] = field(default_factory=dict)


class Collector:
    """Groups the observations of one blink, keyed on (tag, seq)."""

    MAX_GROUPS = 256

    def __init__(self, expected: int = 4):
        self.expected = max(TDOA_MIN_ANCHORS, expected)
        self._g: Dict[Tuple[int, int], _Group] = {}
        self.dup = 0
        self.shed = 0

    def set_expected(self, n: int) -> None:
        self.expected = max(TDOA_MIN_ANCHORS, n)

    def add(self, o: Obs, now_ms: float) -> bool:
        key = (o.tag, o.seq)
        g = self._g.get(key)
        if g is None:
            if len(self._g) >= self.MAX_GROUPS:
                self.shed += 1
                return False
            g = self._g[key] = _Group(first_ms=now_ms)
        if o.anchor in g.obs:
            self.dup += 1
            return False
        g.obs[o.anchor] = o
        return True

    def take_ready(self, now_ms: float) -> List[Tuple[int, List[Obs]]]:
        """Every releasable group, OLDEST FIRST (the caller's dt depends on it)."""
        ready = []
        for key, g in list(self._g.items()):
            age = now_ms - g.first_ms
            n = len(g.obs)
            if n < self.expected:
                if age < COLLECT_WINDOW_MS:
                    continue
                if n < TDOA_MIN_ANCHORS:
                    del self._g[key]          # below minimum: discard
                    continue
            ready.append((age, key, g))
        ready.sort(key=lambda r: -r[0])
        out = []
        for _age, key, g in ready:
            del self._g[key]
            out.append((key[0], [g.obs[a] for a in sorted(g.obs)]))
        return out


# ---- per-tag memo + pipeline (tdoa_gw.c solve_one) ----------------------------
@dataclass
class _Memo:
    has_ref_t: bool = False
    last_ref_t: int = 0
    has_pos: bool = False
    x: float = 0.0
    y: float = 0.0
    pos_ms: float = 0.0
    moving: bool = False
    abg: Abg = field(default_factory=Abg)


@dataclass
class Fix:
    tag: int
    x: float
    y: float
    n_anchors: int
    residual: float


class Pipeline:
    def __init__(self, anchors: Dict[int, Tuple[float, float, float]], tag_z: float,
                 abg: Optional[AbgCfg] = None, sigma_max: int = 0,
                 offsets_dtu: Optional[Dict[int, int]] = None):
        """anchors: id -> (x, y, z) in the survey frame; tag_z: the tag plane's z in
        the same frame (`apos tagz`). The model's dz is anchor z - tag_z, and only
        its square enters, so the sign convention does not matter.

        Two protections that exist because anchors cannot be reached in the field:

        sigma_max     An anchor publishes its own sync error with every observation. One
                      whose figure exceeds this (0 disables) is left out of the solve while
                      at least TDOA_MIN_ANCHORS others remain, and the reference anchor is
                      the one with the SMALLEST sigma. A reference with a corrupt time
                      conversion otherwise shifts every difference at once.
        offsets_dtu   Static per-anchor timestamp error, DTU, SUBTRACTED from that anchor's
                      timestamps before solving: positive = the anchor's converted time is
                      late. Replaces re-flashing `sync tau set` on each anchor; measured
                      with tools/calibrate.py from one tag at a known position."""
        self.anchors = dict(anchors)
        self.sigma_max = sigma_max
        self.offsets_dtu = dict(offsets_dtu or {})
        self.tag_z = tag_z
        self.abg = abg or AbgCfg.from_index(0.02)
        self.collect = Collector(len(self.anchors))
        self.memo: Dict[int, _Memo] = {}
        self.stats = dict(obs_in=0, dup=0, no_anchor=0, implausible=0, solve_fail=0,
                          jump=0, reorder=0, seeded=0, dt_reseed=0, filtered=0,
                          gate_rejected=0, reseed=0, still=0, fixes=0,
                          sigma_excluded=0, sigma_fallback=0)

    def set_anchors(self, anchors: Dict[int, Tuple[float, float, float]]) -> None:
        self.anchors = dict(anchors)
        self.collect.set_expected(len(self.anchors))

    def ingest(self, o: Obs, now_ms: float) -> None:
        if o.anchor not in self.anchors:
            self.stats["no_anchor"] += 1
            return
        mm = self.memo.setdefault(o.tag, _Memo())
        mm.moving = bool(o.flags & BLINK_FLAG_MOVING)
        if self.collect.add(o, now_ms):
            self.stats["obs_in"] += 1
        else:
            self.stats["dup"] += 1

    def step(self, now_ms: float) -> List[Fix]:
        fixes: List[Fix] = []
        for tag, group in self.collect.take_ready(now_ms):
            fx = self._solve_one(tag, group, now_ms)
            if fx is not None:
                fixes.append(fx)
        return fixes

    def _prepare(self, group: List[Obs]) -> List[Obs]:
        """Apply the static offsets, drop unstable anchors, choose the reference."""
        if self.offsets_dtu:
            group = [Obs(o.tag, o.seq, o.anchor,
                         (o.t_dtu - self.offsets_dtu.get(o.anchor, 0)) % DTU_MODULO,
                         o.flags, o.sigma) for o in group]
        if self.sigma_max:
            usable = [o for o in group if o.sigma == 0 or o.sigma <= self.sigma_max]
            if len(usable) < len(group):
                if len(usable) >= TDOA_MIN_ANCHORS:
                    self.stats["sigma_excluded"] += len(group) - len(usable)
                    group = usable
                else:
                    self.stats["sigma_fallback"] += 1     # too few left: solve with all
            group = sorted(group, key=lambda o: (o.sigma if o.sigma > 0 else 1 << 30, o.anchor))
        return group

    def _solve_one(self, tag: int, group: List[Obs], now_ms: float) -> Optional[Fix]:
        st = self.stats
        group = self._prepare(group)
        fix_t = group[0].t_dtu
        rb = rebase([o.t_dtu for o in group])
        if not plausible(rb):
            st["implausible"] += 1
            return None
        mm = self.memo.setdefault(tag, _Memo())

        dt_ok, dt_s = False, 0.0
        if mm.has_ref_t:
            raw = sdelta40(fix_t, mm.last_ref_t)
            dt_s = raw * S_PER_DTU
            if (raw <= 0 and -dt_s <= DT_REORDER_MAX_S) or (raw > 0 and dt_s < ABG_DT_MIN_S):
                st["reorder"] += 1                # reference held on purpose
                return None
            dt_ok = raw > 0 and dt_s <= DT_MAX_S
        mm.last_ref_t, mm.has_ref_t = fix_t, True

        meas = []
        for o, t in zip(group, rb):
            x, y, z = self.anchors[o.anchor]
            meas.append(Meas(x, y, z - self.tag_z, t))
        res = self._resolve(mm, meas, now_ms)
        if res is None:
            return None
        rx, ry, resid = res

        if not mm.abg.init:
            mm.abg.seed(rx, ry)
            st["seeded"] += 1
        elif not dt_ok:
            mm.abg.seed(rx, ry)
            st["dt_reseed"] += 1
        else:
            still = not mm.moving
            r = mm.abg.step(self.abg, dt_s, rx, ry, still)
            if r == ABG_ACCEPTED:
                st["filtered"] += 1
                st["still"] += 1 if still else 0
            elif r == ABG_RESEEDED:
                st["reseed"] += 1
            elif r == ABG_REJECTED:
                st["gate_rejected"] += 1
                return None
            else:
                st["solve_fail"] += 1
                return None

        mm.x, mm.y, mm.pos_ms, mm.has_pos = mm.abg.x, mm.abg.y, now_ms, True
        st["fixes"] += 1
        n = len(group)
        return Fix(tag, mm.abg.x, mm.abg.y, n, 0.0 if n <= TDOA_MIN_ANCHORS else resid)

    def _resolve(self, mm: _Memo, meas: List[Meas], now_ms: float):
        seed = None
        if mm.has_pos and 0 <= now_ms - mm.pos_ms < SEED_AGE_MS:
            seed = (mm.x, mm.y)
        res = solve(meas, seed)
        if res is None:
            self.stats["solve_fail"] += 1
            return None
        if seed is not None and math.hypot(res[0] - seed[0], res[1] - seed[1]) > MAX_JUMP_M:
            self.stats["jump"] += 1
            return None
        return res
