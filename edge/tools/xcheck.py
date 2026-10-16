"""Compare edge/tdoa_server/tdoa.py against the real firmware C (tdoa_solve, pos_abg).

Needs gcc on PATH (the host build the repo's own tests use). Not run in Docker.

    python -I edge/tools/xcheck.py

Generates random geometries and timestamps -- including degenerate and noisy ones
-- feeds them to both implementations and requires agreement to the precision
float32 allows. A case where one side returns a fix and the other refuses is a
failure, because that is exactly the behavioural divergence this guards against.
"""
import math
import os
import random
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(HERE, ".."))

from tdoa_server import tdoa  # noqa: E402

SRC = os.path.join(ROOT, "src")
EXE = os.path.join(HERE, "xcheck.exe" if os.name == "nt" else "xcheck")


def build():
    cmd = ["gcc", "-O1", "-Wall", "-Wextra", "-D__USE_MINGW_ANSI_STDIO=1", f"-I{SRC}", "-o", EXE,
           os.path.join(HERE, "xcheck.c"), os.path.join(SRC, "tdoa_solve.c"),
           os.path.join(SRC, "pos_residual.c"), os.path.join(SRC, "pos_abg.c"), "-lm"]
    subprocess.run(cmd, check=True)


def make_cases(rnd, n_cases):
    cases = []
    for _ in range(n_cases):
        n = rnd.choice([3, 3, 4, 4, 4])
        anchors = [(rnd.uniform(-2, 8), rnd.uniform(-2, 8), rnd.uniform(0.0, 2.5)) for _ in range(n)]
        if rnd.random() < 0.15:
            anchors = [(float(i), 0.0, 1.0) for i in range(n)]       # collinear
        tag = (rnd.uniform(-1, 9), rnd.uniform(-1, 9))
        noise = rnd.choice([0, 0, 5, 40, 400])
        ts = []
        for (x, y, dz) in anchors:
            r = math.sqrt((tag[0] - x) ** 2 + (tag[1] - y) ** 2 + dz * dz)
            ts.append(round(r / tdoa.TDOA_M_PER_DTU) + round(rnd.gauss(0, noise)))
        base = ts[0]
        ts = [t - base for t in ts]
        seed = (rnd.uniform(-1, 9), rnd.uniform(-1, 9)) if rnd.random() < 0.5 else None
        cases.append((anchors, ts, seed))
    return cases


def main():
    build()
    rnd = random.Random(12345)
    cases = make_cases(rnd, 4000)
    lams = [0.005, 0.02, 0.05, 0.1, 0.3]
    lines = []
    for anchors, ts, seed in cases:
        lines.append("S %d %d %.9g %.9g " % (len(anchors), 1 if seed else 0,
                                             seed[0] if seed else 0.0, seed[1] if seed else 0.0)
                     + " ".join("%.9g %.9g %.9g %d" % (a[0], a[1], a[2], t) for a, t in zip(anchors, ts)))
    lines += ["G %.9g" % l for l in lams]
    out = subprocess.run([EXE], input="\n".join(lines) + "\n", capture_output=True, text=True,
                         check=True).stdout.splitlines()

    bad = 0
    both_ok = 0
    for (anchors, ts, seed), line in zip(cases, out):
        meas = [tdoa.Meas(a[0], a[1], a[2], t) for a, t in zip(anchors, ts)]
        py = tdoa.solve(meas, seed)
        c = line.split()
        c_ok = c[1] == "ok"
        if (py is not None) != c_ok:
            bad += 1
            if bad <= 5:
                print("DIVERGENCE (fix vs refusal):", "py", py, "C", line)
            continue
        if c_ok:
            both_ok += 1
            cx, cy, cr = map(float, c[2:5])
            # float32 vs float64: where the cost surface is flat (tag far outside the
            # anchors) the converged point legitimately differs by a few cm per 10 m,
            # while the residual -- the quantity that matters -- agrees to ~1e-6.
            tol = 2e-2 + 5e-3 * math.hypot(cx, cy)
            if abs(py[0] - cx) > tol or abs(py[1] - cy) > tol or abs(py[2] - cr) > 2e-2:
                bad += 1
                if bad <= 5:
                    print("MISMATCH:", py, line)
    for lam, line in zip(lams, out[len(cases):]):
        _g, a, b, g = line.split()
        cfg = tdoa.AbgCfg.from_index(lam)
        for name, pv, cv in (("alpha", cfg.alpha, float(a)), ("beta", cfg.beta, float(b)),
                             ("gamma", cfg.gamma, float(g))):
            if abs(pv - cv) > 1e-5 * max(1.0, abs(cv)):
                bad += 1
                print("GAIN MISMATCH lambda=%g %s py=%.9g C=%.9g" % (lam, name, pv, cv))
    print(f"solve cases: {len(cases)}, both returned a fix: {both_ok}, divergences: {bad}")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
