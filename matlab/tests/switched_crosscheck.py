"""Independent cross-check of plant_switched_step.m (switched boost plant).

Usage:  python3 -I switched_crosscheck.py <csv_dir>

<csv_dir> must contain sw_cases.csv and sw_case_<name>.csv written by
tests/sw_crosscheck_export.m (Octave/MATLAB).  For every case this script
re-simulates the circuit from the same initial state and the same per-period
duty / load / Vin sequence, without using any MATLAB result except those
inputs, and compares the per-period outputs.

The model here is written from the circuit (KVL around the inductor loop,
KCL at the output node) and integrated with classical RK4 at a step of at
most 1e-8 s.  The switching instant d*Ts is a step boundary.  Diode events
(iL reaching 0, re-conduction when Vout < Vin - VF) are located by bisection
on the RK4 step fraction.  Averages are trapezoidal on the fine grid and
min/max are taken over all fine points (both sides of each switching
instant), so they are the "true" values that the MATLAB sampling at nsub
points per period has to approach.
Exit code 0 = all comparisons within tolerance.
"""
import csv
import math
import os
import sys
import time

DT_MAX = 1e-8


def read_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


class Circuit:
    """Boost converter with Vin, L(rL), low-side switch (Ron), diode (VF) or
    synchronous high-side switch (Ron), C with ESR rC, load R."""

    def __init__(self, L, C, rL, rC, Ron, VF, sync):
        self.L, self.C, self.rL, self.rC = L, C, rL, rC
        self.Ron, self.VF, self.sync = Ron, VF, sync

    def out_node(self, i_in, vc, R):
        """Output node: current i_in enters, splits into the load R and the
        capacitor branch (rC in series with C at voltage vc).
        Returns (terminal voltage, capacitor current)."""
        if self.rC == 0.0:
            return vc, i_in - (vc / R if R != math.inf else 0.0)
        gl = 0.0 if R == math.inf else 1.0 / R
        v = (i_in + vc / self.rC) / (1.0 / self.rC + gl)
        return v, (v - vc) / self.rC

    def deriv(self, mode, il, vc, R, vin):
        """mode 0 = switch ON, 1 = OFF conducting, 2 = OFF idle (DCM)."""
        if mode == 0:
            _, ic = self.out_node(0.0, vc, R)
            v_l = vin - (self.rL + self.Ron) * il
        elif mode == 1:
            vo, ic = self.out_node(il, vc, R)
            if self.sync:
                v_sw = vo + self.Ron * il
            else:
                v_sw = vo + self.VF
            v_l = vin - self.rL * il - v_sw
        else:
            _, ic = self.out_node(0.0, vc, R)
            v_l = 0.0
        return v_l / self.L, ic / self.C

    def vout(self, mode, il, vc, R):
        return self.out_node(il if mode == 1 else 0.0, vc, R)[0]

    def rk4(self, mode, il, vc, R, vin, h):
        k1i, k1v = self.deriv(mode, il, vc, R, vin)
        k2i, k2v = self.deriv(mode, il + 0.5 * h * k1i, vc + 0.5 * h * k1v, R, vin)
        k3i, k3v = self.deriv(mode, il + 0.5 * h * k2i, vc + 0.5 * h * k2v, R, vin)
        k4i, k4v = self.deriv(mode, il + h * k3i, vc + h * k3v, R, vin)
        il2 = il + h / 6.0 * (k1i + 2 * k2i + 2 * k3i + k4i)
        vc2 = vc + h / 6.0 * (k1v + 2 * k2v + 2 * k3v + k4v)
        if mode == 2:
            il2 = 0.0
        return il2, vc2


class Acc:
    def __init__(self):
        self.t = None
        self.si = self.sv = 0.0
        self.imin = self.vmin = math.inf
        self.imax = self.vmax = -math.inf
        self.pi = self.pv = 0.0
        self.t_idle = 0.0
        self.dcm = False

    def add(self, t, il, vo):
        if self.t is not None:
            dt = t - self.t
            self.si += 0.5 * dt * (self.pi + il)
            self.sv += 0.5 * dt * (self.pv + vo)
        self.t, self.pi, self.pv = t, il, vo
        self.imin = min(self.imin, il)
        self.imax = max(self.imax, il)
        self.vmin = min(self.vmin, vo)
        self.vmax = max(self.vmax, vo)


def bisect_fraction(fun, lo=0.0, hi=1.0, iters=60):
    """fun(lo) >= 0 > fun(hi); returns the step fraction of the sign change."""
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        if fun(mid) >= 0.0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def run_interval(ck, mode, il, vc, R, vin, t0, T, acc):
    """Integrate an ON (mode 0) or OFF (mode 1/2) interval of length T."""
    if T <= 0.0:
        return mode, il, vc
    n = max(1, int(math.ceil(T / DT_MAX - 1e-9)))
    h = T / n
    vth = vin - ck.VF
    acc.add(t0, il, ck.vout(mode, il, vc, R))
    j = 0
    tl = 0.0  # time already advanced inside the current fine step
    while j < n:
        hs = h - tl
        il2, vc2 = ck.rk4(mode, il, vc, R, vin, hs)
        if mode == 1 and not ck.sync and il2 < 0.0:
            th = bisect_fraction(lambda f: ck.rk4(1, il, vc, R, vin, f * hs)[0])
            il, vc = ck.rk4(1, il, vc, R, vin, th * hs)
            il = 0.0
            tl += th * hs
            acc.add(t0 + j * h + tl, il, ck.vout(1, il, vc, R))
            mode = 2
            acc.dcm = True
            continue
        if mode == 2:
            vo2 = ck.vout(2, 0.0, vc2, R)
            if vth > 0.0 and vo2 < vth:
                th = bisect_fraction(
                    lambda f: ck.vout(2, 0.0, ck.rk4(2, 0.0, vc, R, vin, f * hs)[1], R) - vth)
                _, vc = ck.rk4(2, 0.0, vc, R, vin, th * hs)
                tl += th * hs
                acc.t_idle += th * hs
                acc.add(t0 + j * h + tl, 0.0, ck.vout(2, 0.0, vc, R))
                mode = 1
                continue
            acc.t_idle += hs
        il, vc = il2, vc2
        j += 1
        tl = 0.0
        acc.add(t0 + j * h if j < n else t0 + T, il, ck.vout(mode, il, vc, R))
    return mode, il, vc


def simulate(par, seq):
    ck = Circuit(par["L"], par["C"], par["rL"], par["rC"], par["Ron"], par["VF"],
                 par["sync"] != 0)
    Ts = par["Ts"]
    il, vc = par["iL0"], par["vC0"]
    res = []
    for d, R, vin in seq:
        d = min(max(d, 0.0), 1.0)
        if not ck.sync and il < 0.0:
            il = 0.0
        t_on = d * Ts
        acc = Acc()
        mode = 0
        if t_on > 0.0:
            mode, il, vc = run_interval(ck, 0, il, vc, R, vin, 0.0, t_on, acc)
        if Ts - t_on > 0.0:
            vo_idle = ck.vout(2, 0.0, vc, R)
            if (not ck.sync) and il <= 0.0 and (vin - ck.VF) - vo_idle < 0.0:
                mode = 2
                acc.dcm = True
            else:
                mode = 1
            mode, il, vc = run_interval(ck, mode, il, vc, R, vin, t_on, Ts - t_on, acc)
        res.append({
            "iL1": il, "vC1": vc, "Vout_end": acc.pv,
            "iL_avg": acc.si / Ts, "Vout_avg": acc.sv / Ts,
            "iL_min": acc.imin, "iL_max": acc.imax,
            "Vout_min": acc.vmin, "Vout_max": acc.vmax,
            "dcm": 1.0 if acc.dcm else 0.0, "t_idle": acc.t_idle,
        })
    return res


# absolute tolerances; min/max/averages are looser for coarse nsub
TOL_STATE = {"iL1": 1e-7, "vC1": 1e-7, "Vout_end": 1e-7, "t_idle": 1e-10, "dcm": 0.0}
TOL_SAMPLED = {"iL_avg": 1e-5, "Vout_avg": 1e-5, "iL_min": 1e-5, "iL_max": 1e-5,
               "Vout_min": 1e-4, "Vout_max": 1e-4}


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    d = sys.argv[1]
    cases = read_rows(os.path.join(d, "sw_cases.csv"))
    ok_all = True
    print("%-14s %-9s %12s %10s %s" % ("case", "field", "max|diff|", "tol", "result"))
    for c in cases:
        par = {k: (c[k] if k == "name" else float(c[k])) for k in c}
        rows = read_rows(os.path.join(d, "sw_case_%s.csv" % c["name"]))
        seq = [(float(r["d"]), float(r["R"]), float(r["Vin"])) for r in rows]
        t0 = time.time()
        py = simulate(par, seq)
        el = time.time() - t0
        coarse = par["nsub"] < 50
        fields = list(TOL_STATE) + list(TOL_SAMPLED)
        for f in fields:
            diff = max(abs(float(r[f]) - p[f]) for r, p in zip(rows, py))
            if f in TOL_STATE:
                tol = TOL_STATE[f]
            elif coarse:
                tol = None  # sampled quantities at nsub < 50 are reported only
            else:
                tol = TOL_SAMPLED[f]
            if tol is None:
                res = "info (nsub=%d)" % par["nsub"]
            else:
                ok = diff <= tol
                ok_all = ok_all and ok
                res = "PASS" if ok else "FAIL"
            print("%-14s %-9s %12.3e %10s %s" % (c["name"], f, diff,
                                                 "-" if tol is None else "%.0e" % tol, res))
        ndcm = sum(p["dcm"] for p in py)
        print("%-14s periods=%d dcm_periods=%d python_time=%.1fs" % (c["name"], len(py), ndcm, el))
    print("OVERALL:", "PASS" if ok_all else "FAIL")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
