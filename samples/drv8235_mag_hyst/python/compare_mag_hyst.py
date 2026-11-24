#!/usr/bin/env python3
"""Compare a Python (mag_hyst.py) and a C (samples/drv8235_mag_hyst) sweep log.

    python3 compare_mag_hyst.py python_hyst.log c_hyst.log

Rows are matched by (coil, step). For each swept coil:

1. Voltage of the swept coil, in VMTR counts (0.16733 V each in both drivers), at every
   step. Both drivers command int(|step| * 35.86), so the readings should agree.
2. Current of the swept coil, in IMTR counts. The drivers scale current differently
   (Python 0.01451 A/count, C 3.7/192 A/count), so amps are converted back to counts.
3. The other five coils should read about 0 V (brake) in both.
4. Field: a straight-line fit of each axis against throttle. The slope (uT at full
   throttle) is what the coil adds, so the two slopes should match. The offset is the
   background field and may drift between runs, so it isn't compared. The residual is
   noise plus any non-linearity.

A coil that neither log swept (panel not connected) is skipped. A coil swept in only
one log is a difference.

Every step is a single sample, so the voltage and current checks use the bias: the
mean of C minus Python over the sweep. A command that is one code off moves every
reading the same way, while noise averages out. The largest single difference is
shown too.
"""

import argparse
import math
import sys

COILS = ["XP", "XM", "YP", "YM", "ZP", "ZM"]
VOLTS_PER_COUNT = 0.16733
AMPS_PER_COUNT = {"python": 0.01451, "c": 3.7 / 192}
CODES_PER_THROTTLE = 6.0 / 42.67 * 255  # both drivers: int(|throttle| * this), max 35

V_TOL_COUNTS = 0.5  # |bias| over the sweep
I_TOL_COUNTS = 1.0
I_TOL_FRAC = 0.10
OFF_TOL_COUNTS = 1.0  # coils at brake
SLOPE_TOL_UT = 1.0
SLOPE_TOL_FRAC = 0.10


def parse(path):
    rows, avg = {}, None
    for line in open(path, errors="replace"):
        line = line.strip()
        if line.startswith("Average mag measurement time"):
            avg = line.split(":")[-1].strip()
        f = line.split(",")
        if len(f) != 18 or f[0] not in COILS:
            continue
        try:
            step = round(float(f[1]), 2) + 0.0  # + 0.0: -0.0 -> 0.0
            vals = [float(x) for x in f[2:]]
        except ValueError:
            continue  # header or a line cut off by the capture
        rows[(f[0], step)] = {
            "t_ns": vals[0],
            "mag": vals[1:4],
            "v": dict(zip(COILS, vals[4:10])),
            "i": dict(zip(COILS, vals[10:16])),
        }
    return rows, avg


def finite(*xs):
    return all(not math.isnan(x) for x in xs)


def mean(xs):
    xs = list(xs)
    return sum(xs) / len(xs) if xs else float("nan")


def fit(points):
    """Least-squares b0 + b1 * x; returns (b1, b0, residual rms)."""
    pts = [(x, y) for x, y in points if finite(y)]
    if len(pts) < 3:
        return float("nan"), float("nan"), float("nan")
    mx, my = mean(p[0] for p in pts), mean(p[1] for p in pts)
    sxx = sum((x - mx) ** 2 for x, _ in pts)
    b1 = sum((x - mx) * (y - my) for x, y in pts) / sxx
    b0 = my - b1 * mx
    rms = math.sqrt(mean((y - b0 - b1 * x) ** 2 for x, y in pts))
    return b1, b0, rms


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("python_log")
    ap.add_argument("c_log")
    args = ap.parse_args()

    logs = {}
    for who, path in (("python", args.python_log), ("c", args.c_log)):
        rows, avg = parse(path)
        if not rows:
            sys.exit("no sweep rows in %s" % path)
        logs[who] = rows
        nan_mag = sum(1 for r in rows.values() if not finite(*r["mag"]))
        print("%-6s %3d rows (of %d), mag read failures %d, average mag read %s ns"
              % (who, len(rows), len(COILS) * 61, nan_mag, avg or "?"))
    py, c = logs["python"], logs["c"]

    ok = True
    compared = 0
    print("\ncoil | V counts: bias/max diff | I counts: bias/max diff, at full | others max V | "
          "slope Python (uT)          slope C (uT)              |diff|  residual py/c")
    for coil in COILS:
        in_py = {s for (k, s) in py if k == coil}
        in_c = {s for (k, s) in c if k == coil}
        steps = sorted(in_py & in_c)
        if not in_py and not in_c:
            print("%-4s | not swept in either log (not connected?), skipped" % coil)
            continue
        if not steps:
            print("%-4s | swept only in the %s log" % (coil, "Python" if in_py else "C"))
            ok = False
            continue
        compared += 1
        dv, di, off = [], [], []
        i_full = {"python": [], "c": []}
        for s in steps:
            rp, rc = py[(coil, s)], c[(coil, s)]
            vp, vc = rp["v"][coil], rc["v"][coil]
            ip, ic = rp["i"][coil], rc["i"][coil]
            if finite(vp, vc):
                dv.append((abs(vc) - abs(vp)) / VOLTS_PER_COUNT)
            if finite(ip, ic):
                cp, cc = abs(ip) / AMPS_PER_COUNT["python"], abs(ic) / AMPS_PER_COUNT["c"]
                di.append(cc - cp)
                if abs(s) == 1.0:
                    i_full["python"].append(cp)
                    i_full["c"].append(cc)
            for other in COILS:
                if other == coil:
                    continue
                for r in (rp, rc):
                    if finite(r["v"][other]):
                        off.append(abs(r["v"][other]) / VOLTS_PER_COUNT)

        slopes, resid = {}, {}
        for who, rows in (("python", py), ("c", c)):
            axes = [fit([(s, rows[(coil, s)]["mag"][a]) for s in steps]) for a in range(3)]
            slopes[who] = [ax[0] for ax in axes]
            resid[who] = norm([ax[2] for ax in axes])
        sd = norm([a - b for a, b in zip(slopes["python"], slopes["c"])])

        full = max(mean(i_full["python"]), mean(i_full["c"]))
        v_ok = abs(mean(dv)) <= V_TOL_COUNTS
        i_ok = abs(mean(di)) <= max(I_TOL_COUNTS, I_TOL_FRAC * full)
        off_ok = (max(off) if off else 0.0) <= OFF_TOL_COUNTS
        b_ok = sd <= max(SLOPE_TOL_UT, SLOPE_TOL_FRAC * norm(slopes["python"]))
        ok &= v_ok and i_ok and off_ok and b_ok
        print(
            "%-4s | %+5.2f / %4.1f %-6s | %+5.2f / %4.1f, %4.1f/%4.1f %-6s | %4.1f %-6s | "
            "(%6.2f %6.2f %6.2f)  (%6.2f %6.2f %6.2f)  %5.2f %-6s %4.2f/%4.2f"
            % (
                coil,
                mean(dv), max(abs(x) for x in dv) if dv else float("nan"), "ok" if v_ok else "DIFFER",
                mean(di), max(abs(x) for x in di) if di else float("nan"), mean(i_full["python"]), mean(i_full["c"]),
                "ok" if i_ok else "DIFFER",
                max(off) if off else float("nan"), "ok" if off_ok else "DIFFER",
                *slopes["python"], *slopes["c"], sd, "ok" if b_ok else "DIFFER",
                resid["python"], resid["c"],
            )
        )
        if norm(slopes["python"]) < 3 * resid["python"]:
            print("     %s: the coil's field is close to the noise; is the panel in its flight position?" % coil)

    print("\nV and I in register counts; bias is the mean of C minus Python. Slope = field change from throttle 0 to 1, per axis.")
    print("Tolerances: |bias| V %.1f count, I max(%.1f count, %d%% of full), other coils %.0f count, "
          "slope max(%.1f uT, %d%%)."
          % (V_TOL_COUNTS, I_TOL_COUNTS, I_TOL_FRAC * 100, OFF_TOL_COUNTS, SLOPE_TOL_UT, SLOPE_TOL_FRAC * 100))
    if compared == 0:
        ok = False
    print("\nCompared %d of %d coils." % (compared, len(COILS)))
    print("\nRESULT: %s" % ("same behavior within tolerance" if ok else "differences found, see above"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
