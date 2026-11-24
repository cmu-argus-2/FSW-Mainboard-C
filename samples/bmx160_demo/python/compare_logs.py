#!/usr/bin/env python3
"""Compare a C bmx160_demo log with a bmx160_compare.py (CircuitPython) log.

Both print the same lines. This averages each field over the whole log and prints the
two means side by side. Gyro values are grouped by range, since the demo switches
between 2000 and 125 deg/s. Only meaningful if the board sat still, in the same place
and orientation, for both captures.

    python3 compare_logs.py c_driver.log python_driver.log
"""

import math
import re
import sys

NUM = r"(-?\d+\.\d+)"
PATTERNS = {
    "gyro": re.compile(rf"^gyro\s+{NUM}\s+{NUM}\s+{NUM} rad/s.*range (\d+)\)"),
    "accel": re.compile(rf"^accel\s+{NUM}\s+{NUM}\s+{NUM} m/s\^2"),
    "mag": re.compile(rf"^mag\s+{NUM}\s+{NUM}\s+{NUM} uT"),
    "temp": re.compile(rf"^temp\s+{NUM} degC"),
}
UNITS = {"gyro": "rad/s", "accel": "m/s^2", "mag": "uT", "temp": "degC"}


TICK = re.compile(r"^--- BMX160 \w+, tick (\d+)")


def parse(path):
    """Return {field: [samples]}, each sample a list of floats.

    Skips tick 0 (and anything before it): the first read comes right after init,
    before the chip has data, and the Python driver returns all zeros there.
    """
    fields = {}
    tick = None
    with open(path, errors="replace") as f:
        for raw in f:
            line = raw.strip()
            m = TICK.match(line)
            if m:
                tick = int(m.group(1))
                continue
            if tick is None or tick == 0:
                continue
            for name, pattern in PATTERNS.items():
                m = pattern.match(line)
                if not m:
                    continue
                values = [float(v) for v in m.groups()]
                if name == "gyro":
                    name = f"gyro@{int(values.pop())}"
                fields.setdefault(name, []).append(values)
    return fields


def stats(samples):
    n = len(samples)
    means = [sum(col) / n for col in zip(*samples)]
    stds = [math.sqrt(sum((v - mu) ** 2 for v in col) / n) for col, mu in zip(zip(*samples), means)]
    return n, means, stds


def fmt(values):
    return " ".join(f"{v:9.4f}" for v in values)


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    c_log, py_log = parse(sys.argv[1]), parse(sys.argv[2])
    names = sorted(set(c_log) | set(py_log))
    if not names:
        sys.exit("No IMU lines found in either log.")

    for name in names:
        unit = UNITS[name.split("@")[0]]
        print(f"{name} ({unit})")
        rows = []
        for label, log in (("C", c_log), ("Python", py_log)):
            if name not in log:
                print(f"  {label:6}  no samples")
                continue
            n, means, stds = stats(log[name])
            rows.append(means)
            print(f"  {label:6}  mean {fmt(means)}   std {fmt(stds)}   n={n}")
        if len(rows) == 2:
            print(f"  {'C - Py':6}  diff {fmt([c - p for c, p in zip(*rows)])}")
        print()


if __name__ == "__main__":
    main()
