#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Generate ADCS parity vectors from the Python flight software.

Runs the unmodified Python apps/adcs modules (with numpy standing in for ulab)
and writes a C header used by tests/tasks/adcs.

Usage (needs numpy, e.g. the Zephyr venv):
    python3 test_vectors/adcs/gen_adcs_vectors.py <FSW-mainboard>/flight \
        > tests/tasks/adcs/src/adcs_vectors.h
"""

import sys
import types

import numpy as np


def load_python_adcs(flight_dir):
    sys.path.insert(0, flight_dir)
    ulab = types.ModuleType("ulab")
    ulab.numpy = np
    sys.modules["ulab"] = ulab
    mp = types.ModuleType("micropython")
    mp.const = lambda x: x
    sys.modules["micropython"] = mp
    core = types.ModuleType("core")
    core.__path__ = []
    core.logger = types.SimpleNamespace(warning=lambda *_: None)
    sys.modules["core"] = core
    sc = types.ModuleType("core.satellite_config")

    class adcs_config:
        CONTROLLER_MODE = 0
        MAG_BIAS_UT = [0.0, 0.0, 0.0]
        MAG_SCALE = [1.0, 1.0, 1.0]

    sc.adcs_config = adcs_config
    sys.modules["core.satellite_config"] = sc
    hal = types.ModuleType("hal")
    hal.__path__ = []
    sys.modules["hal"] = hal
    hc = types.ModuleType("hal.configuration")
    hc.SATELLITE = FakeSatellite()
    sys.modules["hal.configuration"] = hc

    import apps.adcs.acs as acs
    import apps.adcs.consts as consts
    import apps.adcs.sensors as sensors
    import apps.adcs.sun as sun

    return acs, consts, sensors, sun, hc.SATELLITE


class FakeSatellite:
    FACES = ("XP", "XM", "YP", "YM", "ZP", "ZM")

    def __init__(self):
        self.available = [True] * 6
        self.applied = [0.0] * 6

    def TORQUE_DRIVERS_AVAILABLE(self, face):
        return self.available[self.FACES.index(face)]

    def APPLY_MAGNETIC_CONTROL(self, face, value):
        self.applied[self.FACES.index(face)] = float(value)


rng = np.random.default_rng(18873)


def f32(values):
    """Round inputs to float32 so C and Python see identical inputs."""
    return [float(v) for v in np.asarray(values, dtype=np.float32).ravel()]


def unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def cfloat(v):
    text = f"{v:.9g}"
    if not any(ch in text for ch in ".e"):
        text += ".0"
    return text + "f"


def carr(values):
    return "{" + ", ".join(cfloat(v) for v in values) + "}"


def emit_controllers(acs, out):
    out.append("struct ctrl_vector { float a[3]; float b[3]; float c[3]; float u[3]; };")
    spin, sun, bcross = [], [], []
    for _ in range(12):
        omega = f32(rng.normal(0, 0.3, 3))
        mag = f32(rng.normal(0, 3e-5, 3))
        spin.append((omega, mag, f32(acs.spin_stabilizing_controller(np.array(omega), np.array(mag)))))
        bcross.append((mag, omega, f32(acs.bcross_controller(np.array(mag), np.array(omega)))))
        s = f32(unit(rng.normal(0, 1, 3)))
        sun.append((s, omega, mag, f32(acs.sun_pointing_controller(np.array(s), np.array(omega), np.array(mag)))))
    out.append("/* a=omega, b=mag */")
    out.append("static const struct ctrl_vector spin_vectors[] = {")
    out += [f"\t{{{carr(o)}, {carr(m)}, {{0}}, {carr(u)}}}," for o, m, u in spin]
    out.append("};")
    out.append("/* a=mag, b=omega */")
    out.append("static const struct ctrl_vector bcross_vectors[] = {")
    out += [f"\t{{{carr(m)}, {carr(o)}, {{0}}, {carr(u)}}}," for m, o, u in bcross]
    out.append("};")
    out.append("/* a=sun, b=omega, c=mag */")
    out.append("static const struct ctrl_vector sun_pointing_vectors[] = {")
    out += [f"\t{{{carr(s)}, {carr(o)}, {carr(m)}, {carr(u)}}}," for s, o, m, u in sun]
    out.append("};")

    out.append("struct bdot_vector { float buffer[6][3]; float u[3]; };")
    out.append("static const struct bdot_vector bdot_vectors[] = {")
    for _ in range(8):
        b0 = rng.normal(0, 3e-5, 3)
        rate = rng.normal(0, 2e-6, 3)  # T/s
        buf = np.array(f32([b0 + rate * 0.08 * k + rng.normal(0, 5e-8, 3) for k in range(6)])).reshape(6, 3)
        u = f32(acs.bdot_controller(buf, 0.08))
        rows = ", ".join(carr(r) for r in buf)
        out.append(f"\t{{{{{rows}}}, {carr(u)}}},")
    out.append("};")


def emit_sun(sun, out):
    cases = []
    err = -1
    for _ in range(10):  # lit, all sensors present
        s = unit(rng.normal(0, 1, 3))
        normals = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0],
                            [0.70710678118, 0, 0.70710678118], [0, -0.70710678118, 0.70710678118],
                            [-0.70710678118, 0, 0.70710678118], [0, 0.70710678118, 0.70710678118],
                            [0, 0, -1]])
        lux = np.clip(normals @ s, 0, None) * 120000 + rng.uniform(0, 400, 9)
        cases.append(f32(lux))
    lux = list(cases[0])
    lux[0] = lux[4] = err  # two failed sensors
    cases.append(lux)
    cases.append([err] * 9)  # SUN_NO_READINGS
    cases.append(f32([100.0] * 9))  # SUN_ECLIPSE
    cases.append(f32([50000, 0, 0, 0, 0, 0, 0, 0, 0]))  # only X lit: not enough
    cases.append(f32([0, 0, 0, 0, 0, 0, 0, 0, 80000]))  # only -Z lit: A = B = 0
    out.append("struct sun_vector { float lux[9]; int status; float sun[3]; };")
    out.append("static const struct sun_vector sun_vectors[] = {")
    for lux in cases:
        status, vec = sun.compute_body_sun_vector_from_lux(list(lux))
        out.append(f"\t{{{carr(lux)}, {status}, {carr(f32(vec))}}},")
    out.append("};")


def emit_allocator(acs, sat, out):
    out.append("struct alloc_vector { unsigned int mask; float u[3]; float b[3]; float throttle[6]; };")
    out.append("static const struct alloc_vector alloc_vectors[] = {")
    for mask in range(64):
        u = f32(rng.uniform(-1.5, 1.5, 3))
        b = f32(rng.normal(0, 3e-5, 3))
        sat.available = [bool(mask & (1 << i)) for i in range(6)]
        sat.applied = [0.0] * 6
        acs.mcm_coil_allocator(np.array(u), np.array(b))
        out.append(f"\t{{0x{mask:02x}, {carr(u)}, {carr(b)}, {carr(f32(sat.applied))}}},")
    out.append("};")


def emit_modes(consts, sensors, out):
    M = consts.Modes
    C = consts.ControllerConst
    thresholds = [M.VF_TUMBLING_TOL_BDOT, M.VF_TUMBLING_TOL, M.TUMBLING_TOL, M.DETUMBLED_TOL_LO,
                  M.DETUMBLED_TOL_HI, M.STABLE_TOL_LO, M.STABLE_TOL_HI, M.SUN_POINTED_TOL_LO,
                  M.SUN_POINTED_TOL_HI]
    target_omega = C.INERTIA_MAJOR_DIR * C.OMEGA_MAG_TARGET

    def ambiguous(omega, s):
        h = C.INERTIA_MAT @ omega
        quantities = [np.linalg.norm(omega), np.linalg.norm(C.INERTIA_MAJOR_DIR - h / C.MOMENTUM_TARGET_MAG)]
        if np.linalg.norm(h) > 0:
            quantities.append(np.linalg.norm(s - h / np.linalg.norm(h)))
        return any(abs(q - t) < 1e-3 for q in quantities for t in thresholds)

    out.append("struct mode_vector { int current; int controller; int gyro_status; float omega[3];")
    out.append("\t\t     int sun_status; float sun[3]; int next; };")
    out.append("static const struct mode_vector mode_vectors[] = {")
    count = 0
    while count < 300:
        kind = rng.integers(0, 3)
        if kind == 0:  # anywhere from still to very fast
            omega = unit(rng.normal(0, 1, 3)) * rng.uniform(0, 3.5)
        elif kind == 1:  # near the spin target
            omega = target_omega + rng.normal(0, 0.06, 3)
        else:  # slow, around the detumbled thresholds
            omega = unit(rng.normal(0, 1, 3)) * rng.uniform(0, 0.15)
        omega = np.array(f32(omega))
        h = C.INERTIA_MAT @ omega
        if rng.uniform() < 0.5 and np.linalg.norm(h) > 0:
            s = unit(h / np.linalg.norm(h) + rng.normal(0, 0.15, 3))
        else:
            s = unit(rng.normal(0, 1, 3))
        s = np.array(f32(s))
        if ambiguous(omega, s):
            continue
        current = int(rng.integers(0, 5))
        ctrl = int(rng.integers(0, 3))
        gyro_status = 21 if rng.uniform() < 0.05 else 0
        sun_status = 53 if rng.uniform() < 0.2 else 0
        nxt = sensors.update_mode(current, ctrl, gyro_status, omega, sun_status, s)
        out.append(f"\t{{{current}, {ctrl}, {gyro_status}, {carr(omega)}, {sun_status}, {carr(s)}, {nxt}}},")
        count += 1
    out.append("};")


def main():
    acs, consts, sensors, sun, sat = load_python_adcs(sys.argv[1])
    out = [
        "/* SPDX-License-Identifier: Apache-2.0 */",
        "/* Generated by test_vectors/adcs/gen_adcs_vectors.py from the Python",
        " * flight software. Do not edit by hand.",
        " */",
        "#ifndef ADCS_VECTORS_H_",
        "#define ADCS_VECTORS_H_",
        "",
    ]
    emit_controllers(acs, out)
    emit_sun(sun, out)
    emit_allocator(acs, sat, out)
    emit_modes(consts, sensors, out)
    out += ["", "#endif"]
    print("\n".join(out))


if __name__ == "__main__":
    main()
