# FSW-mainboard scripts/mag_hyst.py, the Python side of samples/drv8235_mag_hyst.
# Unchanged except for the lines marked "added":
# - pet_watchdog(): boot_sequence() arms the external watchdog, and without a pulse on
#   its input the board resets after a few seconds, long before the ~90 s sweep ends.
#   The function is the one from the XM coil test.
# - Missing coils: a driver that failed to boot is None in TORQUE_DRIVERS, and the
#   original script stops there. This one sweeps only the coils that booted and prints
#   nan for the others, so every row still has all 18 columns.
#
# Copy to the board as code.py. Rows are plain CSV; compare_mag_hyst.py reads them.

import time

import supervisor
from hal.configuration import SATELLITE

print("Booting ARGUS...")
SATELLITE.boot_sequence()
print("ARGUS booted.")
print(f"Boot Errors: {SATELLITE.ERRORS}")


def pet_watchdog():  # added
    try:
        wdt = SATELLITE.WATCHDOG
        if wdt.input:
            wdt.input_low()
        wdt.input_high()
    except Exception:
        pass

pet_watchdog()  # added

supervisor.runtime.autoreload = False

COILS = ["XP", "XM", "YP", "YM", "ZP", "ZM"]
N_STEPS = 60
STEP_SIZE = 2.0 / N_STEPS  # Map 0-255 to -1.0 to 1.0
STEPS = [round(-1.0 + i * STEP_SIZE, 2) for i in range(N_STEPS + 1)]  # -1.0 to 1.0 in steps
SETTLE_TIME = 0.2  # seconds to wait after setting throttle before sampling

headers = [
    "coil",
    "step",
    "time_ns",
    "mag_x",
    "mag_y",
    "mag_z",
    "voltage_xp",
    "voltage_xm",
    "voltage_yp",
    "voltage_ym",
    "voltage_zp",
    "voltage_zm",
    "current_xp",
    "current_xm",
    "current_yp",
    "current_ym",
    "current_zp",
    "current_zm",
]
DRIVERS = SATELLITE.TORQUE_DRIVERS  # added
PRESENT = [c for c in COILS if DRIVERS.get(c) is not None]  # added
print("Coils swept: %s; missing: %s" % (PRESENT, [c for c in COILS if c not in PRESENT]))  # added


def read_all(read):  # added: nan for a missing coil
    return [read(DRIVERS[c]) if c in PRESENT else float("nan") for c in COILS]


print(",".join(headers))

for coil in PRESENT:  # changed: COILS -> PRESENT
    SATELLITE.TORQUE_DRIVERS[coil].set_throttle(0)

mag_measurement_times_ns = []

for coil in PRESENT:  # changed: COILS -> PRESENT
    for step in STEPS:
        for c in PRESENT:  # changed: COILS -> PRESENT
            SATELLITE.TORQUE_DRIVERS[c].set_throttle(step if c == coil else 0)

        pet_watchdog()  # added
        time.sleep(SETTLE_TIME)

        curr_time = int(1e9 * time.monotonic())
        mag_start_ns = time.monotonic_ns()
        mag = SATELLITE.IMU.mag()
        mag_end_ns = time.monotonic_ns()
        mag_measurement_times_ns.append(mag_end_ns - mag_start_ns)

        voltages = read_all(lambda d: d.__read_voltage())  # changed: nan if missing
        currents = read_all(lambda d: d.__read_current())  # changed: nan if missing

        row = [coil, step, curr_time] + list(mag) + voltages + currents
        print(",".join(map(str, row)))
        pet_watchdog()  # added

for coil in PRESENT:  # changed: COILS -> PRESENT
    SATELLITE.TORQUE_DRIVERS[coil].set_throttle(0)

if mag_measurement_times_ns:
    avg_mag_time_ns = sum(mag_measurement_times_ns) / len(mag_measurement_times_ns)
    print(f"Average mag measurement time (ns): {avg_mag_time_ns}")

while True:  # added: keep the watchdog fed after the sweep
    pet_watchdog()
    time.sleep(0.5)
