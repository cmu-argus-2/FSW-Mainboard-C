# EPS telemetry task

After hardware initialization, `main` starts a dedicated Zephyr thread for EPS
and returns. EPS uses preemptive priority 2 (matching the Argus 2 EPS priority
number) and reserves its own 4 KiB stack. It runs at 1 Hz against absolute
deadlines and skips missed ticks. Each run samples four supply monitors and six
torque coils, then publishes an `EPS_WARNING` record with 11 values. Every
fifth run it also reads the fuel gauge and CPU temperature, updates the
battery power flag and heater decisions, and publishes an `EPS` record with
44 values in the Python `EPS_IDX` order.

The rules for power alerts, SOC hysteresis and heater hysteresis live in
`src/apps/eps`. The default firmware supplies driver-backed callbacks through
`eps_io` and publishes both records to the console. Their timestamps are
uptime seconds.

At startup EPS enables the GPS supply using `gps-en-gpios`. The radio
ADM1176 initializes after EPS enables its switched rail. The
Jetson rail is controlled separately, so Jetson readings depend on its
power state. EPS configures both heater outputs and their common enable
inactive at startup.

The local v4 board definition provides the `coil_power` regulator on GPIO 1
and six DRV8235 nodes. EPS enables the rail and initializes the bridges at
zero target in coast before reading them. The negative axes (XM, YM, ZM) are
on I²C0 and the positive axes (XP, YP, ZP) are on I²C1. Each bus uses
addresses `0x30`, `0x31` and `0x33` for X, Y and Z respectively.

## Read validity

Each console record is followed by a companion line with the same time and a
hexadecimal read-validity mask. The labels are `EPS_WARNING_VALIDITY` and
`EPS_VALIDITY` for the warning and full records, respectively. The numeric
fields in the original records retain their count and order.

| Bits | Read group |
| --- | --- |
| 0–3 | Main, peripheral, radio, Jetson |
| 4–9 | XP, XM, YP, YM, ZP, ZM coils |
| 10 | CPU temperature (full record) |
| 11 | Fuel gauge (full record) |

A set bit marks a successful read, including a measured zero. A clear bit
marks the corresponding group as unavailable. For example, a mainboard current
of zero is a valid reading if bit 0 is set and a placeholder if bit 0 is clear.

The `eps_hardware` log names the device and reports its error code on
I/O failure. It logs a new failure after a successful operation or when
the error code changes, while the validity mask records every failed
sample. A board without a ready coil supply also gets a startup message.

## Driver values and temperature

The callbacks use driver-decoded measurements and supply the EPS units,
including mV, mA, seconds and centi-degrees Celsius. The DRV8235 driver PR
[#22](https://github.com/cmu-argus-2/FSW-Mainboard-C/pull/22) applies the agreed
current scale of 3.7 A / 192.

The MAX17205 driver decodes pack temperature as signed Celsius and adds
2732 to its deci-degree value for the Zephyr API. EPS reverses that encoding.
AIN1, AIN2 and die expose absolute temperatures in 0.1 K, converted to
centi-degrees Celsius as `reading * 10 - 27315`. TTE and TTF are converted
from the driver's whole minutes to seconds.

## Tests

`tests/tasks/eps` exercises the periodic task with fake callbacks. It checks
field order, thresholds, hysteresis, failure and recovery, and validity for
successful zero readings and unavailable measurements.
`tests/tasks/eps_hardware` runs the production EPS callbacks with the existing
I²C driver emulators and emulated GPIO and coil regulator. This exercises
driver values through to EPS records, validity after read failures, and
both heater outputs and their common enable. A ten-run thread test checks
the publisher's thread and priority, 1 Hz timing, and full records on runs
5 and 10. On ARM QEMU it also reports the observed EPS stack high-water mark.

```sh
west build -p -b argus/rp2350b/m33_0 -d build/argus
west twister -T tests/tasks/eps -p native_sim/native/64 --inline-logs
west twister -T tests/tasks/eps_hardware -p native_sim/native/64 --inline-logs
```

## On-board checks

With the v4 mainboard and battery board connected, check warning bits 0–1
and full-record bits 10–11 for main/GPS rail, CPU temperature and fuel-gauge
telemetry. Check bit 2 with the radio rail powered and bit 3 with the Jetson
rail powered. Compare voltage, current and temperatures against the
corresponding device diagnostics.

With all six coil devices connected and all reads successful, the warning
mask is `0x3ff` and the full mask is `0xfff`.
Verify that one unavailable device clears only its own bit and that a valid
zero still sets its bit. On the battery board, verify both heater channels
start off and the shared enable stays active while either channel is on.
