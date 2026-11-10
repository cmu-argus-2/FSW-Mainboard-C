# EPS telemetry task

The EPS task runs at 1 Hz. Each run samples four supply monitors and six
torque coils, then publishes an `EPS_WARNING` record with 11 values. Every
fifth run it also reads the fuel gauge and CPU temperature, updates the
battery power flag and heater decisions, and publishes an `EPS` record with
44 values in the Python `EPS_IDX` order.

The rules for power alerts, SOC hysteresis and heater hysteresis live in
`src/apps/eps`. The task and its runner live in `src/tasks`. The default
runner demonstrates the console output with unavailable hardware callbacks.
Hardware integration supplies driver-backed callbacks through `eps_io`.

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

## Driver values and temperature

The callbacks use driver-decoded measurements and supply the EPS units,
including mV, mA, seconds and centi-degrees Celsius. The DRV8235 driver PR
[#22](https://github.com/cmu-argus-2/FSW-Mainboard-C/pull/22) applies the agreed
current scale of 3.7 A / 192.

The MAX17205 driver from [#16](https://github.com/cmu-argus-2/FSW-Mainboard-C/pull/16)
decodes pack temperature as signed and reports it in 0.1 K increments. The
original EPS spec interprets the pack register as unsigned. An adapter should
use the driver-decoded pack temperature and convert its output to centi-degrees
Celsius, retaining the driver's resolution. The AIN1, AIN2 and die properties
already expose 0.1 K values. The EPS spec's offset for those channels is 2731,
while the driver demo uses 2732.

## Tests

`tests/tasks/eps` exercises the periodic task with fake callbacks. It checks
field order, thresholds, hysteresis, failure and recovery, and validity for
successful zero readings and unavailable measurements.

```sh
west build -p -b argus/rp2350b/m33_0 -d build/argus
west twister -T tests/tasks/eps -p native_sim/native/64 --inline-logs
```
