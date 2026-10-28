# EPS task for issue 21

Branch: `21-to-doeps-tasks`. Scope: implement the EPS Task Spec, print to the
console, and test the task. Existing local board files are retained. There is
no board migration and no dependency on an unpublished shared-board commit.

## Protected files

These paths are byte-for-byte unchanged from `origin/21-to-doeps-tasks`:

- `boards/**`
- `west.yml`
- `zephyr/module.yml`
- `.github/**`
- `README.md`

The earlier experimental shared-board migration is excluded from this branch.
An unpushed scratch commit in the separate `eps-board` checkout is not used.

## EPS behavior

- Read four supply monitors and six coils every second; print an eleven-field
  `EPS_WARNING` record.
- On invocations 5, 10, 15, ... read battery and CPU temperature, update the SOC
  flag, control both heaters, and print 44 numeric fields in Python EPS_IDX order.
- Retain the document's units, truncation, unsigned pack-temperature encoding,
  2731 Kelvin offset, alert thresholds, temperature fallback and hysteresis.
- The one approved formula exception is DRV8235 current: raw * 3700 / 192 mA.
  DRV voltage is still raw * 16733 / 100 mV. Negate both when REVERSE.
- TTE/TTF retain the existing gauge API's integer-minute resolution, converted
  to seconds for logging. They do not control EPS state or heaters.
- Solar slots 34 through 41 stay zero. Timestamp is uptime in integer seconds.
- Missing nodes/devices are skipped and their measurements stay zero. Failed
  reads clear that device's current measurement group. Gauge failure uses CPU
  temperature minus 200 centi-Celsius; when both temperatures fail, heaters are
  requested off. No fault recovery or other subsystem task is introduced.

## Application integration and dependencies

`src/tasks/eps.c` contains pure task logic, conversions and record formatting.
`include/argus/tasks/eps.h` defines the data layout and injectable I/O contract.
`src/tasks/eps_zephyr.c` connects that contract to the existing drivers, CPU
sensor, GPIO and console. `src/main.c` invokes one step at absolute 1-second
deadlines. `CMakeLists.txt` adds these sources and conditionally builds the
driver dependencies only when corresponding device nodes are enabled.
`prj.conf` enables application features. `app.overlay` ONLY enables the
existing ADC/die-temperature sensor; it does not add buses, pins, supply-monitor,
coil or fuel-gauge nodes.

The target branch starts with MAX17205 placeholders and no other driver sources.
The following are existing dependency snapshots needed by EPS and its integration
tests, not new driver features authored for this issue:

- DRV8235 implementation/header from `8a6eb42` on
  `18-to-do-implement-drv8235-driver`; its conversion/control logic is unchanged.
- MAX17205 implementation/header from `63400a0` on `max17205`.
- The pre-existing local Zephyr ADM1176 implementation in the original
  `FSW-Mainboard-C` checkout. That dirty checkout remains untouched.
- Their local driver schemas in `dts/bindings/**` and regression test fixtures.
  Schemas describe driver properties, not board pin mappings or device addresses.
  The unrelated DRV diagnostic sample is not imported.

Two small accessors were added for EPS compatibility:
`adm1176_read_sample()` returns a locked sample including raw ADC codes so the
task applies the specified truncation; `max17205_read_raw_temperatures()` preserves
the document's raw temperature encoding. Existing Zephyr sensor/fuel-gauge
properties keep their original behavior.

Board-node integration remains a separate dependency. The branch's untouched
board definition currently has no ADM1176, DRV8235 or MAX17205 nodes. The default
application builds and reports those devices unavailable; it must not be described
as having physically read them. When board support supplies the expected nodes,
the same adapter builds the driver dependencies and uses them. Full data flow is
tested using application-local emulator overlays, without altering `boards/`.

## References

1. Scope: [GitHub issue 21](https://github.com/cmu-argus-2/FSW-Mainboard-C/issues/21),
   whose body requests implementing EPS from the linked task specification.
2. Requirements: the provided `EPS Task Specs.docx`, plus the team's explicit
   decisions in this conversation (192 instead of 255; logging-only TTE/TTF).
3. Python reference commit `b428c544b22b5e6885b81c8a275afa4106e1a2d5`:
   - `flight/tasks/eps.py`: sampling, fuel readings, alerts and heater orchestration.
   - `flight/apps/eps/eps.py`: SOC/temperature hysteresis and power thresholds.
   - `flight/core/dh_constants.py`: 44 telemetry and 11 warning field indices.
   - `flight/core/task_configuration.py`: 1 Hz EPS scheduling.
   - `flight/hal/argus_v4.py`: device associations and heater GPIO roles.
   - `flight/hal/drivers/adm1176.py`, `max17205.py`, `drv8235.py`,
     `batteryheaters.py`: legacy read/convert/control behavior.
4. Existing C driver dependency commits are listed above; only the two raw-read
   accessors are EPS-specific additions.
5. Zephyr reference revision `839728050444`: sensor, fuel-gauge, GPIO, kernel
   timing, Ztest and I2C emulator APIs, plus the existing die-temperature sample.
6. CI requirements are read from the branch's unchanged
   `.github/workflows/build.yml`.

## CI and validation

Each test CMake directory has a `testcase.yaml`. Every added suite allows
`native_sim/native/64`, exactly as existing CI requires. ARM QEMU is additionally
allowed for local macOS execution. No workflow or skip-check rule is weakened.

```sh
west build -p -b argus/rp2350b/m33_0 -d build/argus
west twister -T tests -p native_sim/native/64 --inline-logs --report-filtered
# Local macOS alternative:
west twister -T tests -p qemu_cortex_m3 --inline-logs
```

The 15 EPS unit cases cover schedule, field order, thresholds, hysteresis,
conversion/truncation, failure isolation and CSV format. Five integration cases
exercise the real drivers through two I2C emulator buses, CPU/GPIO emulation and
the raw-read accessors. Driver regressions are also included.

Physical-board measurements and heater actuation have not been validated.
Linux native-sim execution must be distinguished from the local QEMU result.

Revalidated after removing the board migration on 2026-10-28: all 106 executed
QEMU cases passed (20 EPS cases and 86 driver regressions). The existing
native-sim-only placeholder is filtered only on QEMU. All six test directories
have test metadata and all six suites explicitly allow the CI target
`native_sim/native/64`. This is a static CI-configuration check, not a claim of
running the Linux native simulator on macOS. RP2350B revision 4 firmware builds
using this repo's unchanged local board files, without any external board module.

## Complete changed-file inventory

`M` is modified relative to the issue branch; unmarked paths are new files.
Protected board, west, module and CI paths are absent from this inventory.

```text
M	CMakeLists.txt
M	include/argus/drivers/max17205.h
M	prj.conf
M	src/drivers/max17205.c
M	src/main.c
app.overlay
docs/eps_task.md
dts/bindings/fuel-gauge/maxim,max17205.yaml
dts/bindings/motor/ti,drv8235.yaml
dts/bindings/sensor/adi,adm1176.yaml
include/argus/drivers/adm1176.h
include/argus/drivers/drv8235.h
include/argus/tasks/eps.h
src/drivers/adm1176.c
src/drivers/adm1176_common.c
src/drivers/drv8235.c
src/drivers/drv8235_common.c
src/tasks/eps.c
src/tasks/eps_zephyr.c
tests/drivers/adm1176/CMakeLists.txt
tests/drivers/adm1176/boards/native_sim_native_64.overlay
tests/drivers/adm1176/boards/qemu_cortex_m3.overlay
tests/drivers/adm1176/prj.conf
tests/drivers/adm1176/src/adm1176_emul.c
tests/drivers/adm1176/src/adm1176_emul.h
tests/drivers/adm1176/src/driver_api.c
tests/drivers/adm1176/src/main.c
tests/drivers/adm1176/testcase.yaml
tests/drivers/drv8235/CMakeLists.txt
tests/drivers/drv8235/boards/native_sim_native_64.overlay
tests/drivers/drv8235/boards/qemu_cortex_m3.overlay
tests/drivers/drv8235/prj.conf
tests/drivers/drv8235/src/drv8235_emul.c
tests/drivers/drv8235/src/drv8235_emul.h
tests/drivers/drv8235/src/hardware.c
tests/drivers/drv8235/src/main.c
tests/drivers/drv8235/testcase.yaml
tests/drivers/max17205/CMakeLists.txt
tests/drivers/max17205/boards/native_sim.overlay
tests/drivers/max17205/boards/native_sim_native_64.overlay
tests/drivers/max17205/boards/qemu_cortex_m3.overlay
tests/drivers/max17205/max17205_nodes.dtsi
tests/drivers/max17205/prj.conf
tests/drivers/max17205/src/main.c
tests/drivers/max17205/src/max17205_emul.c
tests/drivers/max17205/src/max17205_emul.h
tests/drivers/max17205/testcase.yaml
tests/tasks/eps/CMakeLists.txt
tests/tasks/eps/prj.conf
tests/tasks/eps/src/main.c
tests/tasks/eps/testcase.yaml
tests/tasks/eps_integration/CMakeLists.txt
tests/tasks/eps_integration/boards/native_sim_native_64.overlay
tests/tasks/eps_integration/boards/qemu_cortex_m3.overlay
tests/tasks/eps_integration/eps_nodes.dtsi
tests/tasks/eps_integration/prj.conf
tests/tasks/eps_integration/src/main.c
tests/tasks/eps_integration/testcase.yaml
```
