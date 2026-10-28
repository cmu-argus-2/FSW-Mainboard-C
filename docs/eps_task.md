# EPS task only

Issue: [21](https://github.com/cmu-argus-2/FSW-Mainboard-C/issues/21).
Branch: `21-to-doeps-tasks`.

This change implements the EPS task logic and its own tests. It does not
implement, import, or modify hardware drivers, bindings, or driver tests.

## Behavior

`src/tasks/eps.c` implements one step of the 1 Hz task:
- Read four monitor and six coil values through the injected `eps_io` callbacks.
- Calculate threshold alerts and publish eleven warning fields each run.
- Every fifth run, read battery/CPU data, update SOC state, apply two heater
  decisions, and publish 44 telemetry fields in the Python EPS_IDX order.
- Preserve the specified units, truncation, thresholds, temperature fallback,
  unsigned pack-temperature encoding and 2731 offset.
- Retain the team's one conversion exception: coil current uses raw * 3700 / 192
  mA. Coil voltage uses raw * 16733 / 100 mV; both are negated in REVERSE.
- Keep solar slots zero and skip failed/unavailable measurements.
- Keep TTE/TTF for logging only; I/O integration may retain minute resolution.

`include/argus/tasks/eps.h` defines the data types, field order and I/O contract.
The callbacks supply readings in the documented units and perform the requested
heater operation. A negative errno reports a failure/unavailable device.
Implementing or changing those hardware drivers is a separate task.

`src/tasks/eps_runner.c` calls `eps_step()` at absolute one-second deadlines.
Real hardware integration supplies an `eps_io` provider to `eps_run()`.
The default `src/main.c` explicitly uses a console-only provider: all hardware
callbacks return `-ENODEV`. It prints zero/unavailable telemetry and never
accesses peripherals or actuates heaters. This is not hardware integration and
must not be represented as a successful sensor read.

## Scope

Only these paths differ from the issue's original base `9c8d027`:
- `include/argus/tasks/eps.h`
- `src/tasks/eps.c`
- `src/tasks/eps_runner.c`
- `src/main.c` (start the EPS runner)
- `CMakeLists.txt` (add only the two task sources)
- `prj.conf` (main-thread stack size for console formatting)
- `tests/tasks/eps/**`
- `docs/eps_task.md`

`src/drivers/**`, `include/argus/drivers/**`, `dts/**`,
`tests/drivers/**`, `boards/**`, `west.yml`, `zephyr/module.yml`,
`.github/**` and `README.md` are unchanged from that base. No app overlay,
board migration, shared-board pin, copied driver implementation or driver API
extension is part of this change.

## References

- The supplied EPS Task Specs.docx and the team's decisions in this conversation.
- Python reference `b428c544b22b5e6885b81c8a275afa4106e1a2d5`:
  `flight/tasks/eps.py`, `flight/apps/eps/eps.py`,
  `flight/core/dh_constants.py`, `flight/core/task_configuration.py`.
- Zephyr `839728050444`: kernel scheduling, console and Ztest APIs.
- The branch's existing CI workflow, left unchanged.

## Tests

`tests/tasks/eps` uses fake task-level callbacks only, with no I2C emulator,
hardware driver, sensor API stand-in or Devicetree fixture. Tests cover scheduling,
conversion/truncation, the 192 exception, all field orders, thresholds,
SOC/temperature hysteresis, missing readings, CPU fallback, heater decisions
and CSV format. The console-only provider is checked to return unavailable
without fabricating measurements.

The suite has `testcase.yaml` and allows `native_sim/native/64` for the existing
Linux CI, plus `qemu_cortex_m3` for local macOS execution.

Local validation: all 16 EPS tests passed on `qemu_cortex_m3`, and the
`argus/rp2350b/m33_0` firmware built successfully and generated `zephyr.uf2`.
The Linux CI run and real hardware integration remain separate validation.

```sh
west build -p -b argus/rp2350b/m33_0 -d build/argus
west twister -T tests/tasks/eps -p native_sim/native/64 --inline-logs
# macOS:
west twister -T tests/tasks/eps -p qemu_cortex_m3 --inline-logs
```
