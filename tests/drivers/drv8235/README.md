# DRV8235 regression tests

The real Zephyr driver runs against a small I2C register emulator. Tests use
their own binding and simulated bus, with no Board Library or regulator needed.

From a configured west workspace:

```sh
west twister -T FSW-Mainboard-C/tests/drivers/drv8235 \
  -p qemu_cortex_m3 --inline-logs -x EXTRA_CFLAGS=-Werror
```

On Linux (the CI platform):

```sh
west twister -T FSW-Mainboard-C/tests/drivers/drv8235 \
  -p native_sim/native/64 --inline-logs
```

The focused tests cover:

- Boot/init settings, disabling before protected writes, and reinitialization.
- Python/Rust throttle cases: normalized, volts and signed raw commands,
  clamping, quantization, getters, zero/brake and None/coast equivalents.
- Python-compatible voltage/current conversion in all four bridge modes,
  including nonzero brake current.
- Correct active-low NPOR, fault reporting/clearing, and reset recovery.
- I2C error propagation, failed command cleanup and invalid arguments.

The emulator models register defaults, protected fields and CLR_FLT/NPOR.
It does not model the coil, current decay, supply power or actual fault thresholds.
These tests establish software compatibility, not physical current calibration.

Reference cases:
[Python driver](https://github.com/cmu-argus-2/FSW-mainboard/blob/main/flight/hal/drivers/drv8235.py),
[Rust driver tests](https://github.com/cmu-argus-2/FSW-Embassy-Sandbox/blob/main/fsw-test/tests/drv8235.rs).

The hardware diagnostic still needs the real device definitions described in
[`samples/drv8235_diag/README.md`](../../../samples/drv8235_diag/README.md).
