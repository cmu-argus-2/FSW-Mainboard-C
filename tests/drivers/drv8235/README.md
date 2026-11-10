# DRV8235 regression tests

The real Zephyr driver runs against a small I2C register emulator. Tests use
the same binding as the hardware, a simulated bus and a fake regulator.
The shared CMake helper pins and fetches the Board Library on first configure.
Network is required for that fetch; for offline runs add
`-x ARGUS_BOARD_LIB=/absolute/path/to/FSW-Zephyr-Board-Lib` to Twister.

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

- No chip access at boot, supply-off rejection, retry after failed probe,
  init settings and disabling before protected writes.
- Python/Rust throttle cases: normalized, volts and signed raw commands,
  normalized clamping, quantization, getters, zero/brake and None/coast equivalents.
- Coil voltage limits on all command paths, including the 6 V rounding boundary,
  and rejection without altering the previous output.
- Both reversal directions coast before writing the new magnitude; same-direction
  updates do not insert coast. Register write order is checked.
- Two devices with different voltage, coil resistance and current full scale
  demonstrate that DT properties are used rather than hardcoded.
- Python-compatible voltage/sign behavior and the team's EPS current conversion
  (raw * 3.7 / 192 A) in all four modes, including nonzero brake current.
- Correct active-low NPOR, fault reporting/clearing, reset recovery and setters
  remaining usable with a latched FAULT (including zero/brake).
- I2C error propagation, failed command cleanup and invalid arguments.

The emulator models register defaults, protected fields and CLR_FLT/NPOR.
It does not model the coil, current decay, supply power or actual fault thresholds.
These tests establish software compatibility, not physical current calibration.

Reference cases:
[Python driver](https://github.com/cmu-argus-2/FSW-mainboard/blob/main/flight/hal/drivers/drv8235.py),
[Rust driver tests](https://github.com/cmu-argus-2/FSW-Embassy-Sandbox/blob/main/fsw-test/tests/drv8235.rs).

The hardware diagnostic still needs the real device definitions described in
[`samples/drv8235_diag/README.md`](../../../samples/drv8235_diag/README.md).
