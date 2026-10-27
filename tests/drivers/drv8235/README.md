# DRV8235 regression tests

These Zephyr Ztests run the real driver against an I2C register emulator.
Every test begins with chip power-on defaults and calls the driver to configure
the device. The emulator records writes, rejects changes to data-sheet `*`
fields while EN_OUT is set, and models NPOR and explicit fault clearing.
The Devicetree files under this test's `boards/` directory and the binding under
`dts/bindings/` are test fixtures; they do not change the Argus board definition.
The test CMake project registers its own DTS root before loading Zephyr, so
tests require no Board Library checkout or `ZEPHYR_EXTRA_MODULES` argument.

Run from a configured west workspace with Zephyr SDK and QEMU available:

```sh
west twister -T FSW-Mainboard-C/tests/drivers/drv8235 \
  -p qemu_cortex_m3 --inline-logs \
  -x EXTRA_CFLAGS=-Werror
```

On Linux, the same self-contained test also runs on the platform used by CI:

```sh
west twister -T FSW-Mainboard-C/tests/drivers/drv8235 \
  -p native_sim/native/64 --inline-logs
```

The test-only binding declares the property types used by the driver. Test
overlays provide all limits explicitly, including the provisional current scale.
It does not replace validation of the production hardware binding; the
diagnostic sample still requires Board Library PR #2.

The tests cover reset/configuration validation, safe fault clearing, latch-off
settings, write ordering, same-direction updates, failed I2C writes and lost
writes, voltage quantization limits, separate VMTR scaling, invalid IMTR data,
and rail-off refusal. Brake telemetry preserves nonzero coil-current magnitude;
forward/reverse commands do not invent a measured sign. Coast, disabled output,
fault/reset, unconfigured state, and pin-control mode mark current unavailable
while retaining the raw register and other readings. They also exercise the diagnostic's bridge-disable
fallback with successful and failed shared-rail shutdowns.

The current-conversion tests verify arithmetic using the existing provisional
scale; they do not establish the physical IMTR calibration. The emulator does
not model inductance, switching waveforms, current decay, heating, or real
fault thresholds. Those require hardware measurements.
