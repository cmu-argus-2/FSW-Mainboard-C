# ADM1176 on-target smoke test

This application is a read-only hardware test for the ADM1176 devices on the
Argus v4 mainboard. It reads voltage, current, power, and status from the main,
GPS, and radio power monitors every two seconds. It does not change output
switching or alert thresholds.

The C repository uses its own `boards/cmu/argus/` board definition and
`dts/bindings/sensor/adi,adm1176.yaml` binding. It does not depend on
FSW-Zephyr-Board-Lib. Keep the ADM1176 nodes and binding synchronized with
the separate copies in that repository when changing hardware configuration.

The current configuration covers I2C1 addresses 0x40 (main), 0x41 (GPS),
and 0x42 (radio), each with a 10 milliohm shunt, following the previous
flight-software configuration. Verify these values against the board being
tested before interpreting current readings. Driver initialization starts
continuous voltage/current conversion; status reads also write the command
byte. This test does not intentionally switch any power rail.

Run the automated tests without hardware from the repository root:

```sh
west twister -T tests/drivers/adm1176 -p qemu_cortex_m3 --inline-logs
```

Passing these tests or compiling a UF2 does not replace an on-board test.

Build it from the repository root:

```sh
west build -p always -b 'argus@4/rp2350b/m33_0' \
  -s samples/adm1176_diag \
  -d build-adm1176-diag
```

The UF2 file is generated at:

```text
build-adm1176-diag/zephyr/zephyr.uf2
```

After flashing, connect to the board's USB serial console. Each detected
monitor should repeatedly print plausible voltage, current, power, and status
values. A missing or non-responding device prints `NOT READY` or a negative I2C
error code.

The sensor API returns `-ENODATA` before a successful sample and after a failed
sample. Each fetch restores measurement mode before reading, so a previous
status-read restore failure can recover on the next fetch.

Initialization leaves output switching and write-only alert registers intact.
After a controller-only reboot their previous values may still be present.
Before calling `adm1176_get_overcurrent_threshold()` or `adm1176_clear_alerts()`,
the application must explicitly configure a threshold successfully. That call
sets the requested threshold and enables hot-swap and four-consecutive-sample
ADC alerts. Unknown configuration returns `-ENODATA`; failed configuration or
clear writes invalidate the cached policy until configuration succeeds again.
The diagnostic application only reads telemetry and does not call these APIs.
