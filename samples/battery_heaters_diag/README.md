# Battery heater diagnostic

Based on `python_implementation/scripts/battery_heater_test.py`. At boot it
turns both battery heaters on for and prints every 500 ms whether
each heater is enabled (its line and HEAT_EN both read back high) and any
`device_errors` codes. It then turns both heaters off, retrying until that
succeeds, and prints the final state. If enabling fails it skips straight to
turning them off. Unlike the Python script, it never leaves the heaters on.

| Code | Meaning |
|---|---|
| 0x26 | HEAT_EN readback differs from the commanded value |
| 0x27 | HEAT0_ON readback differs |
| 0x28 | HEAT1_ON readback differs |

## Build and flash

From the west workspace root:

```bash
west build -p always -b argus/rp2350b/m33_0 \
  -d build/battery_heaters_diag FSW-Mainboard-C/samples/battery_heaters_diag
```

Expected output:

```text
Enabling both heaters for 5000 ms: 0
Heater 0: 1 (0)  Heater 1: 1 (0)  errors: none
...
Disabled both heaters
Heater 0: 0 (0)  Heater 1: 0 (0)  errors: none
```
