# MAX17205 demo

Prints every property of the MAX17205 fuel gauge every 2 seconds over the USB serial
console. Use it to check the driver on real hardware.

## Build and flash

From the repo root:

```bash
west build -p -b argus/rp2350b/m33_0 samples/max17205_demo
```

If this checkout isn't inside a west workspace, add `-- -DBOARD_ROOT=$PWD` so the
Argus board is found.

Hold BOOT while plugging in the mainboard, then copy `build/zephyr/zephyr.uf2` to the
RP2350 USB drive. Open the new USB serial device, for example:

```bash
screen /dev/tty.usbmodem* 115200
```

The demo waits 5 s after boot so the console can connect, then turns on the
peripheral 3V3 rail (PERIPH_PWR_EN) before the first read.

## Expected output

```text
--- MAX17205 snapshot 3 (uptime 11234 ms) ---
  State of charge      87 %
  Remaining capacity   2610.000 mAh
  Current              -152.343 mA (discharging)
  Pack voltage         8012.500 mV
  ...
  Time since power-up  11520 s
  ...
```

A property that fails to read prints `ERROR -N` instead of a value, and the snapshot
ends with how many reads failed:

| Error | Meaning |
|---|---|
| Every property fails with `-5` | Nothing answers at `0x36`: battery board missing, not powered, or not connected. |
| Only the AIN1, AIN2 and die temperatures fail | The gauge answers at `0x36` but not at the shadow-RAM address `0x0B`. |
| `fuel gauge not ready` | The I2C bus itself didn't come up. |

## What to check

1. Pack voltage matches a multimeter across the pack.
2. Current changes sign when the charger is plugged in or removed.
3. Temperatures read close to room temperature.
4. Time since power-up only moves in 11520 s (3.2 h) steps, so it reads 0 for the
   first 3.2 hours after the gauge powers up.

Set `DEMO_RESET_ON_BOOT` to 1 in `src/main.c` to call `max17205_reset()` once at boot.
This restarts the gauge's learning, so it is off by default.
