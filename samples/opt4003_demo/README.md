# OPT4003 demo

Prints the state and lux reading of every OPT4003 light sensor once a second over the
USB serial console. Use it to check the driver on real hardware.

## Build and flash

From the repo root:

```bash
west build -p -b argus/rp2350b/m33_0 samples/opt4003_demo
```

If this checkout isn't inside a west workspace, add `-- -DBOARD_ROOT=$PWD` so the
Argus board is found.

Hold BOOT while plugging in the mainboard, then copy `build/zephyr/zephyr.uf2` to the
RP2350 USB drive. Open the new USB serial device, for example:

```bash
screen /dev/tty.usbmodem* 115200
```

## Expected output

```text
--- OPT4003 demo, tick 12
LIGHT_XM     init failed: I2C error at 0x44 (-5)
...
LIGHT_XP         412.307 lux
...
```

Each sensor shows one of:

| State | Meaning |
|---|---|
| `NNN.NNN lux` | Working. Add `OVERLOAD` if the sensor reported overload. |
| `init failed: I2C error at 0xNN (-5)` | With no log line next to it, nothing answered: not populated or not connected. With an `ARB lost` log line, something on the bus is pulling SDA low: a hardware problem on that bus. |
| `init failed: device ID 0xNNNN` | Something answers but it isn't an OPT4003 with ID `0x0221`. `0x0121` would be an OPT4001. |
| `init failed at boot, but answers now` | The sensor is fine now but wasn't answering when init ran at boot: a power-up timing problem. |
| `read failed (-N)` | A fetch failed: `-116` timeout (conversion never ready), `-5` CRC mismatch, others are I2C errors. |

The driver also logs why init failed, but at boot, before the USB console connects, so
that log line is usually missed. The demo repeats the ID read so you can see it.

## What to check

1. A connected sensor doesn't show `init failed`. If it does, either it isn't answering on
   I2C or the `0x221` device ID in the driver is wrong for this part.
2. Indoors, readings are typically a few hundred lux.
3. Covering the sensor drops the reading to near 0; a phone flashlight raises it to
   thousands of lux.
4. Readings change smoothly with no `read failed (-5)` lines (CRC mismatches).
