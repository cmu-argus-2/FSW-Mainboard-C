# DRV8235 board diagnostic

This sample provides an interactive USB serial shell for the six Argus
magnetorquer H-bridges. It starts safe: `COIL_EN` is low, every responding
DRV8235 is disabled, its target is zero, and its bridge is in coast mode.

## Before powering the board

1. Connect the six face-board magnetorquer modules to their correct harnesses.
2. Use a current-limited bench supply and verify the coil rail and ground.
3. Keep a way to remove power immediately. Start at a low command such as
   `500` mV; do not begin at the 6000 mV software limit.
4. Keep magnetically sensitive hardware away from the test area.

The address map is:

| Axis | Bus | 7-bit I2C address |
|---|---|---|
| `xm`, `ym`, `zm` | I2C0 | `0x30`, `0x31`, `0x33` |
| `xp`, `yp`, `zp` | I2C1 | `0x30`, `0x31`, `0x33` |

## Build and flash

From the west workspace root:

```bash
west build -p always -b argus@4/rp2350b/m33_0 \
  FSW-Mainboard-C/samples/drv8235_diag -S argus-common
```

Hold BOOT while plugging in the mainboard, then copy
`build/zephyr/zephyr.uf2` to the RP2350 USB drive. Open the new USB serial
device, for example:

```bash
screen /dev/tty.usbmodem* 115200
```

## Safe test sequence

```text
drv8235 list
drv8235 arm I_UNDERSTAND
drv8235 status all
drv8235 pulse xp 500 1000
drv8235 disarm
```

Expected observations:

- Boot prints `SAFE START: COIL_EN is OFF`.
- With all face boards connected, `list` reports all six axes as `ready`.
- Arming powers the driver rail, restores each chip's configuration after that
  power cycle, and refuses to continue if any device is missing or faulted.
- Immediately after arming, every status line should say `enabled=no`,
  `mode=coast`, and `fault=0x00`.
- `pulse xp 500 1000` drives +X at 500 mV for at most one second, prints live
  telemetry once, and automatically returns the bridge to disabled/coast. The
  exact current and duty depend on the supply and coil. A detected fault aborts
  the pulse early.
- `disarm` first disables and coasts all bridges, then drives the shared rail
  low.

If an axis says `NOT READY`, do not arm. Check face-board power, harness order,
SDA/SCL continuity, and the expected bus/address. If `fault` is nonzero, record
the byte and inspect the individual fault flags in the flight-software logs
before clearing it. Pressing reset or removing power also returns to the safe
startup path.
