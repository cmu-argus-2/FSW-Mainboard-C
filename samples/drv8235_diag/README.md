# DRV8235 board diagnostic

Interactive USB shell for the six Argus magnetorquer drivers. At startup it
coasts responding bridges and switches COIL_EN off. During device initialization
the rail is on and EN_OUT is enabled at target zero/coast, matching Python/Rust.

## Hardware and build

Use the matching Argus v4 mainboard, six connected DRV8235 modules and correct
coil harnesses. Verify the supply/ground and use a current-limited bench supply.
Keep a way to remove power immediately. Start with 500 mV, not the maximum.

| Axis | Bus | 7-bit I2C address |
|---|---|---|
| xm, ym, zm | I2C0 | 0x30, 0x31, 0x33 |
| xp, yp, zp | I2C1 | 0x30, 0x31, 0x33 |

The diagnostic needs the six device nodes and coil_power regulator in
[Board Library PR #2](https://github.com/cmu-argus-2/FSW-Zephyr-Board-Lib/pull/2).
This C PR does not migrate or modify the board files.

From the west workspace root, replace the Board Library path below:

```bash
west build -p always -b argus@4/rp2350b/m33_0 \
  FSW-Mainboard-C/samples/drv8235_diag -S argus-common -- \
  -DZEPHYR_EXTRA_MODULES=/absolute/path/to/FSW-Zephyr-Board-Lib
```

Hold BOOT while connecting the mainboard, then copy build/zephyr/zephyr.uf2
to its RP2350 USB drive. Find and open the new serial device:

```bash
ls /dev/cu.usbmodem*
screen /dev/cu.usbmodem2101 115200
```

Use your actual device name. Exit screen with Ctrl+A, then backslash.

## Manual sequence

Enter these in the board's uart shell, one at a time:

```text
drv8235 list
drv8235 arm I_UNDERSTAND
drv8235 status all
drv8235 pulse xp 500 1000
drv8235 status xp
drv8235 pulse xp -500 1000
drv8235 disarm
```

Expected observations:

- Boot: SAFE START: COIL_EN is OFF. All six list entries must be ready before arm.
  NOT READY requires checking power/connection/address and rebooting.
- Arm: restores chip configuration after power-up, with bridges enabled at target
  zero in coast. It does not deliberately drive a coil.
- Status: mode=coast and fault=0x02 are normal immediately after arm.
- Pulse: command voltage is in mV and duration in ms. Positive is forward,
  negative is reverse, and zero selects brake. Each pulse ends in coast,
  with EN_OUT still enabled; disarm removes the shared rail power.
- A status line reports voltage in mV, current in mA and raw fault status.
  Exact readings depend on hardware. Fault reads clear reported faults after
  selecting zero/coast, consistent with the driver API's documented side effect.

Repeat the low-voltage pulses for the other axes after the first axis passes.
The diagnostic restricts requested voltage to +/-6000 mV and duration to
1..5000 ms. Voltage encoding rounds like Python; a 6000 mV request encodes
approximately 6024 mV, so this is not a strict physical 6 V ceiling.

A coast failure attempts shared-rail shutdown; if the rail remains on, remove
external power. Pulse timing is software-based and the shell is occupied during
a pulse: entering disarm or Ctrl+C is not an independent emergency cutoff.

## Reading semantics

Voltage uses the original 0.16733 V per count. Current follows the team's EPS
exception: raw * 3.7 / 192 A (raw * 3700 / 192 mA), rather than Python's
0.01451 A per count. Reverse negates both by command convention; IMTR does not
measure current direction. Brake retains a nonzero IMTR reading. Coast can return a
stale/unsensed reading, which must not be interpreted as a valid zero or as
battery-input current. Current calibration is still pending hardware confirmation.

The driver itself exposes the original normalized, volts and raw command paths.
Only normalized throttle has the original 6 V scaling; volts/raw cover the
device's full range. Applications own COIL_EN and must call drv8235_init()
after cycling it. No separate enable, telemetry-validity or calibration API is used.
