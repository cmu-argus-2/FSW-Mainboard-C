# DRV8235 board diagnostic

Interactive USB shell for the six Argus magnetorquer drivers. The sample overlay
keeps COIL_EN off at boot, and the driver does no chip access at boot. `arm` powers
the rail and initializes the chips with EN_OUT enabled at zero/coast, matching
Python/Rust. Failed initialization can be retried without rebooting.

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

From the west workspace root (the shared CMake helper fetches the pinned Board
Library commit on first configure, requiring network access):

```bash
west build -p always -b argus@4/rp2350b/m33_0 \
  -d build/drv8235_diag FSW-Mainboard-C/samples/drv8235_diag -S argus-common
```

The sample explicitly selects the fetched Board Library's `boards/argus`, avoiding
duplicate `argus` definitions from the C repo and Board Library. The main
application still uses its existing board. For a nonstandard checkout location,
pass `-- -DARGUS_BOARD_LIB=/absolute/path/to/FSW-Zephyr-Board-Lib` (at commit
`4483fe67a5bf866f25d9d3c5868d3a2e10f0aeaa`) instead of
manually adding another board root.

Hold BOOT while connecting the mainboard, then copy build/drv8235_diag/zephyr/zephyr.uf2
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

- Boot: SAFE START: COIL_EN is OFF. List reports software readiness only.
  Hardware connectivity is tested by arm; a failed arm removes shared power.
  Check wiring/power/address, then retry arm without rebooting.
- Arm: restores chip configuration after power-up, with bridges enabled at target
  zero in coast. It does not deliberately drive a coil.
- Status: mode=coast and fault=0x02 are normal immediately after arm.
- Pulse: command voltage is in mV and duration in ms. Positive is forward,
  negative is reverse, and zero selects brake. Each pulse ends in coast,
  with EN_OUT still enabled; disarm removes the shared rail power.
- A status line reports voltage in mV, current in mA and raw fault status.
  During a pulse readings are saved in memory and printed only after stopping
  the output, so a blocked USB console cannot hold the pulse on. The saved mode
  and readings describe the pulse, not the subsequent coast state.
  Exact readings depend on hardware. Fault reads clear reported faults after
  selecting zero/coast, consistent with the driver API's documented side effect.

Repeat the low-voltage pulses for the other axes after the first axis passes.
The diagnostic restricts requested voltage to +/-6000 mV and duration to
1..5000 ms. The driver also enforces the DT coil voltage limit on every command
path. A 6000 mV request is capped at code 35 (about 5.86 V nominal), since
code 36 would exceed 6 V. This is a register-command limit, not an analog clamp.

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
Normalized throttle scales to the DT limit; volts/raw reject commands above
that limit. Applications own COIL_EN and must call drv8235_init()
after cycling it. No separate enable, telemetry-validity or calibration API is used.
