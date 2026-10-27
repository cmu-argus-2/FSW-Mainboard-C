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

This sample needs the DRV8235 binding, six device nodes, and `coil_power`
regulator from Board Library PR #2. The application's existing board files
do not contain those definitions. No board migration is performed by this
driver change.

From the west workspace root, substitute the absolute path to that Board
Library checkout (which supplies the board and `argus-common` snippet):

```bash
west build -p always -b argus@4/rp2350b/m33_0 \
  FSW-Mainboard-C/samples/drv8235_diag -S argus-common -- \
  -DZEPHYR_EXTRA_MODULES=/absolute/path/to/FSW-Zephyr-Board-Lib
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
  `mode=coast`, and `fault=0x02` (NPOR=1 means the power-up indication has
  been cleared; this is the normal status, not a fault).
- `pulse xp 500 1000` commands +X at 500 mV for a requested one second, prints live
  telemetry once, and automatically returns the bridge to disabled/coast. The
  exact current and duty depend on the supply and coil. A detected fault aborts
  the pulse early.
- If a bridge-disable write fails, the diagnostic attempts to remove shared
  `COIL_EN` power and prevents further pulses. If that also fails, it explicitly
  reports that external power must be removed. The software timeout includes
  elapsed I2C/console time but is not an independent hardware cutoff.
- `disarm` first disables and coasts all bridges, then drives the shared rail
  low.

If an axis says `NOT READY`, do not arm. Check face-board power, harness order,
SDA/SCL continuity, and the expected bus/address. If `fault` differs from `0x02`, record
the byte and inspect the individual fault flags in the flight-software logs
before clearing it. Pressing reset or removing power also returns to the safe
startup path.

## Driver behavior and calibration

- OCP and thermal shutdown latch off. Clearing faults first disables the output,
  zeros the target, and selects coast; another explicit enable is required.
- Stall detection and motor soft-start/soft-stop are disabled. Same-direction
  voltage updates keep driving; a reversal writes coast before target/direction.
  This does not guarantee that inductive coil current has reached zero.
- A reset or mismatched configuration requires `drv8235_reconfigure()` before
  driving again. Clearing faults alone does not restore configuration.
- The 6000 mV software limit caps the nominal target at code 35, approximately
  5857 mV; code 36 would be approximately 6024 mV. Analog tolerances still apply.
- Current calibration is pending hardware confirmation. The existing DT scale
  is unchanged and must not yet be treated as a calibrated current measurement.
  `coil_current_abs` is the sensed coil-current magnitude, not a signed current
  and not battery-input current. Enabled brake retains nonzero IMTR readings.
  Coast, disabled output, reset/fault/unconfigured state, and IMTR above `0xC0`
  report `UNAVAILABLE(raw=...)`, never a fabricated valid zero. Voltage/duty and
  the raw current register remain readable. A valid sensing path does not prove
  the register's sample freshness or the physical calibration.

## Porting differences from CircuitPython

The Python `set_throttle(0)` selects brake, and `set_throttle(None)` selects
coast. This C API keeps `set_output_uv(0)` as coast and provides an explicit
`drv8235_brake()` call; flight tasks must choose the intended stop behavior.
Neither command automatically enables EN_OUT or COIL_EN.

Python negates current when the commanded bridge state is reverse. This C
driver reports unsigned magnitude because IMTR does not encode measured
direction, particularly during braking or a reversal. `current_valid` must be
checked before using `current_ua`; zero with `current_valid=false` is a
placeholder, not a zero-current measurement. The pure conversion helper no
longer accepts a bridge-mode argument.

The provisional current conversion also differs from Python: Python uses
`raw * 0.01451 A` (approximately `3.7/255`), while C uses
`raw/0xC0 * imtr-full-scale-microamp`. The existing full-scale setting is
unchanged pending hardware/TI confirmation; these are not calibrated equivalents.
