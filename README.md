# FSW-Mainboard-C

Flight software for the Argus mainboard (RP2350B), built on [Zephyr](https://docs.zephyrproject.org/).

## Setup

This repo is a [west](https://docs.zephyrproject.org/latest/develop/west/index.html) manifest:
`west.yml` pins the Zephyr version and the modules we use. West clones this repo and
fetches Zephyr next to it in a *workspace* folder.

Follow the Zephyr [Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)
for your OS. It installs the dependencies, a Python venv with `west`, and the Zephyr SDK
under `~/zephyrproject`.

Don't forget to activate the venv in every new terminal. You should see `(.venv)` in front of your prompt!

```bash
source ~/zephyrproject/.venv/bin/activate
```

Then download this repo, along with the Zephyr version it uses, into `~/argus`
(use this instead of `git clone`):

```bash
west init -m https://github.com/cmu-argus-2/FSW-Mainboard-C --mr main ~/argus
cd ~/argus
west update
west packages pip --install
```

Resulting layout:

```
argus/
├── FSW-Mainboard-C/   <- this repo
├── zephyr/            <- Zephyr itself
└── modules/           <- libraries Zephyr uses
```

After a `git pull`, also run `west update` in case the Zephyr version changed.

## Building

Building will output a `.uf2` file at `build/zephyr/zephyr.uf2`. On boot the
firmware prints its version, e.g. `Argus FSW mainboard rev 4 (d142ec7a)`. If the
hash includes `-dirty`, that means you had uncommitted changes when you built.

### Default version:

```bash
cd ~/argus/FSW-Mainboard-C
west build                                # default hardware revision
```

### Building a specific revision:
```bash
cd ~/argus/FSW-Mainboard-C
west build -p -b argus@4/rp2350b/m33_0    # argus@4 is a specific mainboard version
```

## Flash

Hold the boot button while plugging the board in, then drag `zephyr.uf2` onto the `RP2350` drive.

## Layout

| Path | What |
|---|---|
| `west.yml` | Zephyr version + module allowlist |
| `boards/cmu/argus/` | Board definition: pins, buses, radio, SD card. One board, one revision per mainboard version. |
| `zephyr/module.yml` | Makes `boards/` visible to the build |
| `CMakeLists.txt`, `prj.conf` | The application |
| `cmake/` | Build helpers (git version header printed at boot) |
| `src/`, `include/argus/` | Application code |
| `tests/`, `test_vectors/` | Tests |
| `samples/drv8235_diag/` | Safe USB-shell diagnostic for all six magnetorquer drivers |

## Hardware revisions

Each mainboard version is a *revision* of the `argus` board, selected with
`-b argus@<N>/rp2350b/m33_0`. The base files in `boards/cmu/argus/` describe
revision 4 (pins from the CircuitPython `pins.c` for Mainboard v4).

App code must not hardcode pins or versions. Use devicetree names instead
(`lora0`, `led-strip`, the `*-gpios` properties on `zephyr,user`, ...) so a new
mainboard only touches `boards/`.

To add a new version, e.g. 5:

1. Add `- name: "5"` under `revisions:` in `boards/cmu/argus/board.yml`.
2. Create `boards/cmu/argus/argus_rp2350b_m33_0_5.overlay` with only what changed
   from revision 4, for example a moved pin:
   ```dts
   &lora {
       busy-gpios = <&gpio0_map 26 GPIO_ACTIVE_HIGH>;
   };
   ```
   Kconfig differences go in `argus_rp2350b_m33_0_5_defconfig`.
3. Build with `-b argus@5/rp2350b/m33_0`. Once v5 is the main hardware, change
   `default:` in `board.yml`.

If a version changes the chip itself (e.g. RP2040 → RP2350), it
can't be a revision; add the new SoC under `socs:` in `board.yml` instead.

Building for a revision that isn't listed fails on purpose (`exact: true`),
so a v5 build can never silently use v4 pins.

## Board notes (revision 4)

- `PERIPH_PWR_EN` (GPIO42) must be driven high before anything on I2C0/I2C1 responds.
- `WDT_EN` (GPIO15) arms the external watchdog; once armed, `WDT_WDI` (GPIO2) must be toggled or the board resets. Nothing drives it by default.
- Polarity of the `*_FLT` and `BATT_ALRT` inputs has not been checked against the schematic.

## DRV8235 magnetorquer driver

The driver follows the original Python and Embassy Rust interfaces: normalized
throttle, volts and signed raw setters/getters, bridge mode, voltage/current
readings and fault clearing. Zero commands brake; `drv8235_coast()` corresponds
to Python's `None`. Getters return `-ENODATA` while coasting. Initialization
finishes with EN_OUT enabled at target zero/coast; the application owns COIL_EN
and calls `drv8235_init()` again after cycling power.

Normalized throttle scales to the DT coil voltage limit (6 V on Argus).
Volts/raw reject commands exceeding that limit; voltage rounding cannot exceed
the largest safe code (35 at 6 V). Reversals coast before changing magnitude.
Voltage and reverse-sign behavior follow Python; current uses
the team's EPS exception, `raw * full_scale_A / 192` (default `raw * 3.7 / 192 A`). Brake does not fabricate zero
current, and coast readings may be stale/unsensed. Current calibration remains
unconfirmed. Active-low NPOR and
negative raw writes are corrected; fault clearing first selects zero/coast.

Boot initializes software only. After enabling the DT `vm-supply`, the application
calls `drv8235_init()` and can retry if the chip was unavailable. A latched FAULT
does not independently block setters; fault handling remains application policy.

The [ztests](tests/drivers/drv8235/README.md) and hardware use the same binding
from the Board Library commit pinned in `cmake/drv8235_board_lib.cmake`.
Their first configure fetches it (network required), or accepts a local checkout
via `-DARGUS_BOARD_LIB=/absolute/path` for offline builds.
Coil resistance, voltage limit, current full scale and supply come from that binding.
The main application retains its in-tree board, while the diagnostic explicitly
selects the shared board. Board files, west manifest and CI are unchanged.

The interactive board test, build/flash steps, expected output, and safe manual
test sequence are in [`samples/drv8235_diag/README.md`](samples/drv8235_diag/README.md).
