# Magnetorquer field sweep (C port of mag_hyst.py)

C port of FSW-mainboard `scripts/mag_hyst.py`. For each coil in turn (XP, XM, YP, YM, ZP,
ZM) it steps the throttle from -1.0 to +1.0 in 61 steps, with every other coil at 0
(brake). After each step it waits 200 ms, then prints one CSV row: the BMX160 field and
all six coils' voltages and currents. The columns match `mag_hyst.py`:

```text
coil,step,time_ns,mag_x,mag_y,mag_z,voltage_xp,...,voltage_zm,current_xp,...,current_zm
```

A run takes about 90 s. `python/compare_mag_hyst.py` matches a Python log and a C log row
by row, so you can check that the C drivers (`drv8235.c`, `bmx160.c`) behave like the
Python ones.

## Files

| File | What |
|---|---|
| `src/main.c` | The C port |
| `python/mag_hyst.py` | The Python script to run on the board, see below |
| `python/compare_mag_hyst.py` | Compares the two logs |

`python/mag_hyst.py` is `scripts/mag_hyst.py` with two changes, marked `# added` or
`# changed`:

- **Watchdog.** It toggles the external watchdog. `SATELLITE.boot_sequence()` arms the
  watchdog (`hal/drivers/watchdog.py` drives WDT_EN high), and the original script never
  toggles its input, so the board resets a few seconds into the sweep.
- **Missing coils.** A coil driver that fails to boot is `None` in
  `SATELLITE.TORQUE_DRIVERS`, and the original stops at its first `set_throttle`. This
  version sweeps only the coils that booted and prints `nan` for the rest, so every row
  keeps its 18 columns.

## Differences from the Python script

None of these change the rows:

- **No satellite boot.** The C port only turns on the peripheral rail and COIL_EN, sets
  up the six coil drivers and reads the IMU. The external watchdog is never armed.
- **Start and stop.** It starts 1 s after the capture opens the USB port, and stops if
  the port closes. Each new open runs a new sweep.
- **Chip state.** For each coil driver it prints FAULT_STATUS before and after init, and
  at the first rejected command, with CONFIG0/CONFIG4 read back. A chip that has reset
  is back at its defaults (EN_OUT 0, I2C_BC/PMODE 00).
- **BOOTSEL.** Opening the port at 1200 baud reboots the board into BOOTSEL, so a new
  build can be flashed without holding BOOT.
- **Failed reads.** A failed read prints `nan`. Python would stop with an exception.
- **End of run.** After the final all-zero step it also coasts every coil and turns
  COIL_EN off.

Two driver differences show up in the data:

- **Current scaling.** The C driver uses the EPS team's scale, `3.7 / 192` A per count,
  and Python uses 0.01451 A per count. The C currents are therefore about 33% higher for
  the same register value. The comparison converts both back to register counts.
- **Reversal.** The C driver coasts before switching between forward and reverse. The
  sweep passes through 0 (brake) at the middle step, so this never triggers here.

## Hardware

- **Power.** The coil drivers run from VBATT: connect a battery or a current-limited
  bench supply. USB alone is not enough.
- **Positions.** Put the face panels in their flight positions relative to the
  mainboard. The coil field falls off with the cube of the distance, so panels spread
  out on the bench may give a field change close to the magnetometer noise. Don't move
  anything between the two runs, including nearby steel or cables, and keep any
  Helmholtz cage at one setting.
- **Missing panels.** You can run with only some panels, for example ±X and ±Y. Both
  versions sweep only the coils whose driver answers at start-up, and print `nan` for
  the others. The comparison skips a coil that neither log swept. Connect the same
  panels for both runs: a coil swept in only one log counts as a difference.
- **Safety.** Keep a way to remove power immediately.

## Run

1. **Python.** The board needs CircuitPython with the flight build (`./run.sh` in
   FSW-mainboard). Copy `python/mag_hyst.py` to the board's drive (`/Volumes/ARGUS`) as
   `code.py`. Start the capture, then unplug and replug the board so the header line is
   captured too. Press Ctrl+C after the `Average mag measurement time` line:
   ```bash
   cat /dev/cu.usbmodem<N> | tee python_hyst.log
   ```
   Your port name may differ. If `cat` exits when the board unplugs, start it again
   straight after replugging: the satellite boot takes a few seconds before the first
   row.
2. **C.** Build from the repo. The first build downloads the Board Library, so it needs
   network access:
   ```bash
   west build -p always -b argus@4/rp2350b/m33_0 -d build/drv8235_mag_hyst \
     samples/drv8235_mag_hyst -S argus-common
   ```
   Hold BOOT while plugging in (or, if this sample is already running, open the port at
   1200 baud for 2 s with pyserial, which the Zephyr venv has:
   `python -c "import serial, time; s = serial.Serial('/dev/cu.usbmodem<N>', 1200); time.sleep(2)"`),
   copy `build/drv8235_mag_hyst/zephyr/zephyr.uf2` to the `RP2350` drive, and capture.
   The sweep starts when `cat` opens the port and ends with `DONE`:
   ```bash
   cat /dev/cu.usbmodem<N> | tee c_hyst.log
   ```
3. **Compare:**
   ```bash
   python3 samples/drv8235_mag_hyst/python/compare_mag_hyst.py python_hyst.log c_hyst.log
   ```
4. **Back to CircuitPython:** hold BOOT, plug in, and copy
   `FSW-mainboard/firmware/Mainboard v4/ArgusV4.uf2` to the `RP2350` drive.

## What the comparison checks

Checks for each swept coil:

| Check | Pass |
|---|---|
| Swept coil's voltage (VMTR counts) | bias (mean of C minus Python) within 0.5 count |
| Swept coil's current (IMTR counts) | bias within 1 count or 10% of full scale |
| The other five coils | at most 1 count of voltage (they're braking) |
| Field slope: a straight-line fit of each axis against throttle | C and Python within 1 uT or 10% |

**Why bias:** each row is a single sample, so the voltage and current checks use the
bias instead of single readings. A command that's one code off moves every reading the
same way, while noise averages out.

**Fit:** the slope (the field change from throttle 0 to 1) is what the coil adds at the
magnetometer. The fit's offset is the background field, which can drift between runs, so
it isn't compared. The script also prints each fit's residual (noise plus any
non-linearity). It warns if a coil's slope is close to the noise, which usually means the
panel is too far from the mainboard.

**Exit status:** the script exits with status 1 if anything differs.

I tested the comparison on generated logs. It passes matching logs, and flags a C field
30% too strong on one coil and a C command one code too high on another.
