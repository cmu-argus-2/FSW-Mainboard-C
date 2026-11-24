# BMX160 demo

Prints the BMX160 IMU's gyro, accelerometer, magnetometer and die temperature twice a
second over the USB serial console. Every 10 seconds it switches the gyro between the two
ranges the ADCS task uses: 2000 deg/s (detumbling) and 125 deg/s (nominal). Use it to
check the driver on real hardware.

The IMU is at 0x68 on I2C0 (`imu` in `boards/cmu/argus/argus.dtsi`). Only SDA and SCL
are wired, so the driver polls; there are no interrupt pins.

## Build and flash

From the repo root:

```bash
west build -p -b argus/rp2350b/m33_0 samples/bmx160_demo
```

If this checkout isn't inside a west workspace, add `-- -DBOARD_ROOT=$PWD` so the
Argus board is found.

Hold BOOT while plugging in the mainboard, then copy `build/zephyr/zephyr.uf2` to the
RP2350 USB drive. Open the new USB serial device, for example:

```bash
screen /dev/tty.usbmodem* 115200
```

## Expected output

The format, with example values from a board lying flat:

```text
=== gyro range -> 2000 deg/s
--- BMX160 demo, tick 0
gyro    0.0021  -0.0013   0.0008 rad/s  (   0.12   -0.07    0.05 deg/s, range 2000)
accel    0.112   -0.087    9.795 m/s^2  |a|  9.796
mag      18.40    -4.12   -44.90 uT     |B|  48.70
temp   27.41 degC  ERR_REG 0x00
```

If init failed, the demo reads the chip ID again once a second and prints one of:

| Line | Meaning |
|---|---|
| `init failed: I2C error at 0x68 (-5)` | Nothing answered. Check `PERIPH_PWR_EN`, the IMU fuse, and run `samples/i2c_scan`. |
| `init failed: chip ID 0xNN` | Something answers but isn't a BMX160 (`0xD8`). `0xD1` is a BMI160, which has no magnetometer. |
| `init failed at boot, but answers now` | The IMU wasn't ready when init ran at boot: a power-up timing problem. |

The driver also logs why init failed, but at boot, before the USB console connects, so
that line is usually missed.

## What to check

1. No `init failed` line, and `ERR_REG 0x00`.
2. Lying still: `|a|` is about 9.81 m/s^2 and the gyro is near 0 (a small bias is normal).
3. Rotating the board: the gyro axis you rotate about responds, with the sign given by the
   right-hand rule. At 125 deg/s the gyro saturates at about 2.18 rad/s, so turn slowly.
   A steady rotation should read the same in both ranges.
4. Flat on a table, turned through a full circle: mag X and Y trace a circle and Z stays
   steady. `|B|` should be roughly 25 to 65 uT (about 50 uT in Pittsburgh). Keep it away
   from laptops, magnets and steel, which distort it.
5. Temperature near room temperature, a few degrees higher once the board warms up.

The magnetometer will also see the magnetorquer coils. Repeat check 4 with a coil pulsed
through `samples/drv8235_diag` to see how large that effect is.

## Comparing with the Python driver

`python/bmx160_compare.py` reads the IMU with the flight `bmx160.py` and prints the same
lines as this demo. `python/compare_logs.py` averages two captured logs and prints them
side by side. Keep the board still, in the same spot and orientation, for both captures.

1. **C driver.** Flash this demo, close `screen`, and capture about a minute (two gyro
   range switches), then Ctrl+C:
   ```bash
   cat /dev/cu.usbmodem<N> | tee c_driver.log
   ```
2. **Python driver.** Flashing the demo replaced CircuitPython, so put it back: hold BOOT,
   plug in, and copy `FSW-mainboard/firmware/Mainboard v4/ArgusV4.uf2` to the RP2350 drive
   (not the `WDT_ON` build, which arms the watchdog). Then run `./run.sh` in `FSW-mainboard`
   to install the flight files, and copy `python/bmx160_compare.py` to the board's drive
   (`/Volumes/ARGUS`, or `/Volumes/CIRCUITPY` if the label didn't apply) as `code.py`.
   It runs instead of `main.py` and touches only the IMU. The flight software turns off
   CircuitPython's auto-reload, so copying the file doesn't start it: unplug and replug
   the board, or press Ctrl+C then Ctrl+D in the serial console. Capture the same way:
   ```bash
   cat /dev/cu.usbmodem<N> | tee python_driver.log
   ```
3. **Compare** on your computer:
   ```bash
   python3 samples/bmx160_demo/python/compare_logs.py c_driver.log python_driver.log
   ```

```text
mag (uT)
  C       mean   18.3504   -4.1000  -44.9000   std    0.2861 ...   n=120
  Python  mean   18.4012   -4.0870  -44.8710   std    0.2790 ...   n=118
  C - Py  diff   -0.0508   -0.0130   -0.0290
```

Both drivers write the same registers and use the same math, so each `diff` should be
about the size of `std` or smaller. A sign flip, a factor of 100 (gauss vs uT), or a
factor of 16 (missing trim) points at a bug. Temperature can differ by a degree or two as
the board warms up. Gyro is grouped by range (`gyro@125`, `gyro@2000`); at rest both
should show the same small bias.

The next `./run.sh` deletes `code.py`, so the board goes back to the flight software.

## Driver notes

The driver (`src/drivers/bmx160.c`) ports `flight/hal/drivers/bmx160.py`: the same init
sequence, register values, scaling and BMM150 compensation. The differences:

- Magnetometer readings come out of the sensor API in gauss, as Zephyr defines it, not uT.
  This demo multiplies by 100.
- After the gyro normal-mode command the driver waits 80 ms (Bosch's figure) instead of
  Python's 8.1 ms.
- Writes are spaced 450 us apart, as the chip needs while a sensor is suspended.
- A gyro range write is read back. If it didn't take, `bmx160_set_gyro_range()` returns
  `-EIO`, and readings keep the scale of the range the chip actually holds.

The register values match what Python writes, which isn't always what its code intends.
Its ODR setters write bare ODR codes, which also clear the filter bits: accel runs at
25 Hz, and gyro at 50 Hz (25 Hz isn't valid for the gyro).

The tests in `tests/drivers/bmx160` run the driver against an emulated BMX160. They check
the magnetometer against values computed by the Python driver's own compensation code.
