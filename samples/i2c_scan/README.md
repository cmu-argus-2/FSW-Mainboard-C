# I2C bus scan

Every 5 seconds, probes each address from 0x08 to 0x77 on every I2C bus and prints what
answers next to what the devicetree expects there. Use it in bring-up to tell a missing
or unpowered device from a driver problem.

The scan is read-only: each probe is a 1-byte read, so it writes nothing to any device.
That's why this isn't the Zephyr `i2c scan` shell command. That command asks for a
zero-length write, which the RP2350's DesignWare I2C controller can't send, so the driver
sends one uninitialized byte instead.

No device drivers are built in, so nothing else uses the buses during a scan. The
peripheral 3.3 V rail (`PERIPH_PWR_EN`) still comes up at boot through the `periph_3v3`
regulator.

## Build and flash

From the repo root:

```bash
west build -p -b argus/rp2350b/m33_0 samples/i2c_scan
```

Hold BOOT while plugging in the mainboard, then copy `build/zephyr/zephyr.uf2` to the
RP2350 USB drive. Open the new USB serial device, for example:

```bash
screen /dev/tty.usbmodem* 115200
```

## Expected output

```text
--- I2C scan 4 (0x08-0x77)
i2c0 at 400 kHz: 3 answered, 0 of 3 expected missing
  0x44  light_xm              ti,opt4003        ok
  0x45  light_ym              ti,opt4003        ok
  0x46  light_zm              ti,opt4003        ok
i2c1 at 400 kHz: 9 answered, 4 of 12 expected missing
  0x0b                                          answers, not in devicetree
  0x36  fuel_gauge            maxim,max17205    ok
  ...
  0x54  light_zp_xp           ti,opt4003        MISSING (no response, -EIO)
  ...
```

Each pass lists every address that answered and every device the devicetree expects
there. The kHz figure is the `clock-frequency` requested in the devicetree, not a
measurement of SCL.

| Line | Meaning |
|---|---|
| `ok` | The expected device answered. |
| `MISSING (no response, -EIO)` | Usually nothing answered at that address: not populated, not powered, wrong address, or not connected. The driver also returns `-EIO` when the controller loses arbitration, which is a bus fault. An `ARB lost` log line next to it means the latter. |
| `MISSING (error N)` | The probe failed some other way. See the summary line under that bus. |
| `not probed (scan stopped)` | The scan of that bus stopped before this address. See the `scan stopped` line. |
| `answers, not in devicetree` | Something answered at an address the devicetree doesn't list. The MAX17205 fuel gauge also answers at 0x0B (its shadow RAM), so that one is expected. |
| `N probes failed (first error: -116, transfer timed out)` | N probes failed with something other than `-EIO`. Only the first error is named; the others may differ, so check each address. `-116` is a timeout, `-62` means the controller saw SDA or SCL stuck low, `-16` means the bus was busy. Look for a fault on that bus. |
| `scan stopped at 0xNN after 3 timeouts in a row` | Each timeout takes 500 ms, so the scan gives up on a bus after three in a row instead of spending about a minute on it. The rest of that bus isn't probed this pass. |

The I2C driver's `NACK on ...` log lines are turned off in `prj.conf`, since every empty
address produces one. Its `ARB lost` and `Stuck Low` lines still print. If every device on a
bus shows `MISSING (no response, -EIO)` and `ARB lost` lines appear around the scan,
something is holding SDA low.
