# ADM1176

The driver follows the previous CircuitPython driver and the Embassy Rust port.
It starts continuous voltage/current conversion and exposes both measurements
through Zephyr's sensor API. The other functions retain the Python names:

| Python | C |
|---|---|
| `config()` | `adm1176_config()`, using command flags instead of strings |
| `read_voltage_current()` | `sensor_sample_fetch()` and voltage/current channels |
| `set_device_on()`, `device_on()` | `adm1176_set_device_on()`, `adm1176_device_on()` |
| `set_overcurrent_level()`, `overcurrent_level()` | Same names with `adm1176_` prefix; raw 8-bit values |
| `status()`, `clear()` | `adm1176_status()`, `adm1176_clear()` |
| `device_errors` | `adm1176_device_errors()`: report ADC_ALERT, then clear alerts |

The threshold getter is a **software cache**, initially 0xFF, updated after a
successful setter. It does not read hardware registers. The driver owns its
alert mask for the current boot (initially hot-swap only, adding ADC_OC4 when a
threshold is set). CLEAR writes that mask; it does not preserve settings left
by earlier firmware. Applications should reapply their alert policy at startup.
Initialization itself leaves output switching and alert registers unchanged.
Python's empty `reset()` and `deinit()` methods are omitted.

Configuration supports continuous/one-shot, voltage/current-only, and VRANGE.
One-shot reads may return an I2C error until conversion completes; retry the
fetch without reissuing config. Turning on restores continuous V/I, as Python
does. Failed samples return an error; channel reads return `-ENODATA` until
the next successful sample. Status reads restore the configured measurement
command, with a failed restoration retried by the next sample.

Run the automated tests from the C repository root:

```sh
west twister -T tests/drivers/adm1176 -p qemu_cortex_m3 --inline-logs
```

Build the telemetry-only board diagnostic:

```sh
west build -p always -b 'argus@4/rp2350b/m33_0' \
  -s samples/adm1176_diag -d build-adm1176-diag
```

Flash `build-adm1176-diag/zephyr/zephyr.uf2` and open the USB serial console.
The diagnostic reads the main/GPS/radio monitors on I2C1 at 0x40/0x41/0x42
every two seconds, without switching rails or changing alert thresholds.
The 10 milliohm shunts follow the old flight-software configuration; confirm
them on the test board. The C repository uses its own board files and binding.
Compilation and simulated tests do not establish hardware validation.
