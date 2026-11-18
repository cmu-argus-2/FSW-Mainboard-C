# ADCS task

The ADCS task is a port of the Python `tasks/adcs.py` and `apps/adcs`. It runs
at 1 Hz on its own thread. Each run takes a sensor snapshot (magnetometer, gyro,
nine light sensors), publishes an `ADCS` record, updates the ADCS mode and then
runs one blocking control cycle:

| Controller | Cycle |
| --- | --- |
| B-dot | 6 magnetometer samples 80 ms apart, coils on for 500 ms, coils off (0.9 s) |
| B-cross, sun pointing | Gyro re-read and coils updated every 50 ms for 1 s, coils off |

The satellite state gates the task. STARTUP does nothing. LOW_POWER skips
sensing and cuts COIL_EN. DETUMBLING and NOMINAL set the gyro range to
2000 and 125 deg/s respectively. The ACS_OFF and VF_TUMBLING modes also zero
the coils and cut COIL_EN, which is restored before the next control cycle.

The control laws, sun vector, coil allocator and mode logic are pure functions
in `src/apps/adcs`. The task and its runner are in `src/tasks`. Hardware
integration supplies driver-backed callbacks through `adcs_io`. The default
runner uses console-only callbacks that report every device as unavailable.

## Record

`struct adcs_record` holds the 28 fields of Python `ADCS_IDX` with the data
handler format `LBBffffffBfffHHHHHHHHHBBBBBB`. As in Python, it is published
right after the snapshot, so `MODE` and the coil status describe the previous
control cycle. Light sensors are logged as `int(lux / 3) & 0xFFFF`.

Each console record is followed by `ADCS_VALIDITY,<time>,<mask>`. A set bit
means the field holds a measurement. A clear bit marks a zero placeholder.

| Bits | Read group |
| --- | --- |
| 0 | Gyro (read and plausibility check) |
| 1 | Magnetometer (read and plausibility check) |
| 2–10 | XP, XM, YP, YM, ZP_XP, ZP_YM, ZP_XM, ZP_YP, ZM light sensors |

## Differences from Python

- A failed coil command or COIL_EN write is not reported as successful. The
  coil status is set only for a coil whose command was accepted, and a failed
  zero or power-off is retried on the next run.
- The gyro range is written only when it changes or the last write failed.
- Controller mode and magnetometer bias are set at init or through
  `adcs_set_controller` and `adcs_set_mag_bias_ut`. Persisting them to storage
  (Python `/sd/config/*.bin`) is left to the command integration.
- A B-cross or sun-pointing cycle fills the whole 1 s period. The runner then
  starts the next run immediately instead of skipping a period.

## Tests

`tests/tasks/adcs` checks the algorithms against vectors generated from the
Python flight software, and the task against fake hardware and a fake clock.
Regenerate the vectors with a Python environment that has numpy:

```sh
python3 test_vectors/adcs/gen_adcs_vectors.py <FSW-mainboard>/flight \
    > tests/tasks/adcs/src/adcs_vectors.h
west twister -T tests/tasks/adcs -p native_sim/native/64 --inline-logs
```
