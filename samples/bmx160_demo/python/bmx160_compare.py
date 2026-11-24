# CircuitPython counterpart of samples/bmx160_demo: reads the IMU with the flight
# driver (flight/hal/drivers/bmx160.py) and prints in the same format as the C demo, so
# the two logs can be compared with compare_logs.py.
#
# Needs the Python flight build on the board (./run.sh in FSW-mainboard), which puts the
# driver in /lib/hal/drivers. Copy this file to the CIRCUITPY drive as code.py; it runs
# instead of main.py. The next ./run.sh removes code.py again.
#
# Only the IMU is touched: no satellite boot, no radio, no coils.

import math
import time

import board
import busio
import digitalio
from hal.drivers.bmx160 import BMX160

IMU_ADDRESS = 0x68  # IMU_I2C_ADDRESS in hal/argus_v4.py
PERIOD_S = 0.5
TICKS_PER_RANGE = 20  # 10 s, as in the C demo
RANGE_2000DPS = 0
RANGE_125DPS = 4
RANGE_NAMES = ["2000", "1000", "500", "250", "125"]
RAD_TO_DEG = 180.0 / math.pi

# As in ArgusV4Power: the peripheral rail must be on before anything on I2C0 answers
periph_pwr = digitalio.DigitalInOut(board.PERIPH_PWR_EN)
periph_pwr.direction = digitalio.Direction.OUTPUT
periph_pwr.value = True
time.sleep(0.1)

i2c = busio.I2C(board.SCL0, board.SDA0, frequency=400000)


def norm(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def open_imu():
    while True:
        try:
            return BMX160(i2c, IMU_ADDRESS)
        except Exception as e:
            print("IMU init failed: %s" % e)
            time.sleep(1)


def switch_range(imu):
    # Same rule as the C demo: from the 500 deg/s boot default this goes to 2000 first
    nxt = RANGE_125DPS if imu.gyro_range == RANGE_2000DPS else RANGE_2000DPS
    imu.gyro_range = nxt  # the setter reads the register back
    failed = "" if imu.gyro_range == nxt else " FAILED"
    print("=== gyro range -> %s deg/s%s" % (RANGE_NAMES[nxt], failed))


def print_sample(imu):
    g = imu.gyro()
    a = imu.accel()
    m = imu.mag()  # uT
    t = imu.temperature()
    err = imu._error_status  # ERR_REG
    gr = imu.gyro_range
    name = RANGE_NAMES[gr] if 0 <= gr < len(RANGE_NAMES) else str(gr)

    # %-formatting with the C demo's printk format strings. (CircuitPython can't join
    # adjacent f-strings, so keep these as plain % formats.)
    d = [x * RAD_TO_DEG for x in g]
    print(
        "gyro  %8.4f %8.4f %8.4f rad/s  (%7.2f %7.2f %7.2f deg/s, range %s)"
        % (g[0], g[1], g[2], d[0], d[1], d[2], name)
    )
    print("accel %8.3f %8.3f %8.3f m/s^2  |a| %6.3f" % (a[0], a[1], a[2], norm(a)))
    print("mag   %8.2f %8.2f %8.2f uT     |B| %6.2f" % (m[0], m[1], m[2], norm(m)))
    flags = (" FATAL" if err & 0x01 else "") + (" DROP_CMD" if err & 0x40 else "")
    print("temp  %6.2f degC  ERR_REG 0x%02x%s" % (t, err, flags))


imu = open_imu()
tick = 0
while True:
    try:
        if tick % TICKS_PER_RANGE == 0:
            switch_range(imu)
        print("--- BMX160 python, tick %d" % tick)
        print_sample(imu)
    except Exception as e:
        print("read failed (%s)" % e)
    tick += 1
    time.sleep(PERIOD_S)
