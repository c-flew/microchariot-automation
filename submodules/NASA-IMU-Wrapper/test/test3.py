import board
import busio
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_lsm6ds import GyroRange, AccelRange, Rate
from adafruit_lis3mdl import LIS3MDL
import time

if __name__ == "__main__":
    i2c = busio.I2C(board.SCL, board.SDA, frequency=6700) 
    imu = LSM6DSOX(i2c)
    mag = LIS3MDL(i2c)

    rate = Rate.RATE_6_66K_HZ
    gyro_range = GyroRange.RANGE_2000_DPS
    accel_range = AccelRange.RANGE_16G

    imu.rate = rate
    imu.gyro_range = gyro_range
    imu.accel_range = accel_range

    while True:
        start = time.monotonic()
        imu.acceleration()
        mag.magnetic()
        imu.gyro()
        print(time.monotonic() - start)
        time.sleep(0.1)

