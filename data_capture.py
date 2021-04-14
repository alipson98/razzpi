from gpiozero import Button
import sys
import time
import board
import busio
import csv
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
bno = BNO08X_I2C(i2c)

bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

fields = (
    "time",
    "accel_x",
    "accel_y",
    "accel_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "mag_x",
    "mag_y",
    "mag_z",
    "quat_i",
    "quat_j",
    "quat_k",
    "quat_real"
)

button = Button(16)

start_time  = time.time_ns()

if len(sys.argv) > 1:
    fp = open(sys.argv[1], "w")
else:
    fp = open("data.csv", "w")

writer = writer=csv.writer(fp, delimiter=',',lineterminator='\n')
writer.writerow(fields)
print('Data collection ready')

def writeData():
    accel_x, accel_y, accel_z = bno.acceleration  # pylint:disable=no-member
    gyro_x, gyro_y, gyro_z = bno.gyro  # pylint:disable=no-member
    mag_x, mag_y, mag_z = bno.magnetic  # pylint:disable=no-member
    quat_i, quat_j, quat_k, quat_real = bno.quaternion  # pylint:disable=no-member

    to_write = (
        time.time_ns() - start_time,
        accel_x,
        accel_y,
        accel_z,
        gyro_x,
        gyro_y,
        gyro_z,
        mag_x,
        mag_y,
        mag_z,
        quat_i,
        quat_j,
        quat_k,
        quat_real

    )
    writer.writerow(to_write)

getData = False;
rejectTimer = 0
while True:

    # press 'space' to start/pause
    if button.is_pressed and rejectTimer == 0:
        rejectTimer = 100
        if (getData):
            print('Data collection paused')
            getData = False
        else:
            print('Data collection started')
            getData = True
    if rejectTimer > 0:
        rejectTimer -= 1
    if getData:
        writeData()

    time.sleep(0.01)

