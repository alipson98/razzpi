# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
import time
import board
import busio
import csv
import os
import enum 
  
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

# creating enumerations using class 
class State(enum.Enum): 
    standby = 1
    recording = 2
    flip = 3

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
start_time  = time.time_ns()

fp = open("data.csv", "w")

writer = writer=csv.writer(fp, delimiter=',',lineterminator='\n')
writer.writerow(fields)


os.system("echo gpio |sudo tee /sys/class/leds/led0/trigger")
os.system("echo 255 |sudo tee /sys/class/leds/led0/brightness")

state = State.standby
timer = 0;
uprighted = False;


while True:
    # TODO: add the sleep back in if we want, but this is to test the write speeds we can theoretically get
    time.sleep(0.01)
    # print("Acceleration:")
    accel_x, accel_y, accel_z = bno.acceleration  # pylint:disable=no-member
    # print("X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x, accel_y, accel_z))
    # print("")

    # print("Gyro:")
    gyro_x, gyro_y, gyro_z = bno.gyro  # pylint:disable=no-member
    # print("X: %0.6f  Y: %0.6f Z: %0.6f rads/s" % (gyro_x, gyro_y, gyro_z))
    # print("")

    # print("Magnetometer:")
    mag_x, mag_y, mag_z = bno.magnetic  # pylint:disable=no-member
    # print("X: %0.6f  Y: %0.6f Z: %0.6f uT" % (mag_x, mag_y, mag_z))
    # print("")

    # print("Rotation Vector Quaternion:")
    quat_i, quat_j, quat_k, quat_real = bno.quaternion  # pylint:disable=no-member
    # print(
    #     "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real)
    # )
    # print("")
    if(state.name = "recording" and mag_z<-15):
        state = State.flip
        os.system("echo 255 |sudo tee /sys/class/leds/led0/brightness")
        break;
    if(mag_z > 0 and state.name == "standby"):
        os.system("echo 0 |sudo tee /sys/class/leds/led0/brightness")
        state = State.recording
    if(state.name = "flip" ):
        timer = timer + 1;
        if(mag_z > 0)
            uprighted = True;
        if(uprighted and mag_z < -15)
            break;
    if(timer > 200):
        timer = 0;
        state = State.recording;
        uprighted = False;


    if(state.name == "recording" or state.name == "flip"):
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

