import csv
import math
import statistics
import sys
import time
import board
import busio
import csv
import time
# import numpy as np #Varun: I originally used numpy for integration methods, but probably could be written without

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
start_time = time.time_ns()

def throw_detect(time, accel_x, accel_y, accel_z, accel_mag):
    """
    take an array of acceleration values and identify throws
    currently returns nothing, just prints out the samples when the throw occurs
    """
    num_throws = 0

    # constants
    avg_lookback = 10
    min_throw_samples = 10
    forward_len = 10
    forward_skip = 3

    # thresholds
    max_flight_stdev = 3.5
    release_peak_height = 25
    min_throw_jerk = 1
    throw_end_jerk = 5

    start_sample = 0
    release_sample = 0
    end_sample = 0
    in_flight = False

    i = avg_lookback
    while (i < len(time) - forward_len - forward_skip):
        if in_flight:
            # print("in flight")
            # if (i < avg_lookback):
            #     continue
            last_avg = sum(accel_mag[i - avg_lookback : i - 1]) / (avg_lookback - 1)
            if (abs(accel_mag[i] - last_avg) > throw_end_jerk and (i - start_sample) > min_throw_samples):
                in_flight = False
                end_sample = i
                print("Throw finished")
                print("Started at  %d" % start_sample) # start integrating here for velocity
                print("Released at %d" % release_sample) # get release angle here
                # note: release_sample is the local max of acceleration - need to integrate 3 or 4 samples past to get velocity
                # also don't start calculating centripetal acceleration until 3 or 4 samples after release_sample
                print("Finished at %d" % end_sample)
                print("Length: %d\n" % (end_sample - release_sample))
                # TODO: do something with these numbers
                num_throws += 1
        else:
            # print("waiting for throw")
            """ 
            Idea:
            find release when current sample is THRESH more than avg of next + 3 NUM samples and those samples have std dev below THRESH
            find start by stepping back from release
            should find positive jerk
            release is when have found large positive jerk and current jerk is less than THRESH
            """
            next_avg = sum(accel_mag[i + forward_skip : i + forward_skip + forward_len]) / forward_len
            next_stdev = statistics.stdev(accel_mag[i + forward_skip : i + forward_skip + forward_len])

            if (accel_mag[i] - next_avg) > release_peak_height and next_stdev < max_flight_stdev:
                release_sample = i + forward_skip
                in_flight = True
                found = False
                idx = i
                found_positive = False
                i = release_sample + avg_lookback
                while not found:
                    curr_jerk = accel_mag[idx] - accel_mag[idx - 1]
                    if curr_jerk > 0:
                        found_positive = True
                    if found_positive and curr_jerk != 0 and curr_jerk < min_throw_jerk:
                        found = True
                        start_sample = idx
                    idx -= 1
        i += 1

    print("total of %d throws" % num_throws)

#time = []
#accel_x = []
#accel_y = []
#accel_z = []
#accel_mag = []
#with open('mult_throw_1.csv') as f:
#    reader = csv.reader(f)
#    next(reader)
#    for row in reader:
#        time.append(float(row[0]))
#        accel_x.append(float(row[1]))
#        accel_y.append(float(row[2]))
#        accel_z.append(float(row[3]))
#        accel_mag.append(math.sqrt(float(row[1]) ** 2 + float(row[2]) ** 2 + float(row[3]) ** 2))
#
#throw_detect(time, accel_x, accel_y, accel_z, accel_mag)


############
# Note:
# The best way to handle get_next would probably be with multithreading
# one thread could pull data off of the imu at the correct times and add it to the back of an array
# another thread could consume data off the front of that array and do the processing
# I'm not implementing that right now, but it's an option if we need
############

# print(type(time[0]))
# redefine get_next to get the next imu data instead of next array element
# get_next should contain the sleep
i = 0
num_throws = 0
def get_next():
    time.sleep(0.01)
    accel_x, accel_y, accel_z = bno.acceleration
    # also will need to calculate accel_mag here
    return math.sqrt(accel_x**2 + accel_y**2 + accel_z**2) # currently only using these but may modify to use more
    # Varun : We'll need to edit this- Our most accurate integration method came using
    # math.sqrt(accel_x**2 + accel_y**2 + (accel_z - 9.8)**2) to account for gravity
# constants
avg_lookback = 10
min_throw_samples = 10
forward_len = 10
forward_skip = 3

# thresholds
max_flight_stdev = 3.5
release_peak_height = 25
min_throw_jerk = 1
throw_end_jerk = 5
lookback_arr_len = 50

start_sample = 0
release_sample = 0
end_sample = 0
in_flight = False

lookback_arr = []

while len(lookback_arr) < lookback_arr_len:
        curr_accel_mag = get_next()
        lookback_arr.append(curr_accel_mag)

while (True): # run forever for now
    if in_flight:
        curr_throw_len += 1
        last_avg = sum(lookback_arr[len(lookback_arr) - avg_lookback - 1 : len(lookback_arr) - 1]) / avg_lookback
        if (abs(lookback_arr[-1] - last_avg) > throw_end_jerk and curr_throw_len > min_throw_samples):
            in_flight = False
            num_throws += 1
            print("Throw of length %d" % (curr_throw_len))
            # could do something else here if we want
    else:
        # print("checking "+str(i))
        next_avg = sum(lookback_arr[len(lookback_arr) - forward_len - forward_skip : len(lookback_arr) - forward_skip]) / forward_len
        next_stdev = statistics.stdev(lookback_arr[len(lookback_arr) - forward_len : len(lookback_arr) - forward_skip])
        if (lookback_arr[len(lookback_arr) - forward_len - forward_skip] - next_avg) > release_peak_height and next_stdev < max_flight_stdev:
            curr_throw_len = 0
            in_flight = True
            # released at lookback_arr[len(lookback_arr) - forward_skip - forward_len]
            found = False
            idx = len(lookback_arr) - forward_skip - forward_len
            release_idx = idx
            found_positive = False
            invalid = False        

            while not found:
                curr_jerk = lookback_arr[idx] - lookback_arr[idx - 1]
                if curr_jerk > 0:
                    found_positive = True
                if found_positive and curr_jerk != 0 and curr_jerk < min_throw_jerk:
                    found = True
                    start_sample = idx
                idx -= 1
                if (idx <= 0): # could not find the release point
                    invalid = True
                    start_sample = 0
                    break
                
            # curr_throw_len = forward_len + forward_skip
            print(release_idx - start_sample)
            
            # TODO: here is were to integrate lookback_arr from start_sample to release_idx
            # calculate release angle at release_idx
            
            numPoints = release_idx - start_sample + 1 #91
            veloc_mag = {0} * numPoints
            dT = .001 #if we choose to make it constant 
            #If we get time vector, then formula is as follows: dT = time[start_sample : release_idx] - time[start_sample - 1 : release_idx - 1]

            for i in range(0, numPoints-1): 
                veloc_mag[i+1] = veloc_mag[i] + (lookback_arr[i+release_idx+1] * dT) #needs to be dT[i+1] if we use time array
            
            release_velocity = veloc_mag[len(veloc_mag) - 1] #last element of array is final velocity


            # pull enough samples into the array
            # don't need this loop unless avg lookback is larger than forward_len + forward_skip
            # for dummy in range(avg_lookback - (forward_len + forward_skip)):
            #     curr_accel_mag = get_next()
            #     lookback_arr.pop(0)
            #     lookback_arr.append(curr_accel_mag)
            
            # to get centripetal acceleration, we'll need to modify get_next to also return accel_x and accel_y
            # use curr_accel_x and curr_accel_y to calculate centripetal acceleration of the throw

    next_accel_mag = get_next()
    lookback_arr.pop(0)
    lookback_arr.append(next_accel_mag)


    
