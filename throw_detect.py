import csv
import math
import statistics


def throw_detect(time, accel_x, accel_y, accel_z, accel_mag):
    """
    take a stream of acceleration values and identify throws
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
                print("Finished at %d\n" % end_sample)
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


time = []
accel_x = []
accel_y = []
accel_z = []
accel_mag = []
with open('mult_throw_1.csv') as f:
    reader = csv.reader(f)
    next(reader)
    for row in reader:
        time.append(float(row[0]))
        accel_x.append(float(row[1]))
        accel_y.append(float(row[2]))
        accel_z.append(float(row[3]))
        accel_mag.append(math.sqrt(float(row[1]) ** 2 + float(row[2]) ** 2 + float(row[3]) ** 2))

throw_detect(time, accel_x, accel_y, accel_z, accel_mag)

