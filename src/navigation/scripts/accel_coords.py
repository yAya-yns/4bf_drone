#
# *****************************************************
#                                                    |
#           _     __                    _            |
#    _   _ | |_  / _| _ __            _| |__   __    |
#   | | | || __|| |_ | '__|         / _` |\ \ / /    |
#   | |_| || |_ |  _|| |           | (_| | \ V /     |
#    \__,_| \__||_|  |_|    _____   \__,_|  \_/      |
#                          |_____|                   |
#                                                    |
#                                                    |
# *****************************************************
# file: accel_coords.py
# auth: Daniel Asadi
# desc: list acceleration cartesian coordinates
# usage: python3 src/navigation/scripts/accel_coords.py
# usage with custom args: python3 src/navigation/scripts/accel_coords.py --intervals 50 --velocity 20

import argparse
import csv
import numpy as np
import matplotlib.pyplot as plt
from math import cos, pi, sin, floor


def generateAccelPoints(filename, num_intervals, velocity):
    row_list = [['point', 'type', 'x', 'y', 'vel']]
    count = 1
    point_type = 'START'

    x = np.linspace(0, 0, num_intervals)
    y = np.linspace(0, 75, num_intervals)

    # prep lists for csv
    for i in range(len(x)):
        if i > 0:
            point_type = 'INTERMEDIATE'
        if i == len(x)-1:  # reach 100m breaking zone
            point_type = 'FINISH'
            velocity = 0
        row_list.append([count, point_type, round(
            x[i], 2), round(y[i], 2), velocity])
        count += 1

    # write to csv
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(row_list)

    return [x, y]


def plotAccel(x, y, num_intervals):
    figure, ax = plt.subplots(1)
    xl1 = np.linspace(-1.5, -1.5, 175)
    xl2 = np.linspace(1.5, 1.5, 175)
    yl = np.linspace(0, 175, 175)
    # y = np.linspace(0, 175, num_intervals)
    # accel_points_int_scale = int(num_intervals*(75/175))
    ax.plot(xl1, yl, marker='.', label='track limits', color='black')
    ax.plot(xl2, yl, marker='.', color='black')
    ax.plot(x, y, marker='.', label='acceleration straight', color='green')
    # ax.plot(x[:accel_points_int_scale], y[:accel_points_int_scale], marker='.', label='acceleration straight', color='green')
    # ax.plot(x[accel_points_int_scale:], y[accel_points_int_scale:], marker='.', label='braking zone', color='red')
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_xlim(-20, 20)
    ax.set_ylim(0, 200)

    plt.legend(loc="upper left")
    plt.title('Acceleration coordinates\n' + str(num_intervals) + ' points')
    plt.grid()
    plt.show()


parser = argparse.ArgumentParser()
parser.add_argument("--intervals", type=int,
                    help="Number of intervals for the path", default=74)
parser.add_argument("--velocity", type=float,
                    help="Car velocity in m/s", default=20)
args = parser.parse_args()
filename = 'src/navigation/scripts/accel_coords.csv'
coord_list = generateAccelPoints(filename, args.intervals, args.velocity)
plotAccel(coord_list[0], coord_list[1], args.intervals)
