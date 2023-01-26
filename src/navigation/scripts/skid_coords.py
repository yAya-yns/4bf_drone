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
# file: skid_coords.py
# auth: Daniel Asadi
# desc: list skidpad cartesian coordinates
# usage: python3 src/navigation/scripts/skid_coords.py
# usage with custom args: python3 src/navigation/scripts/skid_coords.py --radius 8.375 --intervals 50 --velocity 20

import argparse
import csv
import numpy as np
import matplotlib.pyplot as plt
from math import cos, pi, sin, floor


def generateSkidpadPoints(filename, num_intervals, velocity, radius):
    row_list = [['point', 'type', 'x', 'y', 'vel']]
    count = 1
    point_type = 'START'

    # optional - entry/exit line (not in csv, only in plot)
    # start_end_x = np.linspace(0, 0, 20)
    # start_end_y = np.linspace(-8, 8, 20)

    # laps 1,2
    angle12 = np.linspace(pi, -3*pi, num_intervals*2)
    x12 = radius+radius*np.cos(angle12)
    y12 = radius*np.sin(angle12)

    # laps 3,4
    angle34 = np.linspace(0, 4*pi, num_intervals*2)
    x34 = -radius+radius*np.cos(angle34)
    y34 = radius*np.sin(angle34)

    # prep lists for csv
    for i in range(len(x12)):
        if i > 0:
            point_type = 'INTERMEDIATE'
        row_list.append([count, point_type, round(
            x12[i], 2), round(y12[i], 2), velocity])
        count += 1
    for i in range(len(x34)):
        if i == len(x34)-1:
            point_type = 'FINISH'
            velocity = 0
        row_list.append([count, point_type, round(
            x34[i], 2), round(y34[i], 2), velocity])
        count += 1

    # write to csv
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(row_list)

    return [x12, y12, x34, y34]


def plotSkidpad(x12, y12, x34, y34, num_intervals):
    figure, ax = plt.subplots(1)
    start_end_x = np.linspace(0, 0, 20)
    start_end_y = np.linspace(-8, 8, 20)
    ax.plot(start_end_x, start_end_y, marker='.', label='entry/exit line')
    ax.plot(x12, y12, marker='.', label='laps 1,2')
    ax.plot(x34, y34, marker='.', label='laps 3,4')
    ax.set_aspect(1)
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')

    plt.legend(loc="upper left")
    plt.title('Skidpad coordinates\n' + str(num_intervals) + ' points/circle')
    plt.grid()
    plt.show()


parser = argparse.ArgumentParser()
parser.add_argument("--radius", type=float,
                    help="Track radius in m", default=8.375)
parser.add_argument("--intervals", type=int,
                    help="Number of intervals per circle", default=50)
parser.add_argument("--velocity", type=float,
                    help="Car velocity in m/s", default=20)
args = parser.parse_args()
filename = 'src/navigation/scripts/skid_coords.csv'
coord_list = generateSkidpadPoints(
    filename, args.intervals, args.velocity, args.radius)
plotSkidpad(coord_list[0], coord_list[1], coord_list[2],
            coord_list[3], args.intervals)
